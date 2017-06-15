import os
import gc
from glob import glob
import numpy as np

import cv2

from sklearn.decomposition import IncrementalPCA
from sklearn.svm import LinearSVC
from sklearn.multiclass import OneVsRestClassifier
from sklearn.mixture import GaussianMixture
from sklearn.metrics import accuracy_score, confusion_matrix
from skimage.transform import resize
from datetime import datetime
import pickle
from skimage.color import rgb2gray
from cyvlfeat.sift import dsift, sift
from cyvlfeat.gmm import gmm
from cyvlfeat.fisher import fisher

class LocalFeatureClassifier:
    '''
    LocalFeatureClassifier define a image classifier based on local feature extractor. The local keypoints are encoded into a fisher vector to represent the entire image.
    
Usage:
----------
clf = LocalFeatureClassifier(feature_extractor=sift_extractor)
clf.train(path/to/data/dir/,
          gap=10,           # take one tenth data for training,
          tmp_dir=None,     # assign a tmp dir to store the model,
          ngmm=128,         # number of Gaussian components in gmm
          npca=64,          # number of dimension after compression
          maxiter=100,      # number of iteration training gmm
          ifValidate=True)  # if validate the classifier using other frames on rotary table

clf.test(path/to/test/data/)

# to recognize an object given a numpy array
pred = clf.predict(img)
# to see the label
label = clf.CATEGORIES[pred]
    '''
    def __init__(self, feature_extractor):
        self.feature_extractor=feature_extractor
        return
    
    def train(self, data_dir,
              gap=10,
              tmp_dir=None,
              ngmm=128,
              npca=64,
              maxiter=100,
              ifValidate=True):
        
        self.data_dir = data_dir
        self.gap = gap
        self.tmp_dir = tmp_dir
        self.ngmm = ngmm
        self.npca = npca
        self.maxiter = maxiter

        # create tmp dir
        if tmp_dir is None:
            self.tmp_dir = "./apc_cyvlfeat_dsift_gmm{}gap{}pca{}".format(self.ngmm, self.gap, self.npca)
        if not os.path.exists(self.tmp_dir):
            os.mkdir(self.tmp_dir)

        print(self.nowStr_() + "training started")
        # load data
        data, labels, self.CATEGORIES = self.dataLoader_()

        self.REV_CATEGORIES = {v:k for (k,v) in self.CATEGORIES.items()}
        # extract local features
        features_train = self.stack_feature_extractor_(data)
        print(self.nowStr_() + "features extraction completed")

        # postpreprocessing
        # If the image contains no keypoints at all, although this is not likely with dsift.
        labels = [label for i, label in enumerate(labels) if features_train[i] is not None]
        features_train = [feat for i, feat in enumerate(features_train) if features_train[i] is not None]
        # filter out images #keypoints < #components of gmm
        labels = [label for i, label in enumerate(labels) if len(features_train[i]) > self.ngmm]
        features_train = [feat for feat in features_train if len(feat) > self.ngmm]
        print(self.nowStr_() + "%d samples are used for training" %len(features_train))

        gc.collect()
        labels = np.array(labels)

        # pca
        self.pca = IncrementalPCA(n_components=64)
        self.pca.fit(np.concatenate(features_train))
        features_train_compressed = np.array([self.pca.transform(np.array(feat)) for feat in features_train])

        print(self.nowStr_() + "PCA completed")

        # gmm
        gmm_pkl = "gmm{}p{}k{}.pkl".format(self.ngmm, self.npca,  len(self.CATEGORIES))

        print(self.nowStr_() + "train gmm")
        self.gmm_model = self.train_gmm_(np.concatenate(features_train_compressed))
        print(self.nowStr_() + gmm_pkl + " is trained")
        
        X_train = np.array(self.fisher_vector_encoder_(features_train_compressed))
        y_train = labels
        print(self.nowStr_() + "gmm training completed")

        self.clf = OneVsRestClassifier(LinearSVC()).fit(X_train, y_train)
        print(self.nowStr_() + "classifier is trained")

        if ifValidate:
            print(self.nowStr_() + "validation started")
            self.validate(offset=self.gap//2)            
        return

    def validate(self, offset):
        data, labels, _ = self.dataLoader_(offset)
        features_test = self.stack_feature_extractor_(data)
        print(self.nowStr_() + "features extraction completed")

        labels = [label for i, label in enumerate(labels) if features_test[i] is not None]
        features_test = [feat for i, feat in enumerate(features_test) if features_test[i] is not None]

        labels = [label for i, label in enumerate(labels) if len(features_test[i]) > self.ngmm]
        features_test = [feat for i, feat in enumerate(features_test) if len(features_test[i]) > self.ngmm]
        print(self.nowStr_() + "%d samples are used for testing" %len(features_test))

        gc.collect()
        features_test_compressed = np.array([self.pca.transform(np.array(feat)) for feat in features_test])
        print(self.nowStr_() + "PCA completed")
        X_test = np.array(self.fisher_vector_encoder_(features_test_compressed))
        y_test = labels

        preds = self.clf.predict(X_test).astype(np.uint8)
        print(self.nowStr_() + "validation accuracy= {}".format(accuracy_score(y_test, preds)))
        return
    
    def test(self, test_dir=None):
        data = self.testLoader_(test_dir)
        preds = []
        labels = []
        for img, cat in data:
            if img.dtype is not np.uint8:
                if np.max(img) > 1:
                    if np.max(img) > 1:
                        img = img.astype(np.uint8)
                    else:
                        img = (img * 256).astype(np.uint8)
                    labels.append(self.CATEGORIES[cat])
                    preds.append(self.predict(img))
                    labels = np.array(labels)
                    preds = np.array(preds)
                    print(img)
        # print(preds)
        # print(labels)
        # print("Accuracy: {}".format(accuracy_score(labels, preds)))
        # print("")
        # print("Confusion Matrix:")
        # print(confusion_matrix(labels, preds))
        return
    
    def predict(self, img):
        feat = self.feature_extractor(img)
        feat_compressed = self.pca.transform(feat)
        X = fisher(feat_compressed.T.astype(np.float32), 
                       self.gmm_model["means"].T, 
                       self.gmm_model["covariances"].T, 
                       self.gmm_model["weights"],
                       normalized=True, 
                       square_root=True,
                       improved=True)
        return self.clf.predict(X.reshape([1, -1])).astype(np.uint8)[0]
            
    def dataLoader_(self, offset=0):
        dname = "datasetg{}o{}.pkl".format(self.gap, offset) #dataset
        if not os.path.exists(os.path.join(self.tmp_dir, dname)):
            assert(offset < self.gap)
            CATEGORIES = {}
            files = glob(self.data_dir + "/*.npy")
            data = []
            labels = []
            cat_indx = 0
            for f in files:
                _, fname = os.path.split(f)
                basename, _ = os.path.splitext(fname)

                stack = np.load(f).astype(np.uint8)[offset::self.gap, :, :]
                cat = basename[:-5]
                if cat not in CATEGORIES:
                    CATEGORIES[cat] = cat_indx
                    cat_indx += 1
                data.append(self.prep_image_stacks_(stack))
                labels.append(CATEGORIES[cat] * np.ones(stack.shape[0]))

        print(self.nowStr_() + "training images loaded")
        return np.concatenate(data), np.concatenate(labels), CATEGORIES

    def testLoader_(self, test_dir):
        files = glob(test_dir + "/*.npy")
        print(self.nowStr_() + "%d test images loaded" %len(files))
        fname_list = os.listdir(test_dir)
        for f in files:
            if "seg" not in f:
                if "lab" not in f:
                    _, fname = os.path.split(f)
                    basename, _ = os.path.splitext(fname)
                    segments = np.load([f for f in files if (basename in f and 'seg' in f)][0])
                    labels = np.load([f for f in files if (basename in f and 'lab' in f)][0])
                    img = np.load(f)
                    for seg, label in zip(segments, labels):
                        seg = np.array(seg)
                        if len(seg.shape) < 2:
                            continue
                        else:
                            low0 = np.min(seg[:, 0])
                            hig0 = np.max(seg[:, 0])+1
                            low1 = np.min(seg[:, 1])
                            hig1 = np.max(seg[:, 1])+1
                            patch = np.zeros([hig0 - low0, hig1 - low1, 3])
                            for p in seg:
                                patch[p[0] - low0, p[1] - low1, :] = img[p[1], p[0], :]
                                
                            yield (self.img_prep_(patch), label)

    def img_prep_(self, img):
           return resize(img, (224, 224), mode="reflect").astype(np.uint8)
        
    def prep_image_stacks_(self, stack, offset=70):
        varproj=np.var(stack,0)[:,:,0] #only need to look at one channel
        mask=varproj>np.mean(varproj)*.99 #where is there motion?
        n, h, w, _ = stack.shape
        c=np.nonzero(mask)
        c1=np.median(c[1]) #width 
        c0=np.median(c[0])-offset #height
        sig1=np.std(c[1])*2
        sig0=np.std(c[0])*2
        sig=np.max([sig1,sig0]) #take the larger sig, construct square
        
        #bounding box extremes
        low0  = np.int32(max(c0-sig, 0))
        high0 = np.int32(min(c0+sig, h))
        low1  = np.int32(max(c1-sig, 0))
        high1 = np.int32(min(c1+sig, w))
        #change aspect ratio: turns out giving better generalization
        stack_new=[resize(im,(224,224), mode="reflect") for im in stack[:,low0:high0,low1:high1]]
        stack_new=np.stack(stack_new,0)
        return stack_new

    def stack_feature_extractor_(self, stack):
        return [self.feature_extractor((img * 256).astype(np.uint8)) for img in stack]
    
    def train_gmm_(self, features):
        means, covars, weights, ll, posteriors = gmm(features, n_clusters=self.ngmm, max_num_iterations=self.maxiter)
        params = {}
        params["means"] = means.astype(np.float32)
        params["covariances"] = covars.astype(np.float32)
        params["weights"] = weights.astype(np.float32)
        return params
    
    def fisher_vector_encoder_(self, features):
        return [fisher(feat.T.astype(np.float32), 
                       self.gmm_model["means"].T, 
                       self.gmm_model["covariances"].T, 
                       self.gmm_model["weights"],
                       normalized=True, 
                       square_root=True,
                       improved=True) for feat in features]

    def save(self):
        model = {}
        model["CATEGORIES"] = self.CATEGORIES
        model["pca"] = self.pca
        model["gmm"] = self.gmm_model
        model["clf"] = self.clf
        with open(os.path.join(self.tmp_dir, "model.pkl"), "wb") as f:
            pickle.dump(model, f)
            print(self.nowStr_() + "model.pkl is saved in " + self.tmp_dir)
    def load(self, tmp_dir):
        with open(os.path.join(tmp_dir, "model.pkl"), "rb") as f:
            model = pickle.load(f)
            self.CATEGORIES = model["CATEGORIES"]
            self.pca = model["pca"]
            self.gmm_model = model["gmm"]
            self.clf = model["clf"]
            print(self.nowStr_() + "model.pkl is loaded from " + tmp_dir)
    def nowStr_(self):
        now = datetime.now()
        return ("-- [%02d:%02d:%02d]  " %(now.hour, now.minute, now.second))

def dsift_extractor(img):
    if len(img.shape)==3:
        img = rgb2gray(img)
    kps, des = dsift(img, step=5, size=10)
    return des

def sift_extractor(img):
    if len(img.shape)==3:
        img = rgb2gray(img)
    kps, des = sift(img, compute_descriptor=True)
    return des

def opencv_sift_extractor(img):
    sift = cv2.xfeatures2d.SIFT_create()
    _, des = sift.detectAndCompute(img, None)
    return des

if __name__ == "__main__":
    data_dir = "/home/ubuntu/data/apc_data/"
    test_data_dir = "/home/ubuntu/data/tote_shelf/"
    clf = LocalFeatureClassifier(feature_extractor=opencv_sift_extractor)
    clf.train(data_dir, gap=10, ngmm=128, npca=64, maxiter=100)
    clf.save()
    clf.test(test_data_dir)
