import os
from copy import deepcopy
from functools import partial
from collections import Counter

import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches


from sklearn.cluster import DBSCAN
sift = cv2.xfeatures2d.SIFT_create()


class channel:
    def __init__(self, kps=(), dists=(), angs=(), 
                 des=np.array(np.zeros([0, 128]), dtype='float32'), 
                 labels=(), sources=(), kps_ref=()):
        '''
        Warning: shallow copy for initialization.
        '''
        self.kps = tuple(kps)
        self.dists = tuple(dists)
        self.angs = tuple(angs)
        self.des = des
        self.labels = tuple(labels)
        self.sources = tuple(sources)
        self.kps_ref = tuple(kps_ref)
        if len(kps_ref) != 0:
            assert(len(kps) == len(kps_ref))
        return
    def copy(self):
        '''
        Copy method copy contents of itself and return a 
        new channel object.
        '''
        kps = deepcopy(self.kps)
        dists = deepcopy(self.dists)
        angs = deepcopy(self.angs)
        des = deepcopy(self.des)
        labels = deepcopy(self.labels)
        sources = deepcopy(self.sources)
        kps_ref = deepcopy(self.kps_ref)
        return channel(kps=kps, dists=dists, angs=angs, 
                       des=des, labels=labels, 
                       sources=sources, kps_ref=kps_ref)
    def merge(self, ch):
        '''
        Warning: The merge method shallow copy contents of 
        ch. It is assumed that the user will not use the 
        merged channel in the future.
        '''
        self.kps += ch.kps
        self.dists += ch.dists 
        self.angs += ch.dists
        self.des = np.vstack([self.des, ch.des])
        self.labels  += ch.labels
        self.sources  += ch.sources
        self.kps_ref += ch.kps_ref
        return
    
    def select(self, ind):
        '''
        Warning: select is shallow copy. Ind is a integer tuple of index.
        '''
        if len(self.kps) is not 0: kps = [self.kps[i] for i in ind]
        if len(self.dists) is not 0: dists = [self.dists[i] for i in ind]
        if len(self.angs) is not 0: angs = [self.angs[i] for i in ind]
        if len(self.sources) is not 0: sources = [self.sources[i] for i in ind]
        if len(self.labels) is not 0: labels = [self.labels[i] for i in ind]
        if np.size(self.des, 0) is not 0: des = np.asarray([self.des[i,:] for i in ind], dtype='float32')
        if len(self.kps_ref) is not 0: kps_ref = [self.kps_ref[i] for i in ind]
        return channel(kps=kps, dists=dists, angs=angs, des=des, labels=labels, sources=sources, kps_ref=kps_ref)

class item:
    def __init__(self, name, channel, eps=50, min_samples=5):
        '''
        an item object is instantiated with a name and a channel.
        eps and min_sample can be determined by size of source image
        '''
        self.name = name
        self.exists = False
        self.pt_c = np.asarray([0, 0])
        self.source = 'None'
        self.scale = 1
        self.orientation = 0
        self.pt_win = np.zeros([0,2])
        self.pt_raw = np.zeros([0,2])
        ch = self.valify(channel, eps, min_samples)
        ch = self.cluster(ch, 10, 5)
        self.source = self.voteForSource(ch.sources)
        return
    
    def valify(self, ch, eps, min_samples):
        first_presence_flag = [i for i, kp in enumerate(ch.kps) if kp not in ch.kps[:i]]
        ch = ch.select(first_presence_flag)
        if len(ch.kps) is 0:
            return ch
        pts = [kp.pt for kp in ch.kps]
        pt_raw, ind = self.voteForKps(pts, eps=eps, min_samples=min_samples)
        if len(ind) is 0:
            return ch
        ch = ch.select(ind)
        self.pt_raw = [kp.pt for kp in ch.kps]
        return ch
    
    def cluster(self, ch, eps, min_samples):
        if len(ch.kps) is 0:
            return ch
        self.scale = np.median([kp.size / kp_ref.size for kp, kp_ref in zip(ch.kps, ch.kps_ref)])
        self.centers = self.computeCenters(ch)
        pt_c, win = self.voteForKps(self.centers, eps=eps, min_samples=min_samples)
        if len(win) == 0:
            return ch
        ch.select(win)
        self.exists = True
        self.pt_c = pt_c
        self.pt_win = [kp.pt for kp in ch.kps]
        self.orientation = np.median([(kp1.angle - kp2.angle)%360 for kp1, kp2 in zip(ch.kps, ch.kps_ref)])
        return ch
    
    def computeCenters(self, ch):
        centers = []
        for i, kp in enumerate(ch.kps):
            x = kp.pt[0]
            y = kp.pt[1]
            ang = kp.angle + ch.angs[i]
            if ang < 0:
                ang += 360
            elif ang > 360:
                ang -=360
            x_c = x + self.scale * ch.dists[i] * np.cos(np.deg2rad(ang))
            y_c = y + self.scale * ch.dists[i] * np.sin(np.deg2rad(ang))
            centers.append([x_c, y_c])
        centers = np.asarray(centers)
        return centers
    
    
    def voteForCluster(self, pts, eps=100, min_samples=5):
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(pts)
        labels = db.labels_.astype('bool')
        labels = [not lb for lb in labels]
        if True not in labels:
            return [None, None, labels]
        XC = [pt[0] for i,pt in enumerate(pts) if labels[i]]
        YC = [pt[1] for i,pt in enumerate(pts) if labels[i]]
        return [XC, YC, labels]

    def voteForKps(self, pts, eps=100, min_samples=5):
        XC, YC, labels = self.voteForCluster(pts, eps=eps, min_samples=min_samples)
        ind = [i for i,v in enumerate(labels) if v]
        if len(ind) > 0:
            return [[np.median(XC), np.median(YC)], ind]
        else:
            return [[0, 0], ind]

    def voteForSource(self, sources):
        if len(sources) is 0:
            return 'None'
        c = Counter(sources)
        return c.most_common(1)[0][0]

    def makeBox(self):
        im = cv2.imread(self.source,0)
        r, c = im.shape
        dist = np.linalg.norm([r, c]) / 2 * self.scale
        theta = np.arctan2(r, c)
        orientation = self.orientation /180 * np.pi
        xc, yc = self.pt_c
        verts = [(dist * np.cos(theta + orientation) + xc, dist * np.sin(theta + orientation) + yc),
                 (dist * np.cos(-theta + orientation) + xc, dist * np.sin(-theta + orientation) + yc),
                 (dist * np.cos(-np.pi + theta + orientation) + xc, dist * np.sin(-np.pi + theta + orientation) + yc),
                 (dist * np.cos(np.pi - theta + orientation) + xc, dist * np.sin(np.pi - theta + orientation) + yc),
                 (dist * np.cos(theta + orientation) + xc, dist * np.sin(theta + orientation) + yc)]

        codes = [Path.MOVETO,
                 Path.LINETO,
                 Path.LINETO,
                 Path.LINETO,
                 Path.CLOSEPOLY]

        path = Path(verts, codes)
        return path

class siftSegment:
    def __init__(self, imread=myImread, Nchannels=3):
        self.imread = imread
        self.Nchannels = Nchannels
        self.chs_tr = [];
        self.classes = []
        return
    def train(self, root, cls=list()):
        for i in range(self.Nchannels):
            ch, classes = self.trainOnChannel(root, i)
            self.chs_tr.append(ch)
            self.classes.append(classes)
        if len(cls) is 0:
            self.classes = [cls for i, cls in enumerate(self.classes)if cls not in self.classes[:i]][0]
        else:
            self.classes = [cls]
        return
    
    def predict(self, imgTest, eps=50, min_samples=5):
        #matching
        chs2 = []
        for i in range(self.Nchannels):
            ch = self.sift_detect(imgTest[:,:,i], -1, test)
            chs2.append(ch)
        ch_matched, _ = self.match(self.chs_tr, chs2)
        objs = []
        for i in np.unique(np.array(ch_matched.labels, dtype='int')):
            ind = np.where(np.array(ch_matched.labels, dtype='int') == i)[0]
            channel = ch_matched.select(ind)
            obj = item(self.classes[i], channel, eps=eps, min_samples=min_samples)
            objs.append(obj)
        return objs
    
    def trainOnChannel(self, root, ch):
        sift = cv2.xfeatures2d.SIFT_create()
        kps = []
        des = np.array(np.zeros([0, 128]), dtype='float32')
        labels = []
        sources = []
        classes = []

        dists = []
        angs = []
        _ = 0;
        for dir, subs, files in os.walk(root):
            if subs == []:
                classes.append(dir.replace(root,''))
                for f in files:
                    if 'png' or 'jpg' in f:
                        img = self.imread(os.path.join(dir, f), ch)
                        if img is None:
                            continue
                        kp, de = sift.detectAndCompute(img,None)
                        dist, ang = self.computeVector(kp, img)
                        #update
                        kps += kp
                        des = np.vstack([des, de])
                        labels += [_] * len(kp)
                        sources  += [os.path.join(dir, f)] * len(kp)                    
                        dists += dist
                        angs += ang
                _ += 1
        assert len(kps) == np.size(des, 0)
        assert len(labels) == np.size(des, 0)
        return channel(kps=kps, dists=dists, angs=angs, des=des, labels=labels, sources=sources), classes

    def computeVector(self, kps, img):
        dists = []
        angs = []
        for kp in kps:
            x = kp.pt[0]
            y = kp.pt[1]
            r, c = img.shape
            r_x = c//2 - kp.pt[0]
            r_y = r//2 - kp.pt[1]
            dist = np.linalg.norm([r_x, r_y])
            ang = (np.degrees(np.arctan2(r_y, r_x)) - kp.angle) % 360
            dists.append(dist)
            angs.append(ang)
        dists = dists
        angs = angs
        return [dists, angs]
    
    def sift_detect(self, img, ind, source):
        sift = cv2.xfeatures2d.SIFT_create()
        kp, de = sift.detectAndCompute(img,None)
        dist, ang = self.computeVector(kp, img)
        label = [ind] * len(kp) # label for test_img is -1.
        return channel(kps=kp, dists=dist, angs=ang, des=de, labels=label, sources=source)
    
    def flannMatch(self, des1, des2):
        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary

        flann = cv2.FlannBasedMatcher(index_params,search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # Need to draw only good matches, so create a mask
        matched = []
        # ratio test as per Lowe's paper
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.7*n.distance:
                matched.append([m.trainIdx, m.queryIdx])

        matched = np.array(matched).astype(int)
        return matched

    def match(self, chs1, chs2):
        assert(len(chs1) == len(chs2))
        ch_matched = channel()
        for ch1, ch2 in zip(chs1, chs2):
            matched = self.flannMatch(ch1.des, ch2.des)
            if len(matched) is 0:
                return channel(), np.array([])
            kps = tuple([ch2.kps[ind] for ind in matched[:,0]])
            kps_ref = tuple([ch1.kps[ind] for ind in matched[:,1]])        

            dists = tuple([ch1.dists[ind] for ind in matched[:,1]])
            angs = tuple([ch1.angs[ind] for ind in matched[:,1]])
            des = np.asarray([ch2.des[ind] for ind in matched[:, 0]], dtype='float32')

            labels = tuple([ch1.labels[ind] for ind in matched[:,1]])
            sources = tuple([ch1.sources[ind] for ind in matched[:,1]])
            ch = channel(kps=kps, dists=dists, angs=angs, 
                       des=des, labels=labels, sources=sources,
                       kps_ref=kps_ref)
            ch_matched.merge(ch)
        return ch_matched, matched

def myImread(path, ch_ind):
    img = cv2.imread(path)
    return img[:,:,ch_ind]

if __name__ == "__main__":
    root = '../data/items_300/'
    NCHANNELS = 3
    seg = siftSegment(imread=myImread, Nchannels=NCHANNELS)
    seg.train(root)
    test = '../data/test_1000/test1.jpg'
    test_img = cv2.imread(test)
    plt.imshow(test_img[:,:,::-1])
    objs = seg.predict(test_img)
    objs_exists = [obj for obj in objs if obj.exists]
    names = [obj.name for obj in objs_exists]
    for obj in objs_exists:
        pts = np.array(obj.pt_win)
        plt.scatter(pts[:,0], pts[:,1])
    plt.legend(names)
