import numpy as np
import cv2
import os
import time
import sys
import json

import theano
import theano.tensor as T

import skimage.transform
import sklearn.cross_validation
import pickle

import scipy.io
from scipy.ndimage.interpolation import rotate

import random
import scipy.misc

import lasagne
from lasagne.layers import InputLayer
from lasagne.layers import Conv2DLayer as ConvLayer
from lasagne.layers import BatchNormLayer
from lasagne.layers import Pool2DLayer as PoolLayer
from lasagne.layers import NonlinearityLayer
from lasagne.layers import ElemwiseSumLayer
from lasagne.layers import DenseLayer
from lasagne.nonlinearities import rectify, softmax

global BATCH_SIZE

def mask_fun(input_lab_image):

    meanY0 = input_lab_image[400:410,0:10,0].mean()
    meanU0 = input_lab_image[400:410,0:10,1].mean()
    meanV0 = input_lab_image[400:410,0:10,2].mean()

    meanY1 = input_lab_image[100:110,0:10,0].mean()
    meanU1 = input_lab_image[100:110,0:10,1].mean()
    meanV1 = input_lab_image[100:110,0:10,2].mean()

    t0 = (input_lab_image - [meanY0,meanU0,meanV0])
    t1 = (input_lab_image - [meanY1,meanU1,meanV1])


    mask0 = np.sqrt(t0[:,:,0]**2 + t0[:,:,1]**2 + t0[:,:,2]**2)
    mask0 = np.where(mask0 > 50,0,1).astype('uint8')

    mask1 = np.sqrt(t1[:,:,0]**2 + t1[:,:,1]**2 + t1[:,:,2]**2)
    mask1 = np.where(mask1 > 50,0,1).astype('uint8')

    mask = np.logical_or(mask0, mask1) != 1
    mask = mask.astype('uint8')

    connectivity = 8
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity, cv2.CV_32S)
    min_dist = 10000
    best = -1
    for c,centroid in enumerate(centroids):
        if labels[labels == c].any():
            if stats[c][4] > 200:
                dist = np.sqrt( ((centroid - [320,240])**2).sum())
                if dist < min_dist:
                    min_dist = dist
                    best = c

    mask2 = (labels==best).astype('uint8')


    return mask2

def read_images_segment(root_dir):

    with open('/home/motion/Desktop/reactor/db/items.json') as data_file:
            jsonnames = json.load(data_file)
    names=[]
    for key,value in jsonnames.items():
        names.append(key)
    names.sort()
    objectnames = [x.lower() for x in names]
    #read in ever nth image
    output_dir = root_dir + '/segmented/'

    num_images = []
    for name in objectnames:
        try:
            os.mkdir(output_dir + name)
        except:
            pass
        num_images.append(0)

    skip = 4

    imlist=list()
    imlabels=list()
    for folder in os.listdir(root_dir):
        fnames = sorted(os.listdir(root_dir +'/'+ folder))
        #determine which item this is
        item = [folder.startswith(x) for x in objectnames]
        item = np.where(item)[0]
        if len(item) == 0:
            print("Could not find object {}".format(folder))
            continue
        item = item[0]
        print("Processing item {}".format(folder))
        for i in range(1,len(fnames),skip):
            #read in the image
            im = cv2.imread(root_dir + folder + "/" + fnames[i])
            #convert to lab
            im_lab = cv2.cvtColor(im, cv2.COLOR_BGR2LAB)

            #skip blank images
            if np.all(im == 0):
                continue

            mask = mask_fun(im_lab).astype('uint8')
            if fnames[i].startswith('1'):
                mask[0:40,:] = 0


            masked_bgr = np.zeros(im.shape).astype('uint8')
            masked_bgr[:,:,0] = im[:,:,0]*mask
            masked_bgr[:,:,1] = im[:,:,1]*mask
            masked_bgr[:,:,2] = im[:,:,2]*mask

            #write it out
            #cv2.imwrite(output_dir + objectnames[item] + "/" + str(num_images[item]) + ".png",masked_bgr)

            num_images[item] += 1
            imlist.append(masked_bgr)
            imlabels.append(item)

    return imlist,imlabels



def build_simple_block(incoming_layer, names,
                       num_filters, filter_size, stride, pad,
                       use_bias=False, nonlin=rectify):
    """Creates stacked Lasagne layers ConvLayer -> BN -> (ReLu)
    Parameters:
    ----------
    incoming_layer : instance of Lasagne layer
        Parent layer
    names : list of string
        Names of the layers in block
    num_filters : int
        Number of filters in convolution layer
    filter_size : int
        Size of filters in convolution layer
    stride : int
        Stride of convolution layer
    pad : int
        Padding of convolution layer
    use_bias : bool
        Whether to use bias in conlovution layer
    nonlin : function
        Nonlinearity type of Nonlinearity layer
    Returns
    -------
    tuple: (net, last_layer_name)
        net : dict
            Dictionary with stacked layers
        last_layer_name : string
            Last layer name
    """
    net = []
    net.append((
            names[0],
            ConvLayer(incoming_layer, num_filters, filter_size, stride, pad,
                      flip_filters=False, nonlinearity=None) if use_bias
            else ConvLayer(incoming_layer, num_filters, filter_size, stride, pad, b=None,
                           flip_filters=False, nonlinearity=None)
        ))

    net.append((
            names[1],
            BatchNormLayer(net[-1][1])
        ))
    if nonlin is not None:
        net.append((
            names[2],
            NonlinearityLayer(net[-1][1], nonlinearity=nonlin)
        ))

    return dict(net), net[-1][0]


def build_residual_block(incoming_layer, ratio_n_filter=1.0, ratio_size=1.0, has_left_branch=False,
                         upscale_factor=4, ix=''):
    """Creates two-branch residual block
    Parameters:
    ----------
    incoming_layer : instance of Lasagne layer
        Parent layer
    ratio_n_filter : float
        Scale factor of filter bank at the input of residual block
    ratio_size : float
        Scale factor of filter size
    has_left_branch : bool
        if True, then left branch contains simple block
    upscale_factor : float
        Scale factor of filter bank at the output of residual block
    ix : int
        Id of residual block
    Returns
    -------
    tuple: (net, last_layer_name)
        net : dict
            Dictionary with stacked layers
        last_layer_name : string
            Last layer name
    """
    simple_block_name_pattern = ['res%s_branch%i%s', 'bn%s_branch%i%s', 'res%s_branch%i%s_relu']

    net = {}

    # right branch
    net_tmp, last_layer_name = build_simple_block(
        incoming_layer, list(map(lambda s: s % (ix, 2, 'a'), simple_block_name_pattern)),
        int(lasagne.layers.get_output_shape(incoming_layer)[1]*ratio_n_filter), 1, int(1.0/ratio_size), 0)
    net.update(net_tmp)

    net_tmp, last_layer_name = build_simple_block(
        net[last_layer_name], list(map(lambda s: s % (ix, 2, 'b'), simple_block_name_pattern)),
        lasagne.layers.get_output_shape(net[last_layer_name])[1], 3, 1, 1)
    net.update(net_tmp)

    net_tmp, last_layer_name = build_simple_block(
        net[last_layer_name], list(map(lambda s: s % (ix, 2, 'c'), simple_block_name_pattern)),
        lasagne.layers.get_output_shape(net[last_layer_name])[1]*upscale_factor, 1, 1, 0,
        nonlin=None)
    net.update(net_tmp)

    right_tail = net[last_layer_name]
    left_tail = incoming_layer

    # left branch
    if has_left_branch:
        net_tmp, last_layer_name = build_simple_block(
            incoming_layer, list(map(lambda s: s % (ix, 1, ''), simple_block_name_pattern)),
            int(lasagne.layers.get_output_shape(incoming_layer)[1]*4*ratio_n_filter), 1, int(1.0/ratio_size), 0,
            nonlin=None)
        net.update(net_tmp)
        left_tail = net[last_layer_name]

    net['res%s' % ix] = ElemwiseSumLayer([left_tail, right_tail], coeffs=1)
    net['res%s_relu' % ix] = NonlinearityLayer(net['res%s' % ix], nonlinearity=rectify)

    return net, 'res%s_relu' % ix


def build_model_resnet():
    net = {}
    net['input'] = InputLayer((None, 3, 224, 224))
    sub_net, parent_layer_name = build_simple_block(
        net['input'], ['conv1', 'bn_conv1', 'conv1_relu'],
        64, 7, 2, 3, use_bias=True)
    net.update(sub_net)
    net['pool1'] = PoolLayer(net[parent_layer_name], pool_size=3, stride=2, pad=0, mode='max', ignore_border=False)
    block_size = list('abc')
    parent_layer_name = 'pool1'
    for c in block_size:
        if c == 'a':
            sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1, 1, True, 4, ix='2%s' % c)
        else:
            sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='2%s' % c)
        net.update(sub_net)

    block_size = list('abcd')
    for c in block_size:
        if c == 'a':
            sub_net, parent_layer_name = build_residual_block(
                net[parent_layer_name], 1.0/2, 1.0/2, True, 4, ix='3%s' % c)
        else:
            sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='3%s' % c)
        net.update(sub_net)

    block_size = list('abcdef')
    for c in block_size:
        if c == 'a':
            sub_net, parent_layer_name = build_residual_block(
                net[parent_layer_name], 1.0/2, 1.0/2, True, 4, ix='4%s' % c)
        else:
            sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='4%s' % c)
        net.update(sub_net)

    block_size = list('abc')
    for c in block_size:
        if c == 'a':
            sub_net, parent_layer_name = build_residual_block(
                net[parent_layer_name], 1.0/2, 1.0/2, True, 4, ix='5%s' % c)
        else:
            sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='5%s' % c)
        net.update(sub_net)
    net['pool5'] = PoolLayer(net[parent_layer_name], pool_size=7, stride=1, pad=0,
                             mode='average_exc_pad', ignore_border=False)
    net['prob'] = DenseLayer(net['pool5'], num_units=40, nonlinearity=softmax)

    return net

def load_pretrained_model():
    d = pickle.load(open('resnet50.pkl','rb'),encoding='latin-1')
    net = build_model_resnet()
    lasagne.layers.set_all_param_values(net['prob'], d['values'])
    return net

def deprocess(im,RGB=True):
    #by default, input im is BGR

    if RGB:
        im = im[::-1, :, :]
    im = np.swapaxes(np.swapaxes(im, 0, 1), 1, 2)

    return im

def extract_bounding_box(imlist,imlabels):
    imlist_=list()
    imlabels_=list()
    for (im,lab) in zip(imlist,imlabels):
        mask=np.sum(im,2)
        c=np.nonzero(mask>0)
        if len(c[0])>0: #if not empty seg
            max0=np.max(c[0])
            min0=np.min(c[0])
            max1=np.max(c[1])
            min1=np.min(c[1])
            seg=im[min0:max0,min1:max1]
            if seg.size>20: #if not too small
                imlist_.append(seg)
                imlabels_.append(lab)

    return imlist_,imlabels_

def load_amazon_images(dir):
    imlist=list()
    y_=list() #classes
    for subdir in os.listdir(directory):
        path=directory+subdir

        files=next(os.walk(path))[2] #files in the directory for the class corresponding to subdir

        for f in files:
            if f[-3:]=='png': #load the png images
                im=cv2.imread(path+'/'+f)[:,:,::-1]#.astype(np.uint8)
                resized=skimage.transform.resize(im, (512,512), preserve_range=True)
                imlist.append(resized)
                y_.append(subdir)
        print(subdir)
    return imlist,y_ #pass these into generate_augmented_dataset

def generate_augmented_dataset(imlist,labels,upscale_factor=1,smallest_dim=80,max_translate=40,
                               brightness_scale_range=[.999,1.001],max_skew=.3):

    #this function detects the size and accordingly (de)magnifies the image, translates it, and rotates it

    #the output image size is 224x224 to match VGG net
    #each raw segmented image will be resized from a range such that the largest dimension will be at most 224
    #on the other extreme,the largest dimension will be no smaller than smallest_dim in pixels
    #upscale_factor: how many replicates of each raw image to include (with random transformations)
    #images will also be randomly rotated, color dithered, and randomly translated in a small region near the center
    #max_translate is the maximum distance in pixels to decenter the image in one direction (plus or minus)
    #brightness scale range specifies the max and min possible multiplicative factor of the L channel in
    #HSL
    #random crops are generated by generating a random box size with same aspect ratio as the original image
    #but side lengths are scaled between .5 and 1.0 (to guarantee that no more than half othe image is occluded)

    X=list()
    y=list()
    for im,y_ in zip(imlist,labels):
        for _ in range(upscale_factor): #for each image, do multiple random augmentation operations
            #random crop:
            rawim=np.copy(im)
            h,w,_=rawim.shape
            h_center=h//2
            w_center=w//2

            h_scale=1#np.random.rand(1)*.5+.5
            w_scale=1#np.random.rand(1)*.5+.5
            h_new=int(h*h_scale) #side lengths of the box to sample from the original image
            w_new=int(w*w_scale)

            mask=np.zeros((h,w,1))

            #pick random numbers for translating the box crop
            h_trans=int((np.random.rand(1)-.5)*(h-h_new))
            w_trans=int((np.random.rand(1)-.5)*(w-w_new))
            h_center+=h_trans
            w_center+=w_trans
            mask[h_center-h_new//2:h_center+h_new//2,w_center-w_new//2:w_center+w_new//2,0]=1.
            mask=mask.astype(np.uint8)
            rawim*=mask

            #rotate image randomly:
            #for opencv, the image is the same dimensions, so cropping occurs after rotation
            randangle=np.random.rand(1)[0]*360
            #M = cv2.getRotationMatrix2D((w/2,h/2),int(randangle),1)
            #rawim = cv2.warpAffine(rawim,M,(w,h))
            rawim=rotate(rawim,randangle,order=1) #scipy.ndimage.interpolation; this version doesn't crop
            h,w,_=rawim.shape #new shape

            #random shear
            case=np.random.choice(4)
            #only shear one dim (if you shear both dims, you might get rotation ...; also more annoying to correct field of view)
            #case 0: positive horizontal
            #case 1: negative horizontal
            #case 2: positive vertical
            #case 3: negative vertical
            factor=np.random.rand(1)[0]*max_skew
            if case==0: #need to resize the image after skewing
                rawim=cv2.warpAffine(rawim,np.array([[1.,factor,0],[0,1,0]]),(np.int32(w+factor*h),h))
            if case==1:
                rawim=cv2.warpAffine(rawim,np.array([[1.,-factor,np.int32(factor*h)],[0,1,0]]),(np.int32(w+factor*h),h))
            if case==2:
                rawim=cv2.warpAffine(rawim,np.array([[1.,0,0],[factor,1,0]]),(w,np.int32(w*factor+h)))
            if case==3:
                rawim=cv2.warpAffine(rawim,np.array([[1.,0,0],[-factor,1,np.int32(factor*w)]]),(w,np.int32(w*factor+h)))


            h,w,_=rawim.shape #new shape

            #rescale image
            pix_rescale=np.random.rand(1)[0]*(224-smallest_dim)+smallest_dim #number of pixels to rescale to
            newim=np.zeros((224,224,3))
            if h>w:
                dim1=pix_rescale.astype(np.int32)
                dim2=(pix_rescale*w//h).astype(np.int32)
            else:
                dim1=(pix_rescale*h//w).astype(np.int32)
                dim2=pix_rescale.astype(np.int32)

            #generate random translation:
            dx,dy=np.random.rand((2))
            dx=int((dx-.5)*2*max_translate)
            dy=int((dy-.5)*2*max_translate)

            #create new image

            rawim = skimage.transform.resize(rawim, (dim1,dim2), preserve_range=True)
            #safeguard against translating out of the field of view:
            lowx=np.maximum(112-dim1//2+dx,0)
            highx=np.minimum(112+(dim1-dim1//2+dx),224)
            lowy=np.maximum(112-dim2//2+dy,0)
            highy=np.minimum(112+(dim2-dim2//2)+dy,224)
            rawim=rawim[:highx-lowx,:highy-lowy]
            newim[lowx:highx,
                  lowy:highy]=rawim

            #random brightness scaling (note; if using opencv, have to convert to 8-bit):
            rand_scale=np.random.rand(1)*(brightness_scale_range[1]-brightness_scale_range[0])+brightness_scale_range[0]
            im2=cv2.cvtColor(newim.astype(np.uint8),cv2.COLOR_RGB2HLS)
            im2[:,:,1]=np.maximum(np.minimum(im2[:,:,1]*rand_scale,255),0) #squeeze between 0 and 255
            newim=cv2.cvtColor(im2,cv2.COLOR_HLS2RGB)

            #process the formatting of the image:
            # Shuffle axes to c01
            newim = np.swapaxes(np.swapaxes(newim, 1, 2), 0, 1)

            # Convert to BGR
            newim = newim[::-1, :, :]

            X.append(newim[np.newaxis])
            y.append(y_)
    X=np.vstack(X).astype(np.float32)
    y=np.hstack(y).astype(np.int32)
    return X,y

def compile_theano_functions(net):
    output_layer = DenseLayer(net['pool5'], num_units=40, nonlinearity=softmax)
    # Define loss function and metrics, and get an updates dictionary
    X_sym = T.tensor4()
    y_sym = T.ivector()
    learning_rate=T.scalar()

    prediction = lasagne.layers.get_output(output_layer, X_sym)
    loss = lasagne.objectives.categorical_crossentropy(prediction, y_sym)
    loss = loss.mean()

    #weight decay:
    from lasagne.regularization import regularize_layer_params, l2
    l2_penalty = regularize_layer_params(output_layer, l2)
    lam=2e-4
    loss = loss + lam*l2_penalty

    params = lasagne.layers.get_all_params(output_layer, trainable=True)
    updates = lasagne.updates.nesterov_momentum(
            loss, params, learning_rate=learning_rate, momentum=0.9)

    #deterministic versions for prediction
    deterministic_prediction=lasagne.layers.get_output(output_layer, X_sym,deterministic=True)
    acc = T.mean(T.eq(T.argmax(deterministic_prediction, axis=1), y_sym),
                        dtype=theano.config.floatX)
    deterministic_loss=lasagne.objectives.categorical_crossentropy(deterministic_prediction, y_sym).mean()

    train_fn = theano.function([X_sym, y_sym, learning_rate], loss, updates=updates)
    val_fn = theano.function([X_sym, y_sym], [deterministic_loss, acc])

    return train_fn,val_fn,output_layer

def train_network(X_tr,y_tr,train_fn,val_fn,output_layer):
    BATCH_SIZE = 32
    # generator splitting an iterable into chunks of maximum length N
    def batches(iterable, N):
        chunk = []
        for item in iterable:
            chunk.append(item)
            if len(chunk) == N:
                yield chunk
                chunk = []
        if chunk:
            yield chunk
    def train_batch(lr): #lr is learning rate
        ix = list(range(len(y_tr)))
        np.random.shuffle(ix)
        ix = ix[:BATCH_SIZE]
        return train_fn(X_tr[ix], y_tr[ix], lr)

    for epoch in range(15):
        start=time.time()

        for batch in range(25):
            if epoch<5:
                loss = train_batch(lr=.002)
            else:
                loss = train_batch(lr=.0001)

        ix = list(range(500)) #number of images to validate on
        np.random.shuffle(ix)

        acc_tot = 0.
        for chunk in batches(ix, BATCH_SIZE): #pass through some of training set
            _, acc = val_fn(X_tr[chunk], y_tr[chunk])
            acc_tot += acc * len(chunk)

        acc_tot /= len(ix)
        print(epoch, acc_tot * 100,time.time()-start)

    dummy_dictionary=dict()
    dummy_dictionary['values']=lasagne.layers.get_all_param_values(output_layer)
    return dummy_dictionary

def main(location_of_images,trained_fname):
    BATCH_SIZE = 32
    start=time.time()
    #load images
    try:
        imlist=np.load('imlist_temp.npy')
        imlabels=np.load('imlabels_temp.npy')
    except:
        imlist,imlabels=read_images_segment(location_of_images)
        np.save('imlist_temp.npy',imlist)
        np.save('imlabels_temp.npy',imlabels)

    imlist,imlabels=extract_bounding_box(imlist,imlabels)
    np.save('imlist_temp.npy',imlist)
    np.save('imlabels_temp.npy',imlabels)
    print('loaded images',str(time.time()-start)+' seconds')

    #augment images
    try:
        X_tr=np.load('X_temp')
        y_tr=np.load('y_temp')
    except:
        X_tr,y_tr=generate_augmented_dataset(imlist,imlabels,smallest_dim=170,brightness_scale_range=[.99,1.],max_translate=1)
        np.save('X_temp',X_tr)
        np.save('y_temp',y_tr)
    print('augmented dataset',str(time.time()-start)+' seconds')
    #load and train network
    net=load_pretrained_model()
    print('network loaded',str(time.time()-start)+' seconds')
    train_fn,val_fn,output_layer=compile_theano_functions(net)
    print('functions compiled',str(time.time()-start)+' seconds')
    trained_net_params=train_network(X_tr,y_tr,train_fn,val_fn,output_layer)
    print('network trained',str(time.time()-start)+' seconds')
    #save network
    pickle.dump(trained_net_params,open(trained_fname,"wb"),pickle.HIGHEST_PROTOCOL)

if __name__=='__main__':
    if len(sys.argv)<3:
        print('requires location of images and return network name')
    else:
        main(sys.argv[1],sys.argv[2])
