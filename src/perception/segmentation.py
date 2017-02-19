from status import CameraStatus
import matplotlib.pyplot as plt
import scipy.signal
import numpy as np
import cv2
from skimage import measure
from skimage import morphology

def depthSegmentation(depthImage):
    '''
    Takes in a depth image and returns a list of images
    that can be fed into deep learning to figure out what they are
    '''
    #parameters
    maxDepth = 5000
    medianFileterW = 5
    gradientThreshold = 10
    morphW = 5
    minNumCC = 500

    depth = depthImage    
    #filter
    depth = scipy.signal.medfilt(depth, medianFileterW)
    #remove anything above certain depth
    depth = np.where(depth < maxDepth, depth, 0)
    #gradient
    threshdepth = cv2.Laplacian(depth.astype('float32'), cv2.CV_32F)
    #threshold
    ret, threshdepth = cv2.threshold(threshdepth, gradientThreshold, 1, cv2.THRESH_BINARY_INV)
    #remove anything above certain depth
    threshdepth = np.where(depth == 0, 0, threshdepth)
    #erode
    threshdepth = morphology.erosion(threshdepth, selem=np.ones((morphW, morphW)))
    #connected components
    all_labels, num_labels = measure.label(threshdepth, neighbors=8, background=0, return_num=True)
    #find cc that are greater than X
    numObj = 0
    out_labels = np.zeros(all_labels.shape)
    for i in range(1, num_labels):
        ind = np.where(all_labels == i)
        if len(ind[0]) < minNumCC:
            all_labels[ind] = 0
        else:
            out_labels[ind] = numObj+1
            numObj = numObj + 1

    #where obj labels is non zero that is an object


    #for each of the objects get the bounding square
    rects = []
    for i in range(1, numObj):
        #find element in out_labels == i
        indices = np.array((out_labels == i).nonzero()).astype('float32')
        indices = np.transpose(indices)
        y,x,h,w = cv2.boundingRect(indices)
        # cv2.rectangle(fullcolor,(x,y),(x+w,y+h),(0,255,0),2)

        rects.append([y,y+h, x, x+w])
        #min area rectangle
        # rect = cv2.minAreaRect(indices)
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # rects.append(box)
        # newbox = np.array([ [box[0][1], box[0][0]], [box[1][1], box[1][0]], [box[2][1], box[2][0]], [box[3][1], box[3][0]]  ])
        # cv2.drawContours(fullcolor, [newbox], 0 , (255,0,0), 2)  

    # plt.imshow(fullcolor)
    # plt.show()

    #take the coordinates from rect and get those images in the full color image
    tinyImgs = []
    for i in range(len(rects)):
        startY = rects[i][0]
        endY = rects[i][1]
        startX = rects[i][2]
        endX = rects[i][3]
        img = fullcolor[startY:endY, startX:endX]
        tinyImgs.append(img)

    imagesForDL = []
    for i in range(len(tinyImgs)):
        #create zero image
        if tinyImgs[i].shape[0] > 256 or tinyImgs[i].shape[1] > 256:
            #large image, put it in 512, 512
            bg = np.zeros((512,512, 3))
        else:
            #small image put it in 256, 256
            bg = np.zeros((256,256, 3))
            startY = bg.shape[0]/2 - tinyImgs[i].shape[0]/2
            startX = bg.shape[1]/2 - tinyImgs[i].shape[1]/2
            if startX < 0:
                startX = 0
            if startY < 0:
                startY = 0
            bg[startY:startY +tinyImgs[i].shape[0], startX:startX+tinyImgs[i].shape[1]] = tinyImgs[i]
            imagesForDL.append(bg)
            
    return imagesForDL