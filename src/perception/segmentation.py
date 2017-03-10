import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import scipy.signal
import numpy as np
import cv2
from skimage import measure
from skimage import morphology
import sys
sys.path.append('../hardware/SR300/')
sys.path.append('..')
import realsense as rs

def depthSegmentation(depthImage, fullcolor, extrinsics=None):
    '''
    Takes in a depth image plus full color image and returns a list of images
    that can be fed into deep learning to figure out what they are. also returns
    list of depth images that can be used to determine where objects are in the world
    extrinsics is an rs.extrinsics object that transforms points from depth image to
    the fullcolor image
    '''
    #parameters
    maxDepth = 7000
    minDepth = 0
    medianFileterW = 7
    gradientThreshold = 10
    morphW = 5
    minNumCC = 500

    depth = depthImage    
    #filter
    depth = scipy.signal.medfilt(depth, medianFileterW)
    #plt.imshow(depth)
    #plt.show()
    #remove anything above certain depth
    depth = np.where(depth < maxDepth, depth, 0)
    depth = np.where(depth > minDepth, depth, 0)
    #gradient
    threshdepth = cv2.Laplacian(depth.astype('float32'), cv2.CV_32F)
    #threshold
    ret, threshdepth = cv2.threshold(threshdepth, gradientThreshold, 1, cv2.THRESH_BINARY_INV)
    # plt.imshow(threshdepth)
    # plt.show()
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

    # plt.imshow(np.rot90(fullcolor,3))
    # plt.show()

    #take the coordinates from rect and get those images in the full color image
    tinyColorImgs = []
    tinyDepthImgs = []
    for i in range(len(rects)):
        startY = rects[i][0]
        endY = rects[i][1]
        startX = rects[i][2]
        endX = rects[i][3]
        colorStartX = startX
        colorStartY = startY
        colorEndX = endX
        colorEndY = endY
        #get from depth to color coordinates if we have extrinsics
        if not extrinsics is None:
            point = rs.float3()
            #transform start
            point.x = startX
            point.y = startY
            point.z = 0
            newpt = extrinsics.transform(point)
            colorStartX = int(newpt.x)
            colorStartY = int(newpt.y)
            #transform end
            point.x = endX
            point.y = endY
            point.z = 0
            newpt = extrinsics.transform(point)
            colorEndX = int(newpt.x)
            colorEndY = int(newpt.y)

        img = fullcolor[colorStartY:colorEndY, colorStartX:colorEndX]
        tinyColorImgs.append(img)
        img = np.zeros(depthImage.shape)
        img[startY:endY, startX:endX] = depthImage[startY:endY, startX:endX]
        tinyDepthImgs.append(img)

    imagesForDL = []
    for i in range(len(tinyColorImgs)):
        #create zero image
        if tinyColorImgs[i].shape[0] > 256 or tinyColorImgs[i].shape[1] > 256:
            #large image, put it in 512, 512
            bg = np.zeros((512,512, 3))
            startY = int(bg.shape[0]/2 - tinyColorImgs[i].shape[0]/2)
            startX = int(bg.shape[1]/2 - tinyColorImgs[i].shape[1]/2)
            if startX < 0:
                startX = 0
            if startY < 0:
                startY = 0
            bg[startY:startY +tinyColorImgs[i].shape[0], startX:startX+tinyColorImgs[i].shape[1]] = tinyColorImgs[i]
            imagesForDL.append(bg)
        else:
            #small image put it in 256, 256
            bg = np.zeros((256,256, 3))
            startY = int(bg.shape[0]/2 - tinyColorImgs[i].shape[0]/2)
            startX = int(bg.shape[1]/2 - tinyColorImgs[i].shape[1]/2)
            if startX < 0:
                startX = 0
            if startY < 0:
                startY = 0
            bg[startY:startY +tinyColorImgs[i].shape[0], startX:startX+tinyColorImgs[i].shape[1]] = tinyColorImgs[i]
            imagesForDL.append(bg)
            
    #these images are stored in BGR deep learning expects RGB!
    return (imagesForDL, tinyDepthImgs)


if __name__ == "__main__":
	depth=np.load('images/0_depth.npy')
	color=np.load('images/0.npy')
	list_imgs = depthSegmentation(depth,color)

