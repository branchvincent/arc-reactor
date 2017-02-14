from status import CameraStatus
import matplotlib.pyplot as plt
import scipy.signal
import numpy as np
import cv2
from skimage import measure
from skimage import morphology

def testSegmentation():
    # cam_status = CameraStatus()
    # list_of_serial_nums = cam_status.depthCamera.get_online_cams()
    # cam_status.poll()
    # fullcolor = cam_status.cameraFullColorImages[list_of_serial_nums[0]]
    # depthcolor = cam_status.cameraColorImages[list_of_serial_nums[0]]
    # depth = cam_status.cameraDepthImages[list_of_serial_nums[0]]

    fullcolor = np.load('/home/bk/Documents/arc_images/test/0.npy')
    depth = np.load('/home/bk/Documents/arc_images/test/0_depth.npy')

    # fullcolor = cv2.medianBlur(fullcolor, 15)
    # fullcolor = cv2.bilateralFilter(fullcolor,5,175,175)
    # redgrad = cv2.Laplacian(fullcolor[:,:,0] ,cv2.CV_32F) 
    # greengrad = cv2.Laplacian(fullcolor[:,:,1] ,cv2.CV_32F) 
    # bluegrad = cv2.Laplacian(fullcolor[:,:,2] ,cv2.CV_32F) 

#color gradient 
    # total = np.absolute(redgrad) + np.absolute(bluegrad) + np.absolute(greengrad)
    # ret, total = cv2.threshold(total, 10, 1, cv2.THRESH_BINARY_INV)
    # total = np.where(depth == 0, 0, total)
    # total = morphology.erosion(total)
    # color_labels, num_color_labels = measure.label(total, neighbors=8, background=0, return_num=True)
    # plt.imshow(total,cmap='gray')
    # plt.show()
    # color_out_labels = np.zeros(color_labels.shape)
    # for i in range(1, num_color_labels):
    #     ind = np.where(color_labels == i)
    #     if len(ind[0]) < 50:
    #         continue
    #     else:
    #         color_out_labels[ind] = numObj+1
    #         numObj = numObj + 1


#grabcut
    # mask = np.zeros(fullcolor.shape[:2],np.uint8)
    # bgdModel = np.zeros((1,65),np.float64)
    # fgdModel = np.zeros((1,65),np.float64)
    # mask[50:200, 150:250] = 3
    # cv2.grabCut(fullcolor,mask,None,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_MASK)
    # mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    # fullcolor = fullcolor*mask2[:,:,np.newaxis]
    # cv2.rectangle(fullcolor, (150,50), (250, 200), (255,0,0))

    
    #filter
    depth = scipy.signal.medfilt(depth, 5)
    #remove anything above certain depth
    depth = np.where(depth < 5000, depth, 0)
    #gradient
    threshdepth = cv2.Laplacian(depth.astype('float32'), cv2.CV_32F)
    #threshold
    ret, threshdepth = cv2.threshold(threshdepth, 10, 1, cv2.THRESH_BINARY_INV)
    #remove anything above certain depth
    threshdepth = np.where(depth == 0, 0, threshdepth)
    #erode
    threshdepth = morphology.erosion(threshdepth, selem=np.ones((5,5)))
    #connected components
    all_labels, num_labels = measure.label(threshdepth, neighbors=8, background=0, return_num=True)
    #find cc that are greater than X
    numObj = 0
    out_labels = np.zeros(all_labels.shape)
    for i in range(1, num_labels):
        ind = np.where(all_labels == i)
        if len(ind[0]) < 500:
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
        cv2.rectangle(fullcolor,(x,y),(x+w,y+h),(0,255,0),2)

        rects.append([y,y+h, x, x+w])
        #min area rectangle
        # rect = cv2.minAreaRect(indices)
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # rects.append(box)
        # newbox = np.array([ [box[0][1], box[0][0]], [box[1][1], box[1][0]], [box[2][1], box[2][0]], [box[3][1], box[3][0]]  ])
        # cv2.drawContours(fullcolor, [newbox], 0 , (255,0,0), 2)  

    plt.imshow(fullcolor)
    plt.show()

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
            

if __name__ == '__main__':
    testSegmentation()