import numpy as np
import cv2
import sys
import time
import logging
logger = logging.getLogger(__name__)


class GraphSegmentationParams():
    def __init__(self):

        #parameters for the graph cut segmentation
        self.minSize = 500
        self.k = 300
        self.sigma = 0.5

        self.L_weight = 1
        self.A_weight = 1
        self.B_weight = 1
        self.depth_weight = 1
        self.isTote = True
        self.isShelf = False
        self.mask = None            #mask used to block out unwanted points


def graphSegmentation(depthImage, fcolor, point_cloud, params=GraphSegmentationParams()):
    '''
    Takes in a depth image and full color image and returns a list of images
    that can be fed into deep learning to figure out what they are.
    extrinsics is a matrix made from an rs.extrinsics object that transforms points from depth image to
    the fullcolor image
    '''
    imageW = fcolor.shape[1]
    imageH = fcolor.shape[0]
    #return dictionary
    return_values = {}


    depth = depthImage.astype('uint16')

    #create a 4D array of full color in lab space and the last channel is the depth image alligned to the full color
    #convert to LAB
    labcolor = cv2.cvtColor(fcolor, cv2.COLOR_RGB2LAB)

    color_depth_image = np.zeros((imageH, imageW, 4))
    #fill the first three channels with the LAB image, 4th with depth, 5-7 normals
    color_depth_image[:,:,0:3] = labcolor
    color_depth_image[:,:,3] = depth.copy()

    #remove any points outside the desired rectangle
    if not params.mask is None:
        for i in range(color_depth_image.shape[2]):
            color_depth_image[:,:,i] = np.where(params.mask == 1, color_depth_image[:,:,i], 0)

    #perform the graph segmentation
    gs = cv2.ximgproc.segmentation.createGraphSegmentation()
    gs.setSigma(params.sigma)
    gs.setK(params.k)
    gs.setMinSize(int(params.minSize))

    weights = [params.L_weight, params.A_weight, params.B_weight, params.depth_weight]
    for n,w in enumerate(weights):
        color_depth_image[:,:,n] = color_depth_image[:,:,n]*w
    labeled_image = gs.processImage(color_depth_image)
    numObj = labeled_image.max()
    logger.info("Found {} segments in the image".format(labeled_image.max()))

    segments = []
    #extract the sub image for each label based on the minimum bounding rectangle
    imagesForDL = []

    tote_mask = np.ones(fcolor.shape)
    if params.isTote:
        t = (labcolor - [80,175,175]) #TODO update these at competition
        t2 = np.sqrt(t[:,:,0]**2 + t[:,:,1]**2 + t[:,:,2]**2)
        tote_mask[:,:,0] = np.where(t2 < 50, 0, 1)
        tote_mask[:,:,1] = np.where(t2 < 50, 0, 1)
        tote_mask[:,:,2] = np.where(t2 < 50, 0, 1)
    elif params.isShelf:
        pass
        # t = (labcolor - [140,140,160])#TODO update these at competition
        # t2 = np.sqrt(t[:,:,0]**2 + t[:,:,1]**2 + t[:,:,2]**2)
        # tote_mask[:,:,0] = np.where(t2 < 40, 0, 1)
        # tote_mask[:,:,1] = np.where(t2 < 40, 0, 1)
        # tote_mask[:,:,2] = np.where(t2 < 40, 0, 1)

    for i in range(numObj):
        #find element in labeled_image == i
        dl_tuple = create_deep_learing_image(fcolor, labeled_image, i, params.isTote, params.isShelf, tote_mask)
        if dl_tuple is None:
            continue
        else:
            ind, dl_im = dl_tuple
        segments.append(ind)
        imagesForDL.append(dl_im)

    #create a new labeled image that only has non zero labels for segments that we are using
    #for deep learning inference
    labeled_image = np.zeros(labeled_image.shape)
    for i in range(len(imagesForDL)):
        ys = segments[i][:,0]
        xs = segments[i][:,1]
        labeled_image[ys,xs] = i+1

    labeled_image = np.where(params.mask, labeled_image, 0)
    #tiny depth images are the size of the full depth image and non zero where the object is
    return_values['DL_images'] = imagesForDL
    return_values['pixel_locations'] = segments
    return_values['labeled_image'] = labeled_image.astype(np.int)

    return return_values

def create_deep_learing_image(fullcolor, labeled_image, index, isTote, isShelf, tote_mask=None):
    '''
    Given a full color image, a labeled image, an index of the desired object,
    whether or not the tote appears in this image, and whether or not the shelf
    appears in this images, this function returns a tuple
    (indices, 224x224 image suitable for input into deep learning)
    Returns none if the image is not good for input into DL
    '''
    #given inputs, returns segmented image
    #object coordinates:
    indices = np.array((labeled_image == index).nonzero()).transpose()
    if not indices.shape[1] > 0:
        return None
    #isolate the object
    mask=np.zeros_like(fullcolor)
    mask[indices[:,0],indices[:,1]]=1

    if tote_mask is None:
        tote_mask = np.ones(fullcolor.shape)
    
    masked=fullcolor*mask*tote_mask

    #coordinates of corners of the bounding box (unrotated)
    y1,x1=np.min(indices,0)
    y2,x2=np.max(indices,0)

    cropped_masked_image=masked[y1:y2,x1:x2]

    #count the number of valid pixels after removing the tote
    cntnonzero = np.where(tote_mask*mask)
    cntnonzero = len(cntnonzero[0])
    if cntnonzero < 200:
        logger.info("Index {} in segmentation was mostly tote. Or image is too small. Ignoring".format(index))
        return None

    #create image for deep learning
    h,w,_=cropped_masked_image.shape
    if h > 512 or w > 512:
        #image was really big and is probably the entire image or some mistake, so we are not adding it
        logger.warn("Segment {} was larger than 512x512. Not adding to identification set".format(index))
        return None
    if h>224:
        w=int(w*224./h)
        h=224
    if w>224:
        h=int(h*224./w)
        w=224

    #create new image
    cropped_masked_image_resized = cv2.resize(cropped_masked_image, (w,h))

    image_for_DL=np.zeros((224,224,3)).astype('uint8')
    image_for_DL[112-h//2:112-h//2+h,112-w//2:112-w//2+w]=cropped_masked_image_resized

    #only keep the indices that were not removed by color matching tote/shelf
    y_ind, x_ind, _ = np.where(masked != [0,0,0])
    indices = np.zeros((len(y_ind),2))
    indices[:,0] = y_ind
    indices[:,1] = x_ind
    return (indices.astype('int32'), image_for_DL)


