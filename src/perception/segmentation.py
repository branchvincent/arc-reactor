import numpy as np
import cv2
import sys
sys.path.append('..')
# configure the root logger to accept all records
import logging
logger = logging.getLogger()
logger.setLevel(logging.NOTSET)

formatter = logging.Formatter('%(asctime)s\t[%(name)s] %(pathname)s:%(lineno)d\t%(levelname)s:\t%(message)s')

# set up colored logging to console
from rainbow_logging_handler import RainbowLoggingHandler
console_handler = RainbowLoggingHandler(sys.stderr)
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)


def maskRect(topleft, topright, botright, image):
    imageH = image.shape[0]
    imageW = image.shape[1]
    #create rectangle to remove any unwatned points
    mask = np.zeros((imageH,imageW))
    a = np.array(topleft)
    b = np.array(topright)
    c = np.array(botright)
    AB = b-a
    BC = c-b
    [xs, ys] = np.meshgrid(range(imageW), range(imageH))
    c = np.zeros((imageH,imageW,2))
    c[:,:,0] = xs
    c[:,:,1] = ys
    c = c-a
    mask = np.logical_and(np.logical_and(c.dot(AB) > 0, c.dot(AB) < AB.dot(AB)), np.logical_and(c.dot(BC) > 0, c.dot(BC) < BC.dot(BC)))
    return mask

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

        #filtering parameters for the mean shift filtering
        self.sp_rad = 7     #spatial window radius
        self.c_rad = 3     #color window radius
        
        self.max_elements = 100000  #maximum number of elements a single object can have

        self.mask = None            #mask used to block out unwanted points

def graphSegmentation(depthImage, fcolor, params=GraphSegmentationParams()):
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
    return_values['depth_filter'] = depth.copy()

    
    #create a 4D array of full color in lab space and the last channel is the depth image alligned to the full color
    #convert to LAB
    labcolor = cv2.cvtColor(fcolor, cv2.COLOR_RGB2LAB)

    #mean shift filter to remove texture
    labcolor = cv2.pyrMeanShiftFiltering(labcolor, params.sp_rad, params.c_rad)
    return_values['lab_filter'] = labcolor.copy()

    color_depth_image = np.zeros((imageH, imageW, 4))
    #fill the first three channels with the LAB image
    color_depth_image[:,:,0:3] = labcolor
    color_depth_image[:,:,3] = depth.copy()

    #remove any points outside the desired rectangle
    if not params.mask is None:
        for i in range(4):
            color_depth_image[:,:,i] = np.where(params.mask == 1, color_depth_image[:,:,i], 0)

    color_depth_image[:,:,0:3] = cv2.GaussianBlur(color_depth_image[:,:,0:3],(0,0), params.sigma, params.sigma)

    #perform the graph segmentation
    gs = cv2.ximgproc.segmentation.createGraphSegmentation()
    gs.setSigma(0.001)
    gs.setK(params.k)
    gs.setMinSize(int(params.minSize))
    weights = [params.L_weight, params.A_weight, params.B_weight, params.depth_weight]
    for n,w in enumerate(weights):
        color_depth_image[:,:,n] = color_depth_image[:,:,n]*w
    outp = gs.processImage(color_depth_image)
    numObj = outp.max()
    logger.info("Found {} segments in the image".format(outp.max()))
    
    segments = []

    #create a copy of the full color image we can draw rectangles on
    fcolor_rects = fcolor.copy()

    #extract the sub image for each label based on the minimum bounding rectangle
    tinyColorImgs = []
    imagesForDL = []
    for i in range(numObj):
        #find element in outp == i
        dl_tuple = create_deep_learing_image(fcolor, outp, i)
        if dl_tuple is None:
            continue
        else:
            ind, tiny_img, dl_im = dl_tuple
        tinyColorImgs.append(tiny_img)
        segments.append(ind)
        imagesForDL.append(dl_im)
        
    
    #tiny depth images are the size of the full depth image and non zero where the object is
    return_values['DL_images'] = imagesForDL
    return_values['small_images'] = tinyColorImgs
    return_values['pixel_locations'] = segments
    return_values['labeled_image'] = outp
    #gets the largest items, but isn't working right now 4/29/2017
    # sizes = [x.size for x in segments]
    # ind = np.argsort(sizes)[::-1]
    # for i in range(len(ind)):
    #     rect = cv2.minAreaRect(segments[ind[i]].astype('float32'))
    #     box = cv2.boxPoints(rect)
    #     box = np.int0(box)
    #     newbox = np.array([ [box[0][1], box[0][0]], [box[1][1], box[1][0]], [box[2][1], box[2][0]], [box[3][1], box[3][0]]  ])
    #     cv2.drawContours(fcolor_rects, [newbox], 0 , (0,255,255), 2)
    return_values['boxes_image'] = fcolor_rects

    return return_values

def create_deep_learing_image(fullcolor, labeled_image, index):
    '''
    Given a full color image, a labeled image and index of the desired object,
    this function returns a tuple
    (indices, cropped image, 224x224 image suitable for input into deep learning)
    Returns none if the image is not good for input into DL
    '''
    #find element in labeld image == index
    indices = np.array((labeled_image == index).nonzero())
    if not indices.shape[1] > 0:
        return None
    indices = np.transpose(indices.astype('float32'))
    labcolor = cv2.cvtColor(fullcolor, cv2.COLOR_RGB2LAB)
    #axis aligned rect
    axisrect = cv2.boundingRect(indices)
    maskeditem = np.zeros(fullcolor.shape)
    maskeditem[:,:,0] = np.where(labeled_image == index, fullcolor[:,:,0], 0)
    maskeditem[:,:,1] = np.where(labeled_image == index, fullcolor[:,:,1], 0)
    maskeditem[:,:,2] = np.where(labeled_image == index, fullcolor[:,:,2], 0)

    maskeditemlab = np.zeros(fullcolor.shape)#to decide if this is the tote
    maskeditemlab[:,:,0] = np.where(labeled_image == index, labcolor[:,:,0], 0)
    maskeditemlab[:,:,1] = np.where(labeled_image == index, labcolor[:,:,1], 0)
    maskeditemlab[:,:,2] = np.where(labeled_image == index, labcolor[:,:,2], 0)
    
    startY = axisrect[0]
    endY = axisrect[0] + axisrect[2]
    startX = axisrect[1]
    endX = axisrect[1] + axisrect[3]
    img_crop = maskeditem[startY:endY, startX:endX]
    img_crop_lab = maskeditemlab[startY:endY, startX:endX]

    #is this segment the tote?
    cnt = 0
    cntnonzero = 0
    #remove the red tote
    for y in range(img_crop_lab.shape[0]):
        for x in range(img_crop_lab.shape[1]):
            #80 175 175
            pix = img_crop_lab[y,x]
            if pix[0]!= 0 and pix[1] != 0  and pix[2] != 0:
                cntnonzero += 1
                if np.linalg.norm(pix-np.array([80,175,175])) < 50:
                    cnt+=1
            
    if cnt/cntnonzero > 0.5:
        logger.debug("Found tote in image label {}".format(index))
        return None

    #make image for deep learning
    if (img_crop.shape[0] > 224 or img_crop.shape[1] > 224) and (img_crop.shape[0] < 512 and img_crop.shape[1] < 512):
        #large image, put it in 512, 512
        bg = np.zeros((512,512, 3)).astype('uint8')
        startY = int(bg.shape[0]/2 - img_crop.shape[0]/2)
        startX = int(bg.shape[1]/2 - img_crop.shape[1]/2)
        if startX < 0:
            startX = 0
        if startY < 0:
            startY = 0
        bg[startY:startY +img_crop.shape[0], startX:startX+img_crop.shape[1]] = img_crop
        #shrink to 224 x 224
        im = cv2.resize(bg,(224, 224))
        
    elif img_crop.shape[0] <= 224 and img_crop.shape[1] <= 224:
        #small image put it in 224, 224
        bg = np.zeros((224,224, 3)).astype('uint8')
        startY = int(bg.shape[0]/2 - img_crop.shape[0]/2)
        startX = int(bg.shape[1]/2 - img_crop.shape[1]/2)
        if startX < 0:
            startX = 0
        if startY < 0:
            startY = 0
        bg[startY:startY +img_crop.shape[0], startX:startX+img_crop.shape[1]] = img_crop
        im = bg
    else:
        #image was really big and is probably the entire image or some mistake, so we are not adding it
        logger.warn("Segment {} was larger than 512x512. This is probably a mistake, not adding to identification set".format(index))
        return None

    return (indices.astype('int32'), img_crop.astype('uint8'), im)


