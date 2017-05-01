import matplotlib
from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import scipy.signal
import numpy as np
import cv2
import sys
sys.path.append('../hardware/SR300/')
sys.path.append('..')
import realsense as rs
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
        
        #filtering parameter for the depth image
        self.medianFilterW = 7

        #filtering parameters for the mean shift filtering
        self.sp_rad = 7     #spatial window radius
        self.c_rad = 3     #color window radius
        
        self.max_elements = 100000  #maximum number of elements a single object can have

        self.topLeft = (78, 423)
        self.topRight = (666, 423)
        self.botLeft = (41, 337)
        self.botRight = (666, 92)

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

    #filter depth image
    depth = scipy.signal.medfilt(depthImage, params.medianFilterW)
    dmin = np.min(depth[np.nonzero(depth)])
    depth = (depth-dmin) / (depth.max()-dmin)
    depth = depth*255
    depth = np.where(depth > 0, depth, 0)
    depth = depth.astype('uint8')
    return_values['depth_filter'] = depth.copy()

    
    #create a 4D array of full color in lab space and the last channel is the depth image alligned to the full color
    
    #convert to LAB
    labcolor = cv2.cvtColor(fcolor, cv2.COLOR_RGB2LAB)

    #mean shift filter to remove texture
    labcolor = cv2.pyrMeanShiftFiltering(labcolor, params.sp_rad, params.c_rad)
    return_values['lab_filter'] = labcolor.copy()

    color_depth_image = np.zeros((imageH, imageW, 6))
    #fill the first three channels with the LAB image
    color_depth_image[:,:,0:3] = labcolor
    color_depth_image[:,:,3] = depth.copy()
    color_depth_image[:,:,4] = depth.copy()
    color_depth_image[:,:,5] = depth.copy()


    #create rectangle to remove any unwatned points
    mask = maskRect(params.topLeft, params.topRight, params.botRight, np.zeros((imageH,imageW)))

    #remove any points outside the desired rectangle
    for i in range(6):
        color_depth_image[:,:,i] = np.where(mask == 1, color_depth_image[:,:,i], 0)

    color_depth_image[:,:,0:3] = cv2.GaussianBlur(color_depth_image[:,:,0:3],(0,0), params.sigma, params.sigma)

    #perform the graph segmentation
    gs = cv2.ximgproc.segmentation.createGraphSegmentation()
    gs.setSigma(0.001)
    gs.setK(params.k)
    gs.setMinSize(int(params.minSize))
    outp = gs.processImage(color_depth_image)
    numObj = outp.max()
    logger.info("Found {} segments in the image".format(outp.max()))
    outp = np.where(outp == 0, 255, outp)

    display_segment_img = outp.copy().astype('float32')
    display_segment_img = (display_segment_img - display_segment_img.min()) / (display_segment_img.max() - display_segment_img.min())
    display_segment_img = plt.cm.jet(display_segment_img)
    display_segment_img = (255*display_segment_img).astype('uint8')
    display_segment_img
    return_values['segmented_image'] = np.array(display_segment_img[:,:,0:3])
    
    segments = []

    #create a copy of the full color image we can draw rectangles on
    fcolor_rects = fcolor.copy()

    #extract the sub image for each label based on the minimum bounding rectangle
    tinyColorImgs = []
    imagesForDL = []
    for i in range(numObj):
        #find element in outp == i
        indices = np.array((outp == i).nonzero())
        if not indices.shape[1] > 0:
            continue
        indices = np.transpose(indices.astype('float32'))
    
        #axis aligned rect
        axisrect = cv2.boundingRect(indices)
        maskeditem = np.zeros(fcolor.shape)
        maskeditem[:,:,0] = np.where(outp == i, fcolor[:,:,0], 0)
        maskeditem[:,:,1] = np.where(outp == i, fcolor[:,:,1], 0)
        maskeditem[:,:,2] = np.where(outp == i, fcolor[:,:,2], 0)

        maskeditemlab = np.zeros(fcolor.shape)
        maskeditemlab[:,:,0] = np.where(outp == i, labcolor[:,:,0], 0)
        maskeditemlab[:,:,1] = np.where(outp == i, labcolor[:,:,1], 0)
        maskeditemlab[:,:,2] = np.where(outp == i, labcolor[:,:,2], 0)
        
        startY = axisrect[0]
        endY = axisrect[0] + axisrect[2]
        startX = axisrect[1]
        endX = axisrect[1] + axisrect[3]
        img_crop = maskeditem[startY:endY, startX:endX]
        img_crop_lab = maskeditemlab[startY:endY, startX:endX]



        #code to remove the red tote
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
            continue

        segments.append(indices.astype('int32'))
        #make image for deep learning
        if (img_crop.shape[0] > 256 or img_crop.shape[1] > 256) and (img_crop.shape[0] < 512 and img_crop.shape[1] < 512):
            #large image, put it in 512, 512
            tinyColorImgs.append(img_crop.astype('uint8'))
            bg = np.zeros((512,512, 3)).astype('uint8')
            startY = int(bg.shape[0]/2 - img_crop.shape[0]/2)
            startX = int(bg.shape[1]/2 - img_crop.shape[1]/2)
            if startX < 0:
                startX = 0
            if startY < 0:
                startY = 0
            bg[startY:startY +img_crop.shape[0], startX:startX+img_crop.shape[1]] = img_crop
            imagesForDL.append(bg)
            
        elif img_crop.shape[0] <= 256 and img_crop.shape[1] <= 256:
            #small image put it in 256, 256
            tinyColorImgs.append(img_crop.astype('uint8'))
            bg = np.zeros((256,256, 3)).astype('uint8')
            startY = int(bg.shape[0]/2 - img_crop.shape[0]/2)
            startX = int(bg.shape[1]/2 - img_crop.shape[1]/2)
            if startX < 0:
                startX = 0
            if startY < 0:
                startY = 0
            bg[startY:startY +img_crop.shape[0], startX:startX+img_crop.shape[1]] = img_crop
            imagesForDL.append(bg)
        else:
            #image was really big and is probably the entire image or some mistake, so we are not adding it
            logger.warn("Segment was larger than 512x512. This is probably a mistake, not adding to identification set")
     
    
    #tiny depth images are the size of the full depth image and non zero where the object is
    return_values['DL_images'] = imagesForDL
    return_values['small_images'] = tinyColorImgs
    return_values['pixel_locations'] = segments

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



class SegmentationGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(SegmentationGUI, self).__init__()
        self.initUI()
        self.segret = None
        self.depth_image = None
        self.color_image = None
        self.params = GraphSegmentationParams()
    def initUI(self):
        
        #add a vertical layout
        self.vertLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.vertLayout)

        #add horizontal layouts
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzMid = QtWidgets.QHBoxLayout()
        self.horzBot = QtWidgets.QHBoxLayout()
      
        #top
        self.file_name_textbox = QtWidgets.QLineEdit(self)
        self.file_name_textbox.setMaximumHeight(35)
        self.file_name_textbox.setMinimumWidth(150)
        self.file_name_textbox.setText('/home/bk/Documents/arc_images/competition_images/cluttered_test/12')
        self.load_file_button = QtWidgets.QPushButton("Load", self)
        self.load_file_button.pressed.connect(self.load_file)
        self.horzTop.addStretch(1)
        self.horzTop.addWidget(self.file_name_textbox)
        self.file_name_textbox.show()
        self.horzTop.addSpacing(10)
        self.horzTop.addWidget(self.load_file_button)
        self.horzTop.addStretch(1)
        self.load_file_button.show()

        #mid
        self.grid_layout_mid = QtWidgets.QGridLayout()
        self.param_locations = ['Med filter width', "k", "sigma", "Min number", "DL image", "Color radius"]
        param_initialvals = [5, 250, 0.5, 600, 0, 33]
        self.param_spinboxes = []
        for i in range(len(self.param_locations)):
            #create layout
            vert_layout = QtWidgets.QVBoxLayout()
            parameter_name = QtWidgets.QLabel(self)
            parameter_name.setText(self.param_locations[i])
            spinbox = QtWidgets.QDoubleSpinBox(self)
            spinbox.setMinimum(-100000)
            spinbox.setMaximum(100000)
            spinbox.setDecimals(2)
            spinbox.setValue(param_initialvals[i])
            if i != 4:
                spinbox.editingFinished.connect(self.run_segmentation)
            else:
                spinbox.editingFinished.connect(self.changeDL)
            self.param_spinboxes.append(spinbox)
            vert_layout.addWidget(parameter_name)
            vert_layout.addWidget(spinbox)

            row = i//(len(self.param_locations)/2)
            col = i % (len(self.param_locations)/2) + 1
            self.grid_layout_mid.addItem(vert_layout,row, col)
        
        self.horzMid.addItem(self.grid_layout_mid)
        
        #bot
        self.grid_layout_bot = QtWidgets.QGridLayout()
        self.seg_steps = ['Input', 'Median filter', 'Mean shift', 'Graph Cut', 'Blank', 'Output']
        self.seg_step_displays = []
        for i in range(len(self.seg_steps)):
            #create layout
            vert_layout = QtWidgets.QVBoxLayout()

            #add display
            seg_step_display = QtWidgets.QLabel(self)
            zeroImg = np.zeros((480,640,3),dtype='uint8')
            image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap(image)
            seg_step_display.setPixmap(pix.scaled(320, 240))
            self.seg_step_displays.append(seg_step_display)

            #add label
            seg_step_label = QtWidgets.QLabel(self)
            seg_step_label.setText(self.seg_steps[i])
            seg_step_label.setAlignment(QtCore.Qt.AlignHCenter)

            vert_layout.addWidget(seg_step_label)
            vert_layout.addSpacing(5)
            vert_layout.addWidget(seg_step_display)
            seg_step_label.setMaximumHeight(20)
            
            row = i//(len(self.seg_steps)/2)
            col = i % (len(self.seg_steps)/2) + 1
            self.grid_layout_bot.addItem(vert_layout, row, col)

        self.horzBot.addItem(self.grid_layout_bot)

        #add all layouts in
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzMid)
        self.vertLayout.addItem(self.horzBot)


        self.setWindowTitle('SegmentationGUI')

    def load_file(self):
        if not self.file_name_textbox.text() == "":
            try:
                self.depth_image = np.load(self.file_name_textbox.text() + "_depth.npy")
                self.color_image = np.load(self.file_name_textbox.text() + ".npy")
            except:
                logger.warn("Unable to load file {}".format(self.file_name_textbox.text() + "_depth.npy"))
                return

            self.run_segmentation()
        else:
            logger.warn("No input file specified")

    def run_segmentation(self):
        #collect parameters
        # ['Med filter width', "k", "sigma", "Min number", "Spatial Radius", "Color radius"]
        self.params.medianFilterW = int(self.param_spinboxes[0].value())
        self.params.k = self.param_spinboxes[1].value()
        self.params.sigma = self.param_spinboxes[2].value()
        self.params.minSize = self.param_spinboxes[3].value()
        self.params.sp_rad = 3
        self.params.c_rad = int(self.param_spinboxes[5].value())

        logger.debug("Params Med filter = {} K = {} Sigma = {} Min Size = {} SP Rad {} Color Rad = {}".format(self.params.medianFilterW, self.params.k, self.params.sigma, self.params.minSize, self.params.sp_rad, self.params.c_rad))

        if self.depth_image is None:
            logger.warn("No image has been loaded")
            return

        self.segret = graphSegmentation(self.depth_image, self.color_image, self.params)

        #update the images 
        # ['Input', 'Median filter', 'Gradient filter', 'Threshold', 'Connected Components', 'Output']
        zeroImg = self.color_image
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[0].setPixmap(pix.scaled(320, 240))

        zeroImg = self.segret['depth_filter']
        #scale from zero to 255
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1],QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[1].setPixmap(pix.scaled(320, 240))

        zeroImg = self.segret['lab_filter']
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[2].setPixmap(pix.scaled(320, 240))

        zeroImg = self.segret['segmented_image']
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[3].setPixmap(pix.scaled(320, 240))

        zeroImg = self.segret['DL_images'][0]
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[4].setPixmap(pix.scaled(320, 240))

        zeroImg = self.segret['boxes_image']
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[5].setPixmap(pix.scaled(320, 240))

    def changeDL(self):
        val = int(self.param_spinboxes[4].value())
        if val < len(self.segret['DL_images']):
            zeroImg = self.segret['DL_images'][val]
            image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3, QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap(image)
            self.seg_step_displays[4].setPixmap(pix.scaled(320, 240))

            indices = self.segret['pixel_locations'][val].astype('float32')
                    
            #min area rectangle
            rect = cv2.minAreaRect(indices)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            newbox = np.array([ [box[0][1], box[0][0]], [box[1][1], box[1][0]], [box[2][1], box[2][0]], [box[3][1], box[3][0]]  ])
            zeroImg = self.segret['boxes_image'].copy()
            cv2.drawContours(zeroImg, [newbox], 0 , (0,255,255), 2)
            image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap(image)
            self.seg_step_displays[5].setPixmap(pix.scaled(320, 240))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    # #set up the window
    sg = SegmentationGUI()
    sg.show()
    sys.exit(app.exec_())

