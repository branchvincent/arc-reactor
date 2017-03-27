import matplotlib
from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
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


class DepthSegmentationParams():
    def __init__(self):
        self.maxDepth = 7500
        self.minDepth = 5000
        self.medianFilterW = 7
        self.gradientThreshold = 10
        self.morphW = 3
        self.minNumCC = 500
        self.maxNumCC = 2000
        self.topLeft = (70, 55)
        self.topRight = (549, 83)
        self.botLeft = (41, 337)
        self.botRight = (550, 366)

def depthSegmentation(depthImage, fcolor, extrinsics=None, params=DepthSegmentationParams(), debug=False):
    '''
    Takes in a depth image plus full color image and returns a list of images
    that can be fed into deep learning to figure out what they are. also returns
    list of depth images that can be used to determine where objects are in the world
    extrinsics is an rs.extrinsics object that transforms points from depth image to
    the fullcolor image
    '''

    #return dictionary
    return_values = {}

    #parameters
    maxDepth = params.maxDepth
    minDepth = params.minDepth
    medianFileterW = params.medianFilterW
    gradientThreshold = params.gradientThreshold
    morphW = params.morphW
    minNumCC = params.minNumCC
    maxNumCC = params.maxNumCC
    topLeft = params.topLeft
    topRight = params.topRight
    botLeft = params.botLeft
    botRight = params.botRight

    #get rid of any points outside the rectangle
    mask = np.zeros((480,640))
    a = np.array(topLeft)
    b = np.array(topRight)
    c = np.array(botRight)
    AB = b-a
    BC = c-b
    [xs, ys] = np.meshgrid(range(640), range(480))
    c = np.zeros((480,640,2))
    c[:,:,0] = xs
    c[:,:,1] = ys
    c = c-a
    mask = np.logical_and(np.logical_and(c.dot(AB) > 0, c.dot(AB) < AB.dot(AB)), np.logical_and(c.dot(BC) > 0, c.dot(BC) < BC.dot(BC)))

    depth = depthImage  
    depth = np.where(mask==1, depth, 0)
    #make a copy of the color image to return
    fullcolor = fcolor.copy()  

    #remove anything above certain depth
    depth = np.where(depth < maxDepth, depth, 0)
    depth = np.where(depth > minDepth, depth, 0)

    #filter
    depth = scipy.signal.medfilt(depth, medianFileterW)
    return_values['median_filter'] = depth.copy()

    
    dmin = np.min(depth[np.nonzero(depth)])
    fcolor = (depth-dmin) / (depth.max()-dmin)
    fcolor = fcolor*255
    fcolor = np.where(fcolor > 0, fcolor, 0)
    fcolor = fcolor.astype('uint8')
    return_values['median_filter'] = fcolor.copy()
    if debug:
        plt.imshow(fcolor)
        plt.show()

    
    #gradient
    threshdepth = cv2.Laplacian(depth.astype('float32'), cv2.CV_32F, ksize=1)
    return_values['gradient_filter'] = threshdepth.copy()
    if debug:
        plt.imshow(threshdepth)
        plt.show()

    #threshold
    ret, threshdepth = cv2.threshold(threshdepth, gradientThreshold, 1, cv2.THRESH_BINARY_INV)
    threshdepth = np.where(mask==1, threshdepth, 0)
    threshdepth = np.where(depth == 0, 0, threshdepth)
    return_values['threshold'] = threshdepth.copy()
    if debug:
        plt.imshow(threshdepth)
        plt.show()


    all_labels, num_labels = measure.label(threshdepth, neighbors=8, background=0, return_num=True)
    for i in range(1, num_labels):
        ind = np.where(all_labels == i)
        if len(ind[0]) < minNumCC:
            threshdepth[ind] = 0

    #erode
    threshdepth = morphology.erosion(threshdepth, selem=np.ones((morphW, morphW)))
    return_values['morph'] = threshdepth.copy()
    # threshdepth = morphology.dilation(threshdepth, selem=np.ones((morphW, morphW)))

    #connected components
    all_labels, num_labels = measure.label(threshdepth, neighbors=8, background=0, return_num=True)
    #find cc that are greater than X
    numObj = 0
    out_labels = np.zeros(all_labels.shape)
    for i in range(1, num_labels):
        ind = np.where(all_labels == i)
        if len(ind[0]) < minNumCC or len(ind[0]) > maxNumCC:
            all_labels[ind] = 0
        else:
            out_labels[ind] = numObj+1
            numObj = numObj + 1

    #where obj labels is non zero that is an object
    return_values['cc'] = out_labels.copy()

    #for each of the objects get the bounding square
    rects = []
    for i in range(1, numObj+1):
        #find element in out_labels == i
        indices = np.array((out_labels == i).nonzero()).astype('float32')
        indices = np.transpose(indices)
        # y,x,h,w = cv2.boundingRect(indices)
        # cv2.rectangle(fullcolor,(x,y),(x+w,y+h),(0,255,0),2)
        # rects.append([y,y+h, x, x+w])

        #min area rectangle
        rect = cv2.minAreaRect(indices)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        rects.append(box)
        newbox = np.array([ [box[0][1], box[0][0]], [box[1][1], box[1][0]], [box[2][1], box[2][0]], [box[3][1], box[3][0]]  ])
        cv2.drawContours(fullcolor, [newbox], 0 , (0,255,255), 2)  

    if debug:
        plt.imshow(fullcolor)
        plt.show()

    #take the coordinates from rect and get those images in the full color image
    tinyColorImgs = []
    tinyDepthImgs = []
    # for i in range(len(rects)):
    #     startY = rects[i][0]
    #     endY = rects[i][1]
    #     startX = rects[i][2]
    #     endX = rects[i][3]
    #     colorStartX = startX
    #     colorStartY = startY
    #     colorEndX = endX
    #     colorEndY = endY
    #     #get from depth to color coordinates if we have extrinsics
    #     if not extrinsics is None:
    #         point = rs.float3()
    #         #transform start
    #         point.x = startX
    #         point.y = startY
    #         point.z = 0
    #         newpt = extrinsics.transform(point)
    #         colorStartX = int(newpt.x)
    #         colorStartY = int(newpt.y)
    #         #transform end
    #         point.x = endX
    #         point.y = endY
    #         point.z = 0
    #         newpt = extrinsics.transform(point)
    #         colorEndX = int(newpt.x)
    #         colorEndY = int(newpt.y)

    #     #dont use the image with the rectangles drawn on it
    #     img = fcolor[colorStartY:colorEndY, colorStartX:colorEndX]
    #     tinyColorImgs.append(img)
    #     img = np.zeros(depthImage.shape)
    #     img[startY:endY, startX:endX] = depthImage[startY:endY, startX:endX]
    #     tinyDepthImgs.append(img)

    imagesForDL = []
    # for i in range(len(tinyColorImgs)):
    #     #create zero image
    #     if tinyColorImgs[i].shape[0] > 256 or tinyColorImgs[i].shape[1] > 256:
    #         #large image, put it in 512, 512
    #         bg = np.zeros((512,512, 3)) #TODO assumes 512x512, but isnt always the case
    #         startY = int(bg.shape[0]/2 - tinyColorImgs[i].shape[0]/2)
    #         startX = int(bg.shape[1]/2 - tinyColorImgs[i].shape[1]/2)
    #         if startX < 0:
    #             startX = 0
    #         if startY < 0:
    #             startY = 0
    #         bg[startY:startY +tinyColorImgs[i].shape[0], startX:startX+tinyColorImgs[i].shape[1]] = tinyColorImgs[i]
    #         imagesForDL.append(bg)
    #     else:
    #         #small image put it in 256, 256
    #         bg = np.zeros((256,256, 3))
    #         startY = int(bg.shape[0]/2 - tinyColorImgs[i].shape[0]/2)
    #         startX = int(bg.shape[1]/2 - tinyColorImgs[i].shape[1]/2)
    #         if startX < 0:
    #             startX = 0
    #         if startY < 0:
    #             startY = 0
    #         bg[startY:startY +tinyColorImgs[i].shape[0], startX:startX+tinyColorImgs[i].shape[1]] = tinyColorImgs[i]
    #         imagesForDL.append(bg)
            
    #these images are stored in BGR deep learning expects RGB!
    #tiny depth images are the size of the full depth image and non zero where the object is
    return_values['DL_images'] = imagesForDL
    return_values['depth_images'] = tinyColorImgs
    return_values['boxed_color'] = fullcolor
    return return_values



class SegmentationGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(SegmentationGUI, self).__init__()
        self.initUI()
        self.depth_image = None
        self.color_image = None
        self.params = DepthSegmentationParams()
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
        self.file_name_textbox.setText('/home/bk/Documents/arc_images/competition_images/cluttered_test/3')
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
        self.param_locations = ['Med filter width', "Max CC", "Max depth", "Threshold", "Morph width", "Min number CC"]
        param_initialvals = [7, 2000, 7000, 20, 3, 500]
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
            spinbox.editingFinished.connect(self.run_segmentation)
            self.param_spinboxes.append(spinbox)
            vert_layout.addWidget(parameter_name)
            vert_layout.addWidget(spinbox)

            row = i//(len(self.param_locations)/2)
            col = i % (len(self.param_locations)/2) + 1
            self.grid_layout_mid.addItem(vert_layout,row, col)
        
        self.horzMid.addItem(self.grid_layout_mid)
        
        #bot
        self.grid_layout_bot = QtWidgets.QGridLayout()
        self.seg_steps = ['Input', 'Median filter', 'Gradient filter', 'Threshold', 'Connected Components', 'Output']
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
        # ['Med filter width', "Min depth", "Max depth", "Threshold", "Morph width", "Min number CC"]
        self.params.medianFilterW = int(self.param_spinboxes[0].value())
        self.params.maxNumCC = self.param_spinboxes[1].value()
        self.params.maxDepth = self.param_spinboxes[2].value()
        self.params.gradientThreshold = self.param_spinboxes[3].value()
        self.params.morphW = int(self.param_spinboxes[4].value())
        self.params.minNumCC = int(self.param_spinboxes[5].value())

        logger.debug("Params Med filter = {} Min depth = {} Max depth = {} Threshold = {} Morph width = {} Min CC = {}".format(self.params.medianFilterW, self.params.minDepth, self.params.maxDepth, self.params.gradientThreshold, self.params.morphW, self.params.minNumCC))

        if self.depth_image is None:
            logger.warn("No image has been loaded")
            return
        #segment
        ret = depthSegmentation(self.depth_image, self.color_image,None, self.params)

        #update the images 
        # ['Input', 'Median filter', 'Gradient filter', 'Threshold', 'Connected Components', 'Output']
        zeroImg = self.depth_image
        zeroImg = (zeroImg - zeroImg.min()) / (zeroImg.max()-zeroImg.min())
        zeroImg *= 255
        zeroImg = zeroImg.astype('uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1], QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[0].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['median_filter']
        #scale from zero to 255
        zeroImg = (zeroImg - zeroImg.min()) / (zeroImg.max()-zeroImg.min())
        zeroImg *= 255
        zeroImg = zeroImg.astype('uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1],QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[1].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['gradient_filter']
        zeroImg = (zeroImg - zeroImg.min()) / (zeroImg.max()-zeroImg.min())
        zeroImg *= 255
        zeroImg = zeroImg.astype('uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1],QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[2].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['morph']
        zeroImg = (zeroImg - zeroImg.min()) / (zeroImg.max()-zeroImg.min())
        zeroImg *= 255
        zeroImg = zeroImg.astype('uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1],QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[3].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['cc']
        zeroImg = zeroImg.astype('uint8')
        zeroImg[np.where(zeroImg)] = 255
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1], QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[4].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['boxed_color']
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[5].setPixmap(pix.scaled(320, 240))

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    # #set up the window
    sg = SegmentationGUI()
    sg.show()
    sys.exit(app.exec_())

    # d = np.load('/home/bk/Documents/arc_images/competition_images/cluttered_test/3_depth.npy')
    # c = np.load('/home/bk/Documents/arc_images/competition_images/cluttered_test/3.npy')
    # depthSegmentation(d, c, debug=True)

