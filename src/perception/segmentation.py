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


class GraphSegmentationParams():
    def __init__(self):

        #parameters for the graph cut segmentation
        self.minSize = 500
        self.k = 300
        self.sigma = 0.5
        
        #filtering parameter for the depth image
        self.medianFilterW = 7

        #filtering parameters for the mean shift filtering
        self.sp_rad = 3     #spatial window radius
        self.c_rad = 3     #color window radius
        
        self.topLeft = (70, 55)
        self.topRight = (549, 83)
        self.botLeft = (41, 337)
        self.botRight = (550, 366)

def graphSegmentation(depthImage, fcolor, extrinsics=np.eye(4), params=GraphSegmentationParams(), debug=False):
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

    #make the depth image a 4D matrix so we can transform it
    depth4D = np.zeros((imageH,imageW,4))
    [xs, ys] = np.meshgrid(range(imageW), range(imageH))
    depth4D[:,:,0] = xs
    depth4D[:,:,1] = ys
    depth4D[:,:,2] = 0
    depth4D[:,:,3] = 1
    #transform all points in depth image to full color
    #ref: http://stackoverflow.com/questions/28255351/numpy-dot-product-and-matrix-product
    depth_t_color = np.einsum('...jk,...k->...j', extrinsics , depth4D)

    color_depth_image = np.zeros((imageH, imageW, 4))
    #fill the first three channels with the LAB image
    color_depth_image[:,:,0:3] = labcolor
    xmap = depth_t_color[:,:,0].astype('float32')
    ymap = depth_t_color[:,:,1].astype('float32')
    ndepth = cv2.remap(depth, xmap, ymap, cv2.INTER_CUBIC)
    color_depth_image[:,:,3] = ndepth

    #create rectangle to remove any unwatned points
    mask = np.zeros((imageH,imageW))
    a = np.array(params.topLeft)
    b = np.array(params.topRight)
    c = np.array(params.botRight)
    AB = b-a
    BC = c-b
    [xs, ys] = np.meshgrid(range(imageW), range(imageH))
    c = np.zeros((imageH,imageW,2))
    c[:,:,0] = xs
    c[:,:,1] = ys
    c = c-a
    mask = np.logical_and(np.logical_and(c.dot(AB) > 0, c.dot(AB) < AB.dot(AB)), np.logical_and(c.dot(BC) > 0, c.dot(BC) < BC.dot(BC)))

    #remove any points outside the desired rectangle
    for i in range(4):
        color_depth_image[:,:,i] = np.where(mask == 1, color_depth_image[:,:,i], 0)



    #perform the graph segmentation
    gs = cv2.ximgproc.segmentation.createGraphSegmentation()
    gs.setSigma(params.sigma)
    gs.setK(params.k)
    gs.setMinSize(int(params.minSize))
    outp = gs.processImage(color_depth_image)
    numObj = outp.max()
    logger.info("Found {} segments in the image".format(outp.max()))
    outp = np.where(outp == 0, 255, outp)
    return_values['segmented_image'] = outp.copy()
    
    segments = []

    #create a copy of the full color image we can draw rectangles on
    fcolor_rects = fcolor.copy()

    #extract the sub image for each label based on the minimum bounding rectangle
    tinyColorImgs = []
    imagesForDL = []
    #ref : http://stackoverflow.com/questions/37177811/crop-rectangle-returned-by-minarearect-opencv-python
    for i in range(numObj):
        #find element in outp == i
        indices = np.array((outp == i).nonzero())
        if indices.shape[1] > 0:
            segments.append(np.transpose(indices.astype('int32')))
        else:
            continue
        indices = np.transpose(indices.astype('float32'))
        
        #min area rectangle
        rect = cv2.minAreaRect(indices)

        angle = rect[2]
        rows = imageH
        cols = imageW
        M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
        img_rot = cv2.warpAffine(fcolor,M,(cols,rows))

        # rotate bounding box
        rect0 = (rect[0], rect[1], 0.0)
        box = cv2.boxPoints(rect)
        pts = np.int0(cv2.transform(np.array([box]), M))[0]    
        pts[pts < 0] = 0

        # crop
        img_crop = img_rot[pts[1][1]:pts[0][1], pts[1][0]:pts[2][0]]
        

        #make image for deep learning
        if (img_crop.shape[0] > 256 or img_crop.shape[1] > 256) and (img_crop.shape[0] < 512 and img_crop.shape[1] < 512):
            #large image, put it in 512, 512
            tinyColorImgs.append(img_crop)
            bg = np.zeros((512,512, 3))
            startY = int(bg.shape[0]/2 - img_crop.shape[0]/2)
            startX = int(bg.shape[1]/2 - img_crop.shape[1]/2)
            if startX < 0:
                startX = 0
            if startY < 0:
                startY = 0
            bg[startY:startY +img_crop.shape[0], startX:startX+img_crop.shape[1]] = 255-img_crop
            imagesForDL.append(bg)
            box = np.int0(box)
            newbox = np.array([ [box[0][1], box[0][0]], [box[1][1], box[1][0]], [box[2][1], box[2][0]], [box[3][1], box[3][0]]  ])
            cv2.drawContours(fcolor_rects, [newbox], 0 , (0,255,255), 2)
        elif img_crop.shape[0] <= 256 and img_crop.shape[1] <= 256:
            #small image put it in 256, 256
            tinyColorImgs.append(img_crop)
            bg = np.zeros((256,256, 3))
            startY = int(bg.shape[0]/2 - img_crop.shape[0]/2)
            startX = int(bg.shape[1]/2 - img_crop.shape[1]/2)
            if startX < 0:
                startX = 0
            if startY < 0:
                startY = 0
            bg[startY:startY +img_crop.shape[0], startX:startX+img_crop.shape[1]] = 255-img_crop
            imagesForDL.append(bg)
            box = np.int0(box)
            newbox = np.array([ [box[0][1], box[0][0]], [box[1][1], box[1][0]], [box[2][1], box[2][0]], [box[3][1], box[3][0]]  ])
            cv2.drawContours(fcolor_rects, [newbox], 0 , (0,255,255), 2)
        else:
            #image was really big and is probably the entire image or some mistake, so we are not adding it
            logger.warn("Segment was larger than 512x512. This is probably a mistake, not adding to identification set")
     
    return_values['boxes_image'] = fcolor_rects
    #tiny depth images are the size of the full depth image and non zero where the object is
    return_values['DL_images'] = imagesForDL
    return_values['segments'] = tinyColorImgs
    return_values['pixel_locations'] = segments
    return return_values



class SegmentationGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(SegmentationGUI, self).__init__()
        self.initUI()
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
        self.param_locations = ['Med filter width', "k", "sigma", "Min number", "Spatial Radius", "Color radius"]
        param_initialvals = [3, 250, 0.5, 200, 3, 3]
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
        self.params.sp_rad = int(self.param_spinboxes[4].value())
        self.params.c_rad = int(self.param_spinboxes[5].value())

        logger.debug("Params Med filter = {} K = {} Sigma = {} Min Size = {} SP Rad {} Color Rad = {}".format(self.params.medianFilterW, self.params.k, self.params.sigma, self.params.minSize, self.params.sp_rad, self.params.c_rad))

        if self.depth_image is None:
            logger.warn("No image has been loaded")
            return
        #segment
        testmat = np.array([[ 0.99999267,  0.00340036, -0.00176075, -0.02570188],
       [-0.00339687,  0.99999225,  0.00198321, -0.00134699],
       [ 0.00176748, -0.00197721,  0.99999648, -0.00356173],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

        ret = graphSegmentation(self.depth_image, self.color_image, testmat, self.params)

        #update the images 
        # ['Input', 'Median filter', 'Gradient filter', 'Threshold', 'Connected Components', 'Output']
        zeroImg = self.color_image
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[0].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['depth_filter']
        #scale from zero to 255
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1],QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[1].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['lab_filter']
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[2].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['segmented_image'].astype('uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1],QtGui.QImage.Format_Grayscale8)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[3].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['DL_images'][1]
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1]*3, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[4].setPixmap(pix.scaled(320, 240))

        zeroImg = ret['boxes_image']
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.seg_step_displays[5].setPixmap(pix.scaled(320, 240))

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    # #set up the window
    sg = SegmentationGUI()
    sg.show()
    sys.exit(app.exec_())

