from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
import sys
import time
from OpenGL import GL, GLU
import numpy as np
sys.path.append('../hardware/SR300/')
import realsense as rs
import scipy
import os
from perceptionGUI import GLWidget
from calibration import VideoThread
import logging
logger = logging.getLogger()
class MakeModelsGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(MakeModelsGUI, self).__init__()
        self.openGLWidget = GLWidget(self)
        self.openGLWidget.setMinimumSize(640,480)
        ctx = rs.context()
        self.cam0 = VideoThread(ctx, 0)
        self.cam1 = VideoThread(ctx, 1)
        self.cam0.signal.connect(self.updateView)
        self.cam1.signal.connect(self.updateView)
        self.initUI()
        self.image_num = 0
        self.obj_name = ""
        self.show()

    def initUI(self):

        #3 levels
        #take image    undo image     save cloud
        #object rotation x, y, z
        #3d view  camera view
        
        self.vertLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.vertLayout)

        #add horizontal layouts
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzMid = QtWidgets.QHBoxLayout()
        self.horzBot = QtWidgets.QHBoxLayout()

        #top
        self.take_image_button = QtWidgets.QPushButton("Take image", self)
        self.remove_last_image_button = QtWidgets.QPushButton("Remove last image", self)
        self.save_button = QtWidgets.QPushButton("Save", self)
        self.video_capture_button = QtWidgets.QPushButton("Video Capture", self)
        self.take_image_button.setMaximumWidth(75)
        self.video_capture_button.setMaximumWidth(105)
        self.remove_last_image_button.setMaximumWidth(125)
        self.save_button.setMaximumWidth(75)
        self.take_image_button.show()
        self.video_capture_button.show()
        self.remove_last_image_button.show()
        self.save_button.show()
        self.horzTop.addStretch(1)
        self.horzTop.addWidget(self.video_capture_button)
        self.horzTop.addSpacing(20)
        self.horzTop.addWidget(self.take_image_button)
        self.horzTop.addSpacing(20)
        self.horzTop.addWidget(self.remove_last_image_button)
        self.horzTop.addSpacing(20)
        self.horzTop.addWidget(self.save_button)
        self.horzTop.addStretch(1)

        #connections 
        self.take_image_button.pressed.connect(self.take_images)
        self.video_capture_button.pressed.connect(self.start_stop_video_capture)

        #mid
        self.rotx_text = QtWidgets.QTextEdit(self)
        self.roty_text = QtWidgets.QTextEdit(self)
        self.rotz_text = QtWidgets.QTextEdit(self)
        self.objectNameTextBox = QtWidgets.QLineEdit(self)
        self.objectNameTextBox.editingFinished.connect(self.updateObjectName)
        self.rotx_text.setMaximumHeight(35)
        self.roty_text.setMaximumHeight(35)
        self.rotz_text.setMaximumHeight(35)
        self.objectNameTextBox.setMaximumHeight(35)

        self.horzMid.addStretch(1)
        label = QtWidgets.QLabel(self)
        label.setText("Object Name")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.objectNameTextBox)
        self.horzMid.addSpacing(30)

        label = QtWidgets.QLabel(self)
        label.setText("X rotation")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.rotx_text)
        self.horzMid.addSpacing(10)

        label = QtWidgets.QLabel(self)
        label.setText("Y rotation")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.roty_text)
        self.horzMid.addSpacing(10)
        
        label = QtWidgets.QLabel(self)
        label.setText("Z rotation")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.rotz_text)

        self.horzMid.addStretch(1)
        

        #bot
        self.vertBot = QtWidgets.QVBoxLayout()
        self.camera_display0 = QtWidgets.QLabel(self)
        self.camera_display0.show()
        zeroImg = np.zeros((480,640,3),dtype='uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display0.setPixmap(pix.scaled(320, 240))

        self.camera_display1 = QtWidgets.QLabel(self)
        self.camera_display1.show()
        zeroImg = np.zeros((480,640,3),dtype='uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display1.setPixmap(pix.scaled(320, 240))

        self.vertBot.addWidget(self.camera_display0)
        self.vertBot.addSpacing(20)
        self.vertBot.addWidget(self.camera_display1)

        self.horzBot.addWidget(self.openGLWidget)
        self.horzBot.addSpacing(20)
        self.horzBot.addItem(self.vertBot)

        #add all layouts to the vertical layout
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzMid)
        self.vertLayout.addItem(self.horzBot)


    def take_images(self):
        pass

    def updateObjectName(self):
        self.obj_name = self.objectNameTextBox.text()
        #if the folder isnt there create it
        folder_names = os.listdir()
        if not self.obj_name in folder_names:
            os.mkdir(self.obj_name)

    def start_stop_video_capture(self):
        if self.cam0.isRunning():
            logger.info("Stopping image capture on camera 0")
            self.cam0.keepRunning = False
        else:
            self.cam0.keepRunning = True
            while not self.cam0.can_emit:
                self.cam0.can_emit = True
            self.cam0.start()
            logger.info("Starting image capture on camera 0")
        if self.cam1.isRunning():
            self.cam1.keepRunning = False
            logger.info("Stopping image capture on camera 1")
        else:
            self.cam1.keepRunning = True
            while not self.cam1.can_emit:
                self.cam1.can_emit = True
            self.cam1.start()
            logger.info("Starting image capture on camera 1")

    def updateView(self, image, aligned_image, point_cloud, cam_num):
        logger.info("Received image from camera {}".format(cam_num))
        pix_img = QtGui.QImage(aligned_image, image.shape[1], image.shape[0], image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(pix_img)
        if cam_num == 0:
            self.camera_display0.setPixmap(pix.scaled(320,240))
            self.cam0.mutex.acquire()
            self.cam0.can_emit = True
            self.cam0.mutex.release()
        elif cam_num == 1:
            self.camera_display1.setPixmap(pix.scaled(320,240))
            self.cam1.mutex.acquire()
            self.cam1.can_emit = True
            self.cam1.mutex.release()
        else:
            logger.error("did not get cam num")

        #save the images
        if self.obj_name != "":
            scipy.misc.imsave(self.obj_name + "/" + str(self.image_num)+ ".png", image)
            self.image_num += 1

def main():
    app = QtWidgets.QApplication(sys.argv)

    #set up the window
    m3 = MakeModelsGUI()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()