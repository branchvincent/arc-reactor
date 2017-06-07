from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
import sys
import time
from OpenGL import GL, GLU
import numpy as np
import realsense as rs
import os
from calibration import VideoThread
import logging
logger = logging.getLogger(__name__)
class TakeImagesGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(TakeImagesGUI, self).__init__()
        ctx = rs.context()
        self.cam0 = VideoThread(ctx, 0)
        self.cam1 = VideoThread(ctx, 1)
        self.cam0.signal.connect(self.updateView)
        self.cam1.signal.connect(self.updateView)
        self.initUI()
        self.timer = QtCore.QTimer()
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
    
        self.video_capture_button = QtWidgets.QPushButton("Video Capture", self)
        self.video_capture_button.setMaximumWidth(105)
        self.video_capture_button.show()
        self.horzTop.addStretch(1)
        self.horzTop.addWidget(self.video_capture_button)
        self.horzTop.addStretch(1)
        
        #connections 
        self.video_capture_button.pressed.connect(self.start_video_capture)

        #mid
        self.objectNameTextBox = QtWidgets.QLineEdit(self)
        self.objectNameTextBox.editingFinished.connect(self.updateObjectName)
        self.objectNameTextBox.setMaximumHeight(35)

        self.horzMid.addStretch(1)
        label = QtWidgets.QLabel(self)
        label.setText("Object Name")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.objectNameTextBox)
        self.horzMid.addSpacing(30)

        self.horzMid.addStretch(1)
        

        #bot
        self.camera_display0 = QtWidgets.QLabel(self)
        self.camera_display0.show()
        zeroImg = np.zeros((480,640,3),dtype='uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display0.setPixmap(pix.scaled(640,480))

        self.camera_display1 = QtWidgets.QLabel(self)
        self.camera_display1.show()
        zeroImg = np.zeros((480,640,3),dtype='uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display1.setPixmap(pix.scaled(640,480))

        self.horzBot.addWidget(self.camera_display0)
        self.horzBot.addSpacing(20)
        self.horzBot.addWidget(self.camera_display1)

        #add all layouts to the vertical layout
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzMid)
        self.vertLayout.addItem(self.horzBot)

    def updateObjectName(self):
        self.obj_name = self.objectNameTextBox.text()
        #if the folder isnt there create it
        folder_names = os.listdir()
        if not self.obj_name in folder_names:
            os.mkdir(self.obj_name)

    def start_video_capture(self):
        #add timer for 50 seconds
        self.timer.timeout.connect(self.stop_video_capture)
        if self.timer.isActive():
            return
        else:
            self.timer.setSingleShot(True)
            self.timer.setInterval(10000)
            self.timer.start()
        self.cam0.keepRunning = True
        while not self.cam0.can_emit:
            self.cam0.can_emit = True
        self.cam0.start()
        logger.info("Starting image capture on camera 0")
       
        self.cam1.keepRunning = True
        while not self.cam1.can_emit:
            self.cam1.can_emit = True
        self.cam1.laseron = False
        self.cam1.start()
        logger.info("Starting image capture on camera 1")

    def stop_video_capture(self):
        logger.info("Stopping image capture on camera 0")
        logger.info("Stopping image capture on camera 1")
        self.cam0.keepRunning = False
        self.cam1.keepRunning = False

    def updateView(self, image, aligned_image, point_cloud, cam_num):
        logger.info("Received image from camera {}".format(cam_num))
        pix_img = QtGui.QImage(image, image.shape[1], image.shape[0], image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(pix_img)
        if cam_num == 0:
            self.camera_display0.setPixmap(pix.scaled(640,480))
            self.cam0.mutex.acquire()
            self.cam0.can_emit = True
            self.cam0.mutex.release()
        elif cam_num == 1:
            self.camera_display1.setPixmap(pix.scaled(640,480))
            self.cam1.mutex.acquire()
            self.cam1.can_emit = True
            self.cam1.mutex.release()
        else:
            logger.error("did not get cam num")

        #save the images
        if self.obj_name != "":
            np.save(self.obj_name + "/image_" + str(self.image_num), image)
            if cam_num == 0:
                np.save(self.obj_name + "/aligned_image_" + str(self.image_num), aligned_image)
                np.save(self.obj_name + "/pc_" + str(self.image_num), point_cloud)
            self.image_num += 1

class TakeImagesGUI2(QtWidgets.QWidget):
    
    def __init__(self):
        super(TakeImagesGUI2, self).__init__()
        ctx = rs.context()
        self.cam0 = VideoThread(ctx, 0)
        self.cam1 = VideoThread(ctx, 1)
        self.cam0.signal.connect(self.updateView1)
        self.cam1.signal.connect(self.updateView2)
        self.cam0.keepRunning = False
        self.cam1.keepRunning = False
        self.initUI()
        self.timer = QtCore.QTimer()
        self.image_num = 0
        self.obj_name = ""
        self.capture_frame1 = False
        self.capture_frame2 = False
        self.show()

    def initUI(self):

        self.vertLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.vertLayout)

        #add horizontal layouts
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzMid = QtWidgets.QHBoxLayout()
        self.horzBot = QtWidgets.QHBoxLayout()

        #top
    
        self.video_capture_button = QtWidgets.QPushButton("Video Capture", self)
        self.video_capture_button.setMaximumWidth(105)
        self.video_capture_button.show()
        self.take_image_button = QtWidgets.QPushButton("Take image", self)
        self.take_image_button.setMaximumWidth(105)
        self.take_image_button.show()

        self.horzTop.addStretch(1)
        self.horzTop.addWidget(self.video_capture_button)
        self.horzTop.addSpacing(20)
        self.horzTop.addWidget(self.take_image_button)
        self.horzTop.addStretch(1)
        
        #connections 
        self.video_capture_button.pressed.connect(self.start_video_capture)
        self.take_image_button.pressed.connect(self.take_image)

        #mid
        self.objectNameTextBox = QtWidgets.QLineEdit(self)
        self.objectNameTextBox.editingFinished.connect(self.updateObjectName)
        self.objectNameTextBox.setMaximumHeight(35)

        self.horzMid.addStretch(1)
        label = QtWidgets.QLabel(self)
        label.setText("Object Name")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.objectNameTextBox)
        self.horzMid.addSpacing(30)

        self.horzMid.addStretch(1)
        

        #bot
        self.camera_display0 = QtWidgets.QLabel(self)
        self.camera_display0.show()
        zeroImg = np.zeros((480,640,3),dtype='uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display0.setPixmap(pix.scaled(640,480))

        self.camera_display1 = QtWidgets.QLabel(self)
        self.camera_display1.show()
        zeroImg = np.zeros((480,640,3),dtype='uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display1.setPixmap(pix.scaled(640,480))

        self.horzBot.addWidget(self.camera_display0)
        self.horzBot.addSpacing(20)
        self.horzBot.addWidget(self.camera_display1)

        #add all layouts to the vertical layout
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzMid)
        self.vertLayout.addItem(self.horzBot)

    def updateObjectName(self):
        self.obj_name = self.objectNameTextBox.text()
        #if the folder isnt there create it
        folder_names = os.listdir()
        if not self.obj_name in folder_names:
            logger.info("Creating directory {}".format(self.obj_name))
            os.mkdir(self.obj_name)

    def start_video_capture(self):
        if self.cam0.keepRunning:
            self.stop_video_capture()
            return

        self.cam0.keepRunning = True
        while not self.cam0.can_emit:
            self.cam0.can_emit = True
        self.cam0.start()
        logger.info("Starting image capture on camera 0")
       
        self.cam1.keepRunning = True
        while not self.cam1.can_emit:
            self.cam1.can_emit = True
        self.cam1.start()
        logger.info("Starting image capture on camera 1")

    def take_image(self):
        logger.info("Requesting to save images")
        self.capture_frame1 = True
        self.capture_frame2 = True

    def stop_video_capture(self):
        logger.info("Stopping image capture on camera 0")
        logger.info("Stopping image capture on camera 1")
        self.cam0.keepRunning = False
        self.cam1.keepRunning = False

    def updateView1(self, image, aligned_image, point_cloud, cam_num):
        logger.info("Received image from camera {}".format(cam_num))
        pix_img = QtGui.QImage(image, image.shape[1], image.shape[0], image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(pix_img)
        if cam_num == 0:
            self.camera_display0.setPixmap(pix.scaled(640,480))
            self.cam0.mutex.acquire()
            self.cam0.can_emit = True
            self.cam0.mutex.release()
        elif cam_num == 1:
            self.camera_display1.setPixmap(pix.scaled(640,480))
            self.cam1.mutex.acquire()
            self.cam1.can_emit = True
            self.cam1.mutex.release()
        else:
            logger.error("did not get cam num")

        #save the images
        if self.obj_name != "" and self.capture_frame1:
            np.save(self.obj_name + "/image_cam1_" + str(self.image_num), image)
            # if cam_num == 0:
            np.save(self.obj_name + "/aligned_image_cam1_" + str(self.image_num), aligned_image)
            np.save(self.obj_name + "/pc_cam1_" + str(self.image_num), point_cloud)
            self.image_num += 1
            logger.info("Image from cam 0 saved")
            self.capture_frame1 = False

    def updateView2(self, image, aligned_image, point_cloud, cam_num):
        logger.info("Received image from camera {}".format(cam_num))
        pix_img = QtGui.QImage(image, image.shape[1], image.shape[0], image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(pix_img)
        if cam_num == 0:
            self.camera_display0.setPixmap(pix.scaled(640,480))
            self.cam0.mutex.acquire()
            self.cam0.can_emit = True
            self.cam0.mutex.release()
        elif cam_num == 1:
            self.camera_display1.setPixmap(pix.scaled(640,480))
            self.cam1.mutex.acquire()
            self.cam1.can_emit = True
            self.cam1.mutex.release()
        else:
            logger.error("did not get cam num")

        #save the images
        if self.obj_name != "" and self.capture_frame2:
            np.save(self.obj_name + "/image_cam2_" + str(self.image_num), image)
            # if cam_num == 0:
            np.save(self.obj_name + "/aligned_image_cam2_" + str(self.image_num), aligned_image)
            np.save(self.obj_name + "/pc_cam2_" + str(self.image_num), point_cloud)
            self.image_num += 1
            logger.info("Image from cam 1 saved")
            self.capture_frame2 = False

def main():
    app = QtWidgets.QApplication(sys.argv)

    #set up the window
    m3 = TakeImagesGUI2()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()