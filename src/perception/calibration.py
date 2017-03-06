from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import cv2
import math
import numpy as np
import os
import logging
import sys
sys.path.append('../hardware/SR300/')
import time
import realsense as rs
#for when we want to read the current position of the robot from the server
# from pensive.client import PensiveClient
# from pensive.coders import register_numpy

logger = logging.getLogger(__name__)
# configure the root logger to accept all records
logger = logging.getLogger()
logger.setLevel(logging.NOTSET)

formatter = logging.Formatter('%(asctime)s\t[%(name)s] %(pathname)s:%(lineno)d\t%(levelname)s:\t%(message)s')

# set up colored logging to console
from rainbow_logging_handler import RainbowLoggingHandler
console_handler = RainbowLoggingHandler(sys.stderr)
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

class RealsenseCalibration(QtWidgets.QWidget):
    def __init__(self):
        super(RealsenseCalibration, self).__init__()
        self.initUI()
        self.thread = VideoThread()
        self.thread.signal.connect(self.updateView)
        cameramat, cameracoeff = self.thread.getIntrinsics()
        if cameramat is None:
            logger.warning("Unable to get camera coefficients for camera. Setting to zeros...")
            cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
            cameracoeff = np.zeros((5))

        self.camera_matrix = cameramat
        self.camera_coeff = cameracoeff
        self.cameraXform = np.identity(4)
    def initUI(self):

        #make the layout vertical
        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)

        #add places to enter/show the 4x4 matrix of the current postion of the robot
        self.horzMid = QtWidgets.QHBoxLayout()
        self.gridRobot = QtWidgets.QGridLayout()
        self.gridCamera = QtWidgets.QGridLayout()
        self.horzMid.addItem(self.gridRobot)
        spacer =QtWidgets.QLabel("     ",self)
        self.horzMid.addWidget(spacer)
        spacer.show()
        self.horzMid.addItem(self.gridCamera)
        self.robotTextBoxes = []
        self.cameraTextBoxes = []
        for row in range(4):
            self.robotTextBoxes.append([])
            self.cameraTextBoxes.append([])
            for col in range(4):
                #add textbox to the grid
                tbox = QtWidgets.QLineEdit(self)
                self.robotTextBoxes[row].append(tbox)
                self.gridRobot.addWidget(self.robotTextBoxes[row][col], row, col)

                tbox = QtWidgets.QLineEdit(self)
                self.cameraTextBoxes[row].append(tbox)
                self.gridCamera.addWidget(self.cameraTextBoxes[row][col], row, col)

        label = QtWidgets.QLabel("Camera Xform",self)
        self.gridCamera.addWidget(label, 5, 2)
        label.show()
        label = QtWidgets.QLabel("Robot Xform",self)
        self.gridRobot.addWidget(label, 5, 2)
        label.show()

        #set initial values for the matrices
        eye = np.identity(4)
        for row in range(4):
            for col in range(4):
                self.robotTextBoxes[row][col].setText(str(eye[row, col]))
                self.cameraTextBoxes[row][col].setText(str(eye[row, col]))
        
        #add a push button at the top
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzTop.addStretch(1)
        self.calibrate_button = QtWidgets.QPushButton("Calibrate", self)
        self.save_button = QtWidgets.QPushButton("Save", self)
        self.horzTop.addWidget(self.calibrate_button)
        self.horzTop.addWidget(self.save_button)
        self.horzTop.addStretch(1)
        self.calibrate_button.clicked.connect(self.calibrate_camera)
        self.save_button.clicked.connect(self.save_calibration)

        #add widget to show what the camera sees
        self.camera_display = QtWidgets.QLabel(self)
        zeroImg = np.zeros((480,640,3),dtype='uint8')
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display.setPixmap(pix)
        self.horzBot = QtWidgets.QHBoxLayout()
        self.horzBot.addStretch(1)
        self.horzBot.addWidget(self.camera_display)
        self.horzBot.addStretch(1)

        #add everthing to the layout
        self.layout.addItem(self.horzTop)
        self.layout.addItem(self.horzMid)
        self.layout.addItem(self.horzBot)

     
        self.calibrate_button.show()

    
    def calibrate_camera(self):
        if self.thread.isRunning():
            self.thread.keepRunning = False
        else:
            self.thread.keepRunning = True
            self.thread.start()

    def updateView(self, image):
        pix_img = QtGui.QImage(image, image.shape[1], image.shape[0], image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(pix_img)
        self.camera_display.setPixmap(pix)

        t = time.time()
        #set the dictionary
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

        #need to measure out a target
        board = cv2.aruco.CharucoBoard_create(8,11,.0172, 0.0125, dictionary)

        #get the camera transform
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        res = cv2.aruco.detectMarkers(gray,dictionary)
        if len(res[0])>0:
            pose = cv2.aruco.estimatePoseBoard(res[0], res[1], board, self.camera_matrix, self.camera_coeff)
            rotMat = cv2.Rodrigues(pose[1]) #returns rotation vector and translation vector
            rotMat = rotMat[0]              #returns rotation matrix and jacobian
            xform = np.zeros((4,4))
            xform[0:3, 0:3] = rotMat
            xform[0:3, 3] = pose[2].flatten()
            xform[3, 3] = 1
            self.cameraXform = xform
        else:
            logger.warning("Unable to get the camera transform. Camera cannot see Charuco.")
            xform = np.zeros((4,4))
            xform[0,0] = 1
            xform[1,1] = 1
            xform[2,2] = 1
            xform[3,3] = 1
            self.cameraXform = xform

        #update the gui
        for row in range(4):
            for col in range(4):
                self.cameraTextBoxes[row][col].setText(str(self.cameraXform[row, col]))
        # logger.info(time.time()-t)

    def save_calibration(self):
        #get the camera serial number
        sn = self.thread.serialNum

        #get the robot matrix
        robotmat = np.identity(4)
        for row in range(4):
            for col in range(4):
                num = float(self.robotTextBoxes[row][col].text())
                robotmat[row, col] = num

        
        #multiply the transforms
        full_transform = self.cameraXform * robotmat
        print(full_transform)

        #save it
        np.save(sn + "_xform", full_transform)

class VideoThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(np.ndarray)
    def __init__(self):
        QtCore.QThread.__init__(self)
        self.keepRunning = True
        self.serialNum = ''

    def getIntrinsics(self):
        context = rs.context()
        if context.get_device_count() == 0:
            logger.warn("No cameras attached")
            mat = np.zeros((3,3))
            coeffs = np.zeros((5))
            return (mat, coeffs)

        cam = context.get_device(0)
        try:
            self.serialNum = cam.get_info(rs.camera_info_serial_number)
            cam.enable_stream(rs.stream_color, rs.preset_best_quality)
            intrinsics = cam.get_stream_intrinsics(rs.stream_color)
        except:
            logger.exception("Unable to enable stream and get intrinsics")
            return (None, None)
        mat = np.zeros((3,3))
        mat[0, 0] = intrinsics.fx
        mat[0, 2] = intrinsics.ppx
        mat[1, 1] = intrinsics.fy
        mat[1, 2] = intrinsics.ppy
        mat[2, 2] = 1

        coeffs = np.zeros((5))
        coeffs[0] = rs.floatp_getitem(intrinsics.coeffs, 0)
        coeffs[1] = rs.floatp_getitem(intrinsics.coeffs, 1)
        coeffs[2] = rs.floatp_getitem(intrinsics.coeffs, 2)
        coeffs[3] = rs.floatp_getitem(intrinsics.coeffs, 3)
        coeffs[4] = rs.floatp_getitem(intrinsics.coeffs, 4)

        return (mat, coeffs)

    def run(self):
        context = rs.context()
        if context.get_device_count() == 0:
            logger.warn("No cameras attached")
            return
        cam = context.get_device(0)
        try:
            #enable the stream
            for s in [rs.stream_color, rs.stream_depth, rs.stream_infrared]:
                cam.enable_stream(s, rs.preset_best_quality)
        except:
            logger.exception("Could not enable the stream")
            return
        c = 0
        cam.start()
        t = time.time()
        while self.keepRunning:
            cam.wait_for_frames()
            imageFullColor = cam.get_frame_data_u8(rs.stream_color)
            if imageFullColor.size == 1:
                #something went wrong
                logger.error("Could not capture the full color image. Size of requested stream did not match")
                imageFullColor = None
            else:
                width = cam.get_stream_width(rs.stream_color)
                height = cam.get_stream_height(rs.stream_color)    
                imageFullColor = np.reshape(imageFullColor, (height, width, 3) )

            imageDepth = cam.get_frame_data_u16(rs.stream_depth)
            if imageDepth.size == 1:
                logger.error("Could not capture the depth image. Size of requested stream did not match")
                imageDepth = None
            else:
                width = cam.get_stream_width(rs.stream_depth)
                height = cam.get_stream_height(rs.stream_depth)    
                imageDepth = np.reshape(imageDepth, (height, width) )

            # logger.info(1/(time.time()-t))
            # t = time.time()
            if c%20 == 1:
                self.signal.emit(imageFullColor)
            # QtWidgets.QApplication.instance().processEvents()
            c = c+1
        cam.stop()

def main():
    app = QtWidgets.QApplication(sys.argv)

    #set up the window
    rc = RealsenseCalibration()
    rc.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()        

