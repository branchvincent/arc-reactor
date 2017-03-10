from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
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
sys.path.append('../')
from pensive.client import PensiveClient
from pensive.coders import register_numpy

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

        self.objectName = ""
        self.cameraName = ""

        self.db_client = PensiveClient(host='http://10.10.1.60:8888')
        self.store = self.db_client.default()
        register_numpy()
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
        self.objectXformLabel = QtWidgets.QLabel("Robot Xform",self)
        self.gridRobot.addWidget(self.objectXformLabel, 5, 2)
        self.objectXformLabel.show()

        #set initial values for the matrices
        eye = np.identity(4)
        for row in range(4):
            for col in range(4):
                self.robotTextBoxes[row][col].setText(str(eye[row, col]))
                self.cameraTextBoxes[row][col].setText(str(eye[row, col]))
        
        #add a push button at the top
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzTop.addStretch(1)
        self.gridTop = QtWidgets.QGridLayout()
        self.calibrate_button = QtWidgets.QPushButton("Calibrate", self)
        self.save_button = QtWidgets.QPushButton("Save", self)
        self.objectNameTextBox = QtWidgets.QLineEdit(self)
        self.objectNameTextBox.editingFinished.connect(self.updateObjectName)
        self.calib_cameraCheckBox = QtWidgets.QCheckBox("Calibrate Camera?",self)
        self.calib_cameraCheckBox.setChecked(True)
        self.calib_cameraCheckBox.show()
        self.cameraNameTextBox = QtWidgets.QLineEdit(self)
        self.cameraNameTextBox.editingFinished.connect(self.updateCameraName)

        self.gridTop.addWidget(self.calibrate_button, 0, 0)
        self.gridTop.addWidget(self.save_button, 0, 1)
        self.gridTop.addWidget(self.calib_cameraCheckBox, 0, 2)
        label = QtWidgets.QLabel("Camera name",self)
        label.show()
        self.gridTop.addWidget(label, 1, 0)
        self.gridTop.addWidget(self.cameraNameTextBox, 1, 1)
        label = QtWidgets.QLabel("Calibration Object Name", self)
        label.show()
        self.gridTop.addWidget(label, 2, 0)
        self.gridTop.addWidget(self.objectNameTextBox, 2, 1)
        self.horzTop.addLayout(self.gridTop)
        
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

    def updateCameraName(self):
        self.cameraName = self.cameraNameTextBox.text()

    def updateObjectName(self):
        self.objectName = self.objectNameTextBox.text()

    def updateView(self, image, aligned_image, point_cloud):
        
        #set the dictionary
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

        #need to measure out a target
        board = cv2.aruco.GridBoard_create(4, 6, .021, 0.003, dictionary)

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
            #draw what opencv is tracking
            #flip r and b channels to draw
            image = image[:,:,::-1].copy()
            cv2.aruco.drawAxis(image, self.camera_matrix, self.camera_coeff, pose[1], pose[2], 0.1)
            image = image[:,:,::-1].copy()
            #flip back
        else:
            logger.warning("Unable to get the camera transform. Camera cannot see Charuco.")
            xform = np.zeros((4,4))
            xform[0,0] = 1
            xform[1,1] = 1
            xform[2,2] = 1
            xform[3,3] = 1
            self.cameraXform = xform


        pix_img = QtGui.QImage(image, image.shape[1], image.shape[0], image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(pix_img)
        self.camera_display.setPixmap(pix)


        if self.calib_cameraCheckBox.isChecked():
            self.objectXformLabel.setText("Robot Xform")
            #calbrate the camera
            #get the base to end effector transform
            base2ee = self.store.get(key='robot/tcp_pose')
            #get the end effector to aruco pose
            ee2aruco = self.store.get(key='calibration/target_xform')

            #check if they actually exist
            if base2ee is None or ee2aruco is None:
                logger.warn("Did not recieve transforms from database. Not updating poses")
                return

            target_pose = base2ee.dot(ee2aruco)
            #update the gui
            for row in range(4):
                for col in range(4):
                    self.robotTextBoxes[row][col].setText(str(target_pose[row, col]))


            #multiply to get the camera world pose
            pose_of_obj = target_pose.dot(np.linalg.inv(self.cameraXform))
            #update the gui
            for row in range(4):
                for col in range(4):
                    self.cameraTextBoxes[row][col].setText(str(pose_of_obj[row, col]))
            # logger.info(time.time()-t)
            self.thread.can_emit = True

            #update the database with the camera world location
            if self.cameraName != "":
                # logger.debug("storing {}".format("camera/" + self.cameraName + "/pose"))
                self.store.put(key="camera/" + self.cameraName + "/pose", value=pose_of_obj)

        else:
            #change text of robotTextBoxes
            self.objectXformLabel.setText("Object Xform")
            #calibrate what is in the text box
            #read in camera world pose
            camera_xform = np.identity(4)
            if self.cameraName != "":
                camera_xform = self.store.get(key="camera/" + self.cameraName + "/pose")
            if camera_xform is None:
                logger.warn("Could not retrieve camera_xform")
                camera_xform = np.identity(4)
                return

            #where is the origin of the object with respect to the target
            origin_to_target = np.identity(4)
            #try reading from the database
            if self.objectName != "":
                origin_to_target = self.store.get(key=self.objectName + "/origin_to_target")
                if origin_to_target is None:
                    origin_to_target = np.identity(4)

            pose_of_obj = camera_xform.dot(self.cameraXform).dot(np.linalg.inv(origin_to_target))
            #update the gui
            for row in range(4):
                for col in range(4):
                    self.robotTextBoxes[row][col].setText(str(pose_of_obj[row, col]))

            #update the gui
            for row in range(4):
                for col in range(4):
                    self.cameraTextBoxes[row][col].setText(str(self.cameraXform[row, col]))

            if self.objectName != "":
                # logger.debug("storing {}".format(self.objectName + "/pose"))
                self.store.put(key=self.objectName + "/pose", value=pose_of_obj)
            
            #tell the thread to continue
            self.thread.can_emit = True

        # #push out the point cloud
        if self.cameraName != "":
            self.store.put(key="camera/" + self.cameraName + "/point_cloud", value=point_cloud)
            self.store.put(key="camera/" + self.cameraName + "aligned_image", value=aligned_image)
            self.store.put(key="camera/" + self.cameraName + "timestamp", value=time.time())

    def save_calibration(self):
        if self.calib_cameraCheckBox.isChecked():
            #get the camera serial number
            sn = self.thread.serialNum

            #get the robot matrix
            cameraMat = np.identity(4)
            for row in range(4):
                for col in range(4):
                    num = float(self.cameraTextBoxes[row][col].text())
                    cameraMat[row, col] = num

            #save it
            np.save(sn + "_xform", cameraMat)

        else:
            #save the object name
            objectmat = np.identity(4)
            for row in range(4):
                for col in range(4):
                    num = float(self.robotTextBoxes[row][col].text())
                    objectmat[row, col] = num

            np.save(self.objectName + "_xform", objectmat)

class VideoThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(np.ndarray, np.ndarray, np.ndarray)
    def __init__(self):
        QtCore.QThread.__init__(self)
        self.keepRunning = True
        self.serialNum = ''
        self.can_emit = True

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

            imageAllignedColor = cam.get_frame_data_u8(rs.stream_color_aligned_to_depth)
            if imageAllignedColor.size == 1:
                #something went wrong
                logger.error("Could not capture the depth alinged image. Size of requested stream did not match")
                imageAllignedColor = None
            else:
                width = cam.get_stream_width(rs.stream_color_aligned_to_depth)
                height = cam.get_stream_height(rs.stream_color_aligned_to_depth)    
                imageAllignedColor = np.reshape(imageAllignedColor, (height, width, 3) )

            points = cam.get_frame_data_f32(rs.stream_points)
            if points.size == 1:
                #something went wrong
                logger.error("Could not capture the point cloud. Size of requested stream did not match")
                points = None
            else:
                width = cam.get_stream_width(rs.stream_points)
                height = cam.get_stream_height(rs.stream_points)    
                points = np.reshape(points, (height, width, 3) )

            # logger.info(1/(time.time()-t))
            # t = time.time()
            if c%20 == 1 and self.can_emit:
                self.signal.emit(imageFullColor, imageAllignedColor, points)
                self.can_emit = False
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

