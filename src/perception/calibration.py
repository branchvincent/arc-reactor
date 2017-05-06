from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
import cv2
import math
import numpy as np
import os
import logging
from scipy import optimize
from multiprocessing import Lock
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
        self.ctx = rs.context()
        
        #what cameras are connected?
        self.connected_cams = self.ctx.get_device_count()
        self.cam_serial_nums = []
        for i in range(self.connected_cams):
            cam = self.ctx.get_device(i)
            #return the serial num of the camera too
            sn = cam.get_info(rs.camera_info_serial_number)
            self.cam_serial_nums.append(sn)
        
        #fill in the combo box with the cameras attached
        self.camera_selector_combo.addItems(self.cam_serial_nums)
        self.camera_selector_combo.currentIndexChanged.connect(self.change_camera)
        self.change_camera(0)

        self.cameraXform = np.identity(4)

        self.objectName = ""
        self.cameraName = ""

        self.last_x_poses = []              #stores the last 10 poses of the camera charuco
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
        self.horzTop.addItem(self.gridTop)
        
        self.camera_selector_combo = QtWidgets.QComboBox(self)
        self.horzTop.addWidget(self.camera_selector_combo)

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


    def change_camera(self, cam_num):
        logger.info("Camera changed to {}".format(self.cam_serial_nums[cam_num]))
        self.thread = VideoThread(self.ctx, cam_num)
        self.thread.signal.connect(self.updateView)
        cameramat, cameracoeff = self.thread.getIntrinsics()
        self.extrinsics = self.thread.getExtrinsics()
        if cameramat is None:
            logger.warning("Unable to get camera coefficients for camera. Setting to zeros...")
            cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
            cameracoeff = np.zeros((5))

        self.camera_matrix = cameramat
        self.camera_coeff = cameracoeff
        self.last_x_poses = []
        self.pose_counter = 0
    
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

    def updateView(self, image, aligned_image, point_cloud, cam_num):

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

            if len(self.last_x_poses) >= 10:
                #replace at the counter
                self.last_x_poses[self.pose_counter] = xform
                self.pose_counter = (self.pose_counter + 1) % 10
            else:
                self.last_x_poses.append(xform)
            #draw what opencv is tracking
            #flip r and b channels to draw
            image = image[:,:,::-1].copy()
            cv2.aruco.drawAxis(image, self.camera_matrix, self.camera_coeff, pose[1], pose[2], 0.1)
            image = image[:,:,::-1].copy()
            #flip back

            #average the poses last X poses
            avg_pose = np.zeros((4,4))
            for pose in self.last_x_poses:
                avg_pose = avg_pose + pose
            avg_pose /= len(self.last_x_poses)
            self.cameraXform = self.extrinsics.dot(avg_pose)
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
            ee2aruco = self.store.get(key='robot/target_xform')

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
                origin_to_target = self.store.get(key=self.objectName + "/target_xform")
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
            self.store.put(key="camera/" + self.cameraName + "/aligned_image", value=aligned_image)
            self.store.put(key="camera/" + self.cameraName + "/timestamp", value=time.time())

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


class RobotCamCalibration(QtWidgets.QWidget):
    def __init__(self):
        super(RobotCamCalibration, self).__init__()
        self.initUI()
        self.ctx = rs.context()
        
        #what cameras are connected?
        self.connected_cams = self.ctx.get_device_count()
        self.cam_serial_nums = []
        for i in range(self.connected_cams):
            cam = self.ctx.get_device(i)
            #return the serial num of the camera too
            sn = cam.get_info(rs.camera_info_serial_number)
            self.cam_serial_nums.append(sn)
        
        #fill in the combo box with the cameras attached
        self.camera_selector_combo.addItems(self.cam_serial_nums)
        self.camera_selector_combo.currentIndexChanged.connect(self.change_camera)
        self.change_camera(0)

        self.cameraXform = np.identity(4)
        self.cameraName = 'ee_cam'
        # self.db_client = PensiveClient(host='http://10.10.1.60:8888')
        # self.store = self.db_client.default()
        # register_numpy()

        self.last_images = []                      #last camera image
        self.last_img_cnt = 0                       #index of last image
        self.robot_xforms = []
        self.cam_to_target_xforms = []

    def initUI(self):

        #make the layout vertical
        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)

        #add places to enter/show the 4x4 matrix of the current postion of the robot
        self.horzMid = QtWidgets.QHBoxLayout()
        self.gridCamera = QtWidgets.QGridLayout()
        self.horzMid.addStretch(1)
        self.horzMid.addItem(self.gridCamera)
        self.horzMid.addStretch(1)
        self.offset_guesses = []
        names = ['Rx', 'Ry', 'Rz', 'X','Y','Z']
        for row in range(3):
            self.offset_guesses.append([])
            for col in range(8):
                if col % 2 == 0:
                    #label
                    l = QtWidgets.QLabel(self)
                    if col in [0,4]:
                        l.setText(names[row])
                    elif col in [2,6]:
                        l.setText(names[row+3])
                    self.gridCamera.addWidget(l,row,col)
                    l.show()
                else:
                    #lineedit
                    tbox = QtWidgets.QLineEdit(self)
                    self.offset_guesses[row].append(tbox)
                    self.gridCamera.addWidget(self.offset_guesses[row][col//2], row, col)

        for row in range(3):
            for col in range(4):
                self.offset_guesses[row][col].setText('0')


        label = QtWidgets.QLabel("Base to target guess",self)
        self.gridCamera.addWidget(label, 5, 1)
        label.show()
        label = QtWidgets.QLabel("EE to camera guess",self)
        self.gridCamera.addWidget(label, 5, 5)
        label.show()
        
        #add a push button at the top
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzTop.addStretch(1)
        self.store_pair_button = QtWidgets.QPushButton("Store calibration pair", self)
        self.calibrate_button = QtWidgets.QPushButton("Calibrate camera", self)
        self.start_camera_button = QtWidgets.QPushButton("Start camera", self)


        self.horzTop.addWidget(self.store_pair_button)
        self.horzTop.addSpacing(5)
        self.horzTop.addWidget(self.calibrate_button)
        self.horzTop.addSpacing(5)
        self.horzTop.addWidget(self.start_camera_button)
        self.horzTop.addSpacing(5)
        self.camera_selector_combo = QtWidgets.QComboBox(self)
        self.horzTop.addWidget(self.camera_selector_combo)

        self.horzTop.addStretch(1)
        self.calibrate_button.clicked.connect(self.calibrate_camera)
        self.store_pair_button.clicked.connect(self.store_calib_pair)
        self.start_camera_button.clicked.connect(self.start_stop_camera)

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
        self.store_pair_button.show()
        self.start_camera_button.show()


    def change_camera(self, cam_num):
        logger.info("Camera changed to {}".format(self.cam_serial_nums[cam_num]))
        self.thread = VideoThread(self.ctx, cam_num)
        self.thread.signal.connect(self.updateView)
        cameramat, cameracoeff = self.thread.getIntrinsics()
        self.extrinsics = self.thread.getExtrinsics()
        if cameramat is None:
            logger.warning("Unable to get camera coefficients for camera. Setting to zeros...")
            cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
            cameracoeff = np.zeros((5))

        self.camera_matrix = cameramat
        self.camera_coeff = cameracoeff
    
    def calibrate_camera(self):
        logger.info("Optimizing to find end effector to camera xform")
        global camera2targetpts
        global base2eepts

        base2eepts = self.robot_xforms
        camera2targetpts = self.cam_to_target_xforms

        if len(base2eepts) == 0:
            logger.warning("Tried to calibrate with zero points")
            return

        ee2camera_guess = np.eye(4)
        base2target_guess = np.eye(4)

        x = float(self.offset_guesses[0][1].text())
        y = float(self.offset_guesses[1][1].text())
        z = float(self.offset_guesses[2][1].text())
        #yaw is z pitch is y roll is x
        roll = float(self.offset_guesses[0][0].text())
        pitch = float(self.offset_guesses[1][0].text())
        yaw = float(self.offset_guesses[2][0].text())

        ee2camera_guess = transformMat(x,y,z, yaw, pitch, roll)


        x = float(self.offset_guesses[0][3].text())
        y = float(self.offset_guesses[1][3].text())
        z = float(self.offset_guesses[2][3].text())
        #yaw is z pitch is y roll is x
        roll = float(self.offset_guesses[0][2].text())
        pitch = float(self.offset_guesses[1][2].text())
        yaw = float(self.offset_guesses[2][2].text())
        base2target_guess = transformMat(x,y,z, yaw, pitch, roll)

        logger.info("EE to camera guess {}".format(ee2camera_guess))
        logger.info("Base to target guess {}".format(base2target_guess))

        
        guess = (ee2camera_guess, base2target_guess)
        res = optimize.minimize(function_to_min, guess, method='BFGS', tol=0.0001)
        print("Total RMS {}. {}".format(res.fun, res.message))

        #TODO make this correct
        self.store.put('/camera/ee_cam/pose', res.x)

    def store_calib_pair(self):
        logger.info("Storing calibration point...")
        #set the dictionary
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

        #need to measure out a target
        board = cv2.aruco.GridBoard_create(4, 6, .021, 0.003, dictionary)

        #look through the last x images and average the pose for images where the camera saw the target
        last_x_poses = []
        for image in self.last_images:
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
                last_x_poses.append(xform)
                logger.info("Found charuco!")
            else:
                logger.warning("Could not see Charuco in image. Skipping...")
        #average the poses last X poses
        avg_pose = np.zeros((4,4))
        for pose in last_x_poses:
            avg_pose = avg_pose + pose
        avg_pose /= len(last_x_poses)
        camera_to_target = self.extrinsics.dot(avg_pose)
        logger.info("Camera to target was\n {}".format(camera_to_target))
       
        #get the robot pose
        base2ee = self.store.get(key='robot/tcp_pose')

        logger.info('Robot pose was {}'.format(base2ee))
        if not base2ee is None:
            self.robot_xforms.append(base2ee)
            self.cam_to_target_xforms.append(camera_to_target)
            logger.info("Have {} calibration points".format(len(self.robot_xforms)))
        else:
            logger.warning("Robot pose was None, not storing")

    def start_stop_camera(self):
        if self.thread.isRunning():
            logger.info("Stopping camera")
            self.thread.keepRunning = False
        else:
            logger.info("Starting camera")
            self.thread.keepRunning = True
            self.thread.start()

    def updateView(self, image, aligned_image, point_cloud, cam_num):
        #keep the last 10 images
        if len(self.last_images) < 10:
            #append
            self.last_images.append(image)
        else:
            #replace at the counter
            self.last_images[self.last_img_cnt] = image
            self.last_img_cnt = (self.last_img_cnt + 1) % 10

        pix_img = QtGui.QImage(image, image.shape[1], image.shape[0], image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(pix_img)
        self.camera_display.setPixmap(pix)
        #tell the thread to continue
        self.thread.can_emit = True

    #2 numpy 4x4 xform matrix of the guess of the transform from the end effector to the camera
def function_to_min(tuple_of_xforms):
    ee2camera = tuple_of_xforms[0]
    base2target = tuple_of_xforms[1]
    sse = 0
    for i in range(len(base2eepts)):
        robot_xform = base2eepts[i]
        camera2target = camera2targetpts[i]

        guess = np.linalg.inv(base2target).dot(robot_xform).dot(ee2camera)
        xyz_guess = np.zeros((4,1))
        xyz_guess[0] = guess[0, 3]
        xyz_guess[1] = guess[1, 3]
        xyz_guess[2] = guess[2, 3]
        xyz_guess[3] = guess[3, 3]

        sse += (xyz_guess[0] - camera2target[0,3])**2 + (xyz_guess[1] - camera2target[1,3])**2 + (xyz_guess[2] - camera2target[2,3])**2
    
    return math.sqrt(sse/len(base2eepts))

def transformMat(x, y, z, yaw, pitch, roll):
	newpt = np.identity(4)
	yaw = yaw * math.pi / 180
	pitch = pitch * math.pi / 180
	roll = roll * math.pi / 180
	newpt[0, 0] = math.cos(yaw)*math.cos(pitch)
	newpt[0, 1] = math.cos(yaw)*math.sin(pitch)*math.sin(roll) - math.sin(yaw)*math.cos(roll)
	newpt[0, 2] = math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll)
	newpt[1, 0] = math.sin(yaw)*math.cos(pitch)
	newpt[1, 1] = math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll)
	newpt[1, 2] = math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll)
	newpt[2, 0] = -math.sin(pitch)
	newpt[2, 1] = math.cos(pitch)*math.sin(roll)
	newpt[2, 2] = math.cos(pitch)*math.cos(roll)
	newpt[3, 3] = 1
	newpt[0, 3] = x
	newpt[1, 3] = y
	newpt[2, 3] = z
	return newpt

class VideoThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(np.ndarray, np.ndarray, np.ndarray, int)
    def __init__(self, context, cam_num, update_rate=20):
        QtCore.QThread.__init__(self)
        self.keepRunning = True
        self.serialNum = ''
        self.can_emit = True
        self.context = context
        self.cam_num = cam_num
        self.update_rate = update_rate
        self.mutex = Lock()
        self.laseron = True
    def getIntrinsics(self):
        if self.context.get_device_count() == 0:
            logger.warn("No cameras attached")
            mat = np.zeros((3,3))
            coeffs = np.zeros((5))
            return (mat, coeffs)

        cam = self.context.get_device(self.cam_num)
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

    def getExtrinsics(self):
        if self.context.get_device_count() == 0:
            logger.warn("No cameras attached")
            mat = np.eye(4)
            return mat

        cam = self.context.get_device(self.cam_num)
        try:
            self.serialNum = cam.get_info(rs.camera_info_serial_number)
            extrinsics = cam.get_extrinsics(rs.stream_color, rs.stream_depth)
        except:
            logger.exception("Unable to enable stream and get intrinsics")
            return (None, None)

        mat = np.eye(4)
        mat[:3, :3] = np.array([rs.floatp_getitem(extrinsics.rotation, i) for i in range(9)]).reshape((3, 3))
        mat[:3, 3] = [rs.floatp_getitem(extrinsics.translation, i) for i in range(3)]

        return mat

    def run(self):
        if self.context.get_device_count() == 0:
            logger.warn("No cameras attached")
            return
        cam = self.context.get_device(self.cam_num)
        try:
            #enable the stream
            for s in [rs.stream_color, rs.stream_depth, rs.stream_infrared]:
                cam.enable_stream(s, rs.preset_best_quality)
        except:
            logger.exception("Could not enable the stream")
            return
        c = 0
        cam.start()
        if not self.laseron:
            cam.set_option(rs.option_f200_laser_power, 0)
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
            self.mutex.acquire()
            if c%self.update_rate == 1 and self.can_emit:
                self.signal.emit(imageFullColor, imageAllignedColor, points, self.cam_num)
                self.can_emit = False

            self.mutex.release()

            c = c+1
        cam.stop()

def main():

    if len(sys.argv) < 2:
        print("Usage 'calibration.py [endeffector/regular]'")
        return

    if sys.argv[1] == 'endeffector':
        #calibrate the endeffector cam
        app = QtWidgets.QApplication(sys.argv)
        #set up the window
        rc = RobotCamCalibration()
        rc.show()
        sys.exit(app.exec_())
    elif sys.argv[1] == 'regular':
        app = QtWidgets.QApplication(sys.argv)
        #set up the window
        rc = RealsenseCalibration()
        rc.show()
        sys.exit(app.exec_())
    else:
        print("unrecognized option {}".format(sys.argv[1]))


if __name__ == '__main__':
    main()

