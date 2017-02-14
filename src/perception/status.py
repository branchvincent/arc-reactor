import numpy as np
import math
import sys
sys.path.append('../hardware/SR300/')
sys.path.append('..')
import depthCamera
import realsense as rs
import cv2
from pensive.client import PensiveClient
from pensive.coders import register_numpy
import logging
logger = logging.getLogger(__name__)

class CameraStatus:

    def __init__(self):
        self.connectedCameras = {}
        self.cameraColorImages = {}
        self.cameraFullColorImages = {}
        self.cameraDepthImages = {}
        self.cameraPointClouds = {}
        self.depthCamera = depthCamera.DepthCameras()
        self.depthCamera.connect()
        # self.db_client = PensiveClient(host='http://10.10.1.102:8888')
        # self.store = self.db_client.default()
        # register_numpy()

        self.cameraIntrinsics = {}
        #transforms from aruco to camera
        self.cameraXforms = {}
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.board = cv2.aruco.CharucoBoard_create(8,11,.0172, 0.0125, self.dictionary)

        #get the camera intrinsics for all cameras
        self.get_camera_intrinsics()

    def poll(self):
        #see what cameras are connected
        list_of_serial_nums = self.depthCamera.get_online_cams()
        for cam_num, sn in enumerate(list_of_serial_nums):
            #update entries in the dictionary that are in the list
            self.connectedCameras[sn] = True

            #get the recent pictures for the connected cameraStatus
            picSnTuple = self.depthCamera.acquire_image(cam_num)
            if not picSnTuple is None:
                images, serialNum = picSnTuple
                self.cameraColorImages[serialNum] = images[1]
                self.cameraDepthImages[serialNum] = images[4]
                self.cameraFullColorImages[serialNum] = images[0]
                self.cameraPointClouds[serialNum] = images[5]

                # key = 'camera/' + serialNum + "/full_color"
                # self.store.put(key=key,value=images[0])
                
                # key = 'camera/' + serialNum + "/aligned_color"
                # self.store.put(key=key,value=images[1])

                # key = 'camera/' + serialNum + "/aligned_depth"
                # self.store.put(key=key,value=images[4])

                # key = 'camera/' + serialNum + "/point_cloud"
                # self.store.put(key=key,value=images[5])
            
                #get the camera transform
                gray = cv2.cvtColor(images[0], cv2.COLOR_BGR2GRAY)
                res = cv2.aruco.detectMarkers(gray,self.dictionary)
                if len(res[0])>0:
                    cameramat, cameracoeff = self.cameraIntrinsics[sn]
                    pose = cv2.aruco.estimatePoseBoard(res[0], res[1], self.board, cameramat, cameracoeff)
                    rotMat = cv2.Rodrigues(pose[1]) #returns rotation vector and translation vector
                    rotMat = rotMat[0]              #returns rotation matrix and jacobian
                    xform = np.zeros((4,4))
                    xform[0:3, 0:3] = rotMat
                    xform[0:3, 3] = pose[2].flatten()
                    xform[3, 3] = 1
                    self.cameraXforms[sn] = xform
                else:
                    logger.warning("Unable to get the camera transform for {}. Camera cannot see Charuco.".format(sn))
                    xform = np.zeros((4,4))
                    xform[0,0] = 1
                    xform[1,1] = 1
                    xform[2,2] = 1
                    xform[3,3] = 1
                    self.cameraXforms[sn] = xform
        #loop though the entire dictionary and see what camers are not in
        #the list. mark those false
        for key, value in self.connectedCameras.items():
            if not key in list_of_serial_nums:
                self.connectedCameras[key] = False

    # create an array that has xyz points with RGB colors of the world based on
    # what each camera currently sees   
    # returns a tuple of numpy arrays. one for the vertices and one for the color 
    def create_current_world_view(self):

        total_pts = []
        total_color = []

        #loop though all the cameras that are currently online
        for key, value in self.connectedCameras.items():
            
            #get the point cloud
            pc = self.cameraPointClouds[key]
            
            #get the color image
            color = self.cameraColorImages[key]

            #remove any points and colors that have a z value of zero
            nonzero = pc[:,:,2] != 0
            pc = pc[nonzero] #(numpoints x 3 array)
            color = color[nonzero]

            #transform the points to the real world here
            xformPC = []
            for pts in pc:
                invXform = np.linalg.inv(self.cameraXforms[key])
                fourArray = np.append(pts, 1)
                fourArray = invXform.dot(fourArray)
                xformPC.append(fourArray[0:3])


            total_pts.append(np.array(xformPC))
            total_color.append(color)
        if len(total_pts) == 0:
            #no cameras are connected return empty lists
            return (np.zeros((1,1)), np.zeros((1,1)))
        
        res_pc = total_pts[0]
        for i in range(1,len(total_pts)):
            res_pc = np.concatenate((res_pc, total_pts[i]))

        res_c = total_color[0]
        for i in range(1,len(total_color)):
            res_c = np.concatenate((res_c, total_color[i]))

        return (res_pc, res_c)


    def get_camera_intrinsics(self):
        list_of_serial_nums = self.depthCamera.get_online_cams()
        for cam_num, sn in enumerate(list_of_serial_nums):
            cameramat, cameracoeff = self.depthCamera.get_camera_intrinsics(cam_num, rs.stream_color)
            if cameramat is None:
                logger.warning("Unable to get camera intrinsics for camera {}. Setting to zeros...".format(sn))
                cameramat = np.array([[0,0,0],[0,0,0],[0,0,1]])
                cameracoeff = np.zeros((5))
                self.cameraIntrinsics[sn] = (cameramat, cameracoeff)

            self.cameraIntrinsics[sn] = (cameramat, cameracoeff)
