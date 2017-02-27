import numpy as np
import math
import sys
sys.path.append('../hardware/SR300/')
sys.path.append('..')
import depthCamera
import realsense as rs
import segmentation
import deepLearning as dl
import cv2
# from pensive.client import PensiveClient
# from pensive.coders import register_numpy
import logging
logger = logging.getLogger(__name__)

class CameraStatus:
    '''
    Class that contains the status for all cameras in the system
    '''
    def __init__(self, xformsFile = ''):
        self.connectedCameras = {}              #dictionary says whether or not camera is connected
        self.cameraColorImages = {}             #depth aligned color images
        self.cameraFullColorImages = {}         #full color images
        self.cameraDepthImages = {}             
        self.cameraPointClouds = {}             #point clouds in xyz
        self.depthCamera = depthCamera.DepthCameras()
        self.depthCamera.connect()
        self.objectRecognizer = dl.ObjectRecognizer()
        # self.db_client = PensiveClient(host='http://10.10.1.102:8888')
        # self.store = self.db_client.default()
        # register_numpy()

        self.cameraIntrinsics = {}
        #transforms from robot base to camera
        self.cameraXforms = {}
        self.load_camera_xforms(xformsFile)
        
        #get the camera intrinsics for all cameras
        self.get_camera_intrinsics()

    def poll(self):
        '''
        Checks what cameras are connected and gets new images from the cameras
        Pushes results to the database
        '''
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
            
        #loop though the entire dictionary and see what camers are not in
        #the list. mark those false
        for key, value in self.connectedCameras.items():
            if not key in list_of_serial_nums:
                self.connectedCameras[key] = False


    def create_current_world_view(self):
        '''
        create an array that has xyz points with RGB colors of the world based on
        what each camera currently sees   
        returns a tuple of numpy arrays. one for the vertices and one for the color 
        Returns a tuple of points and colors if cameras are connected
        '''
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
        '''
        Gets and sets the camera intrinsics for online cameras
        '''
        list_of_serial_nums = self.depthCamera.get_online_cams()
        for cam_num, sn in enumerate(list_of_serial_nums):
            cameramat, cameracoeff = self.depthCamera.get_camera_intrinsics(cam_num, rs.stream_color)
            if cameramat is None:
                logger.warning("Unable to get camera intrinsics for camera {}. Setting to zeros...".format(sn))
                cameramat = np.array([[0,0,0],[0,0,0],[0,0,1]])
                cameracoeff = np.zeros((5))
                self.cameraIntrinsics[sn] = (cameramat, cameracoeff)

            self.cameraIntrinsics[sn] = (cameramat, cameracoeff)


    def load_camera_xforms(self, filename=''):
        '''
        Loads in a dictionary of camera transforms. The key is the serial number and the value
        is the transform from the robot base to the camera. If no file name is given sets all transforms
        to be the identity matrix
        '''
        if filename == "":
            list_of_serial_nums = self.depthCamera.get_online_cams()
            for sn in list_of_serial_nums:
                logger.warning("No camera transform provided. Setting to identity matrix")
                xform = np.zeros((4,4))
                xform[0,0] = 1
                xform[1,1] = 1
                xform[2,2] = 1
                xform[3,3] = 1
                self.cameraXforms[sn] = xform


    def find_objects(self):
        '''
        Tries to locate objects in images
        '''
        list_of_serial_nums = self.depthCamera.get_online_cams()
        for cam_num, serialNum in enumerate(list_of_serial_nums):
            #get the depth image
            depthImage = self.cameraDepthImages[serialNum]
            colorImage = self.cameraFullColorImages[serialNum]
            #segment the image
            list_of_images = segmentation.depthSegmentation(depthImage, colorImage)
            
            #for all sub images in the depth image guess what it is
            for im in list_of_images:
                _, best_guess = self.objectRecognizer.guessObject(im)
                logger.info("Found object {}".format(best_guess))