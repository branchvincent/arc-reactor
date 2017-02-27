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
        self.cameraIntrinsics = {}              #camera intrinsics. used to deproject points
        self.cameraDepthScale = {}              #depth scaling for cameras
        self.cameraExtrinsicsD2C = {}           #depth to color image extrinsics
        self.depthCamera = depthCamera.DepthCameras()
        self.depthCamera.connect()
        self.objectRecognizer = dl.ObjectRecognizer()

        # self.db_client = PensiveClient(host='http://10.10.1.102:8888')
        # self.store = self.db_client.default()
        # register_numpy()

        self.cameraCoeffs = {}
        #transforms from robot base to camera
        self.cameraXforms = {}
        self.load_camera_xforms(xformsFile)
        
        #get the camera intrinsics for all cameras
        self.get_camera_intrinsics()

        #get the camera extrinsics from depth to color for segmentation
        self.get_camera_extrinsics()

        self.object_names = ['ashland_decorative_fillers', 'band_aid_paper_tape', 'bathery_sponge', 'black_gloves', 'burts_baby_bees', 'clorox_toilet_brush', 'cloth_duct_tape', 'colgate_toothbrush', 'crayola_crayons', 'dr_teals_epsom_salt', 'elmers_glue_sticks', 'expo_eraser', 'greener_clean_sponges', 'hanes_cushion_crew_socks', 'ice_cube_tray', 'irish_spring_soap', 'kleenex', 'lol_joke_book', 'measuring_spoons', 'pink_scissors', 'pink_tablecloth', 'poland_springs_water', 'reynolds_pie_pans', 'reynolds_wrap', 'robots_dvd', 'robots_everywhere_book', 'rolodex_pencil_cup', 'ruled_index_cards', 'speed_stick', 'spritz_balloons', 'tennis_balls', 'ticonderoga_pencil', 'tomcat_mouse_trap', 'two_lb_hand_weight', 'wash_cloth', 'white_three_ring_binder', 'wide_ruled_notebook', 'windex', 'wine_glass']

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
            if picSnTuple != (None, None):
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
            cameramat, cameracoeff = self.depthCamera.get_camera_coefs(cam_num, rs.stream_color)
            if cameramat is None:
                logger.warning("Unable to get camera coefficients for camera {}. Setting to zeros...".format(sn))
                cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
                cameracoeff = np.zeros((5))
                self.cameraCoeffs[sn] = (cameramat, cameracoeff)
            self.cameraCoeffs[sn] = (cameramat, cameracoeff)

            cameraIntrinsics = self.depthCamera.get_camera_intrinsics(cam_num, rs.stream_depth)
            if cameraIntrinsics is None:
                logger.warning("Unable to get camera intrinsics for camera {}".format(sn))
            self.cameraIntrinsics[sn] = cameraIntrinsics

            cameraScale = self.depthCamera.get_camera_depthscale(cam_num)
            if cameraScale is None:
                logger.warning("Unable to get camera depth scale for {}. Setting to 1!".format(sn))
                cameraScale = 1
            self.cameraDepthScale[sn] = cameraScale

    def get_camera_extrinsics(self):
        list_of_serial_nums = self.depthCamera.get_online_cams()
        for cam_num, sn in enumerate(list_of_serial_nums):
            cameraEx = self.depthCamera.get_extrinsics(cam_num, rs.stream_depth, rs.stream_color)
            if cameraEx is None:
                logger.warning("Unable to get camera extrinsics for {}. Setting to None".format(sn))
            self.cameraExtrinsicsD2C[sn] = cameraEx

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
            color_images, depth_images = segmentation.depthSegmentation(depthImage, colorImage, self.cameraExtrinsicsD2C[serialNum])
            
            #for all sub images in the depth image guess what it is
            for num, im in enumerate(color_images):
                _, best_guess = self.objectRecognizer.guessObject(im)
                best_guess = best_guess[0]
                logger.info("Found object {}".format(self.object_names[best_guess]))
                #where is this object?
                #get list of indices of points that are non zero in the depth image
                indices = np.array(depth_images[num].nonzero()).astype('float32')
                indices = np.transpose(indices)
                pts_in_space = []
                #transform those to points in space
                for pt in indices:
                    p = rs.float2()
                    p.y = float(pt[0])
                    p.x = float(pt[1])
                    depthVal = self.cameraDepthScale[serialNum] * depth_images[num][int(p.y), int(p.x)]
                    point3d = self.cameraIntrinsics[serialNum].deproject(p, depthVal)
                    #TODO get this in real world coordinates based off of the camera xform
                    #cameraXform * point3d? inv(cameraXform) * point3d?
                    pts_in_space.append([point3d.x, point3d.y, point3d.z])
                #guess object centroid by using mean
                centroid = np.array(pts_in_space).mean(0)
                logger.info("Object's {} point cloud centroid is {}, {}, {}".format(self.object_names[best_guess], centroid[0], centroid[1], centroid[2]))
                #TODO send this info to the database server
                # key = best_guess + "/centroid/"
                # self.store.put(key=key,value=centroid)
