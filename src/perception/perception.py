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
import os
import scipy.misc
from pensive.client import PensiveClient
from pensive.coders import register_numpy
import logging
import json
import time
logger = logging.getLogger(__name__)

class Perception:
    '''
    Class that contains the status for all cameras in the system
    '''
    def __init__(self):

        #dictionary from serial number to camera variable object
        self.camera_variables = {}                              
        #get the depth camera object                        
        self.depthCamera = depthCamera.DepthCameras()

        #dictionary that converts serial numbers to names for cameras
        self.serial_nums_2_cams = {}

        #create a deepLearning object ObjectRecognizer
        self.objectRecognizer = dl.ObjectRecognizer('vgg_finetuned_ARC2017.pkl',40)

        #try to connect to the database
        self.store = None
        try:
            self.db_client = PensiveClient(host='http://10.10.1.60:8888')
            self.store = self.db_client.default()
            register_numpy()
        except:
            logger.warn("Could not connect to the database on {}".format('10.10.1.60:8888'))
            self.store = None
        

        #try to connect to the depth camera
        if self.depthCamera.connect():
            #find how many cameras there are and create camera_variables for each
            list_of_serial_nums = self.depthCamera.get_online_cams()
            for num, sn in enumerate(list_of_serial_nums):
                self.camera_variables[sn] = CameraVariables()
                self.camera_variables[sn].connected = True
                self.camera_variables[sn].number = num

            #get the camera intrinsics for all cameras
            self.get_camera_intrinsics()

            #get the camera extrinsics for all cameras
            self.get_camera_extrinsics()

            #load in the world location for all cameras
            self.load_camera_xforms()

        #query the database to find which objects each camera can see
        self.update_items_cameras_see()

        #list of object names of the items. 
        names = []
        #load in the items.json file
        with open('../../db/items.json') as data_file:    
            jsonnames = json.load(data_file)
        for key,value in jsonnames.items():
            names.append(key)
        names.sort()
        self.object_names = names

    def acquire_images(self, list_of_serial_nums=None):
        '''
        Update camera images only.  Takes in one or more camera serial numbers as a list. If list_of_serial_nums is
        empty all cameras are queried.
        Point clouds, aligned images, and color images are updated and pushed to the database
        '''
        if list_of_serial_nums is None:
            list_of_serial_nums = self.camera_variables.keys()

        for sn in list_of_serial_nums:
           
           #check if the camera is connected
           if self.camera_variables[sn].connected:

            #get the recent pictures for the connected camera
            picSnTuple = self.depthCamera.acquire_image(self.camera_variables[sn].number)
            if picSnTuple != (None, None):
                images, serialNum = picSnTuple
            
                self.camera_variables[sn].aligned_color_image = images[1]
                self.camera_variables[sn].depth_image = images[4]
                self.camera_variables[sn].full_color_image = images[0]
                self.camera_variables[sn].point_cloud = images[5]

                if not self.store is None:
                    #write stuff out to the database
                    key = 'camera/' + self.serial_nums_2_cams[serialNum] + "/full_color"
                    self.store.put(key=key,value=images[0])
                    
                    key = 'camera/' + self.serial_nums_2_cams[serialNum] + "/aligned_color"
                    self.store.put(key=key,value=self.camera_variables[sn].aligned_color_image)

                    key = 'camera/' + self.serial_nums_2_cams[serialNum] + "/aligned_depth"
                    self.store.put(key=key,value=self.camera_variables[sn].depth_image)

                    key = 'camera/' + self.serial_nums_2_cams[serialNum] + "/point_cloud"
                    self.store.put(key=key,value=self.camera_variables[sn].point_cloud)

                    key = '/camera/' + self.serial_nums_2_cams[serialNum] + "/time_stamp"
                    self.store.put(key, time.time())

            else:
                #mark that camera as disconnected
                self.camera_variables[sn].connected = False
        

    def segment_objects(self, list_of_serial_nums=None, list_of_bins=None):
        '''
        Performs segmentation on images from camera serial numbers in the list
        If a bin is specified for the camera, the image is cropped to only that bin
        '''

        if list_of_serial_nums is None:
            list_of_serial_nums = self.camera_variables.keys()

        #loop though all the cameras
        for i,sn in enumerate(list_of_serial_nums):

            d_image = self.camera_variables[sn].depth_image
            c_image = self.camera_variables[sn].full_color_image

            seg_params = segmentation.GraphSegmentationParams()
            #crop the image for desired bin 
            if not list_of_bins is None:
                if i < len(list_of_bins):
                    #top directory is either shelf, box, or tote
                    if 'box' in list_of_bins[i]:
                        topdir = '/box/'
                    elif 'bin' in list_of_bins[i]:
                        topdir = '/shelf/bin/'
                    elif 'tote' in list_of_bins[i]:
                        topdir = '/tote/'
                    else:
                        logger.warn("Bad location passed to segmentation {}".format(list_of_bins[i]))
                        topdir = "noooooooooothing"
                    bin_bounds = self.store.get(topdir + list_of_bins[i] + '/bounds')
                    pixel_bounds = []
                    if not bin_bounds is None:
                        #get camera world location
                        cam_pose_world = self.camera_variables[sn].world_xform

                        #get bin relative to shelf local location (two points that are at opposite corners of the bounding box)
                        bin_bounds_world = []

                        #get all 8 points
                        bin_bounds = sorted(bin_bounds, key=lambda k: k[0])
                        smallestX = bin_bounds[0][0]
                        largestX = bin_bounds[1][0]
                        bin_bounds = sorted(bin_bounds, key=lambda k: k[1])
                        smallestY = bin_bounds[0][1]
                        largestY = bin_bounds[1][1]
                        bin_bounds = sorted(bin_bounds, key=lambda k: k[2])
                        smallestZ = bin_bounds[0][2]
                        largestZ = bin_bounds[1][2]

                        bin_bounds_world.append([smallestX,smallestY,smallestZ])
                        bin_bounds_world.append([smallestX,smallestY,largestZ])
                        bin_bounds_world.append([smallestX,largestY,smallestZ])
                        bin_bounds_world.append([smallestX,largestY,largestZ])

                        bin_bounds_world.append([largestX,smallestY,smallestZ])
                        bin_bounds_world.append([largestX,smallestY,largestZ])
                        bin_bounds_world.append([largestX,largestY,smallestZ])
                        bin_bounds_world.append([largestX,largestY,largestZ])
                        
                        if 'bin' in list_of_bins[i]:
                            #its a bin doesnt have a pose. Use the shelf's
                            origin_2_ref = self.store.get("/shelf/pose")
                        else:
                            origin_2_ref = self.store.get(topdir + list_of_bins[i] + "/pose")
                        #convert all these points to world coordinates
                        for i in range(len(bin_bounds_world)):
                            pt = np.array(bin_bounds_world[i] + [1])
                            bin_bounds_world[i] = origin_2_ref.dot(pt)

                        #get the bin in camera local coordinates
                        bin_in_camera_local = []
                        for point in bin_bounds_world:
                            bin_in_camera_local.append(np.linalg.inv(cam_pose_world).dot(point.transpose()))

                        #get camera intrinsics to project 3d point on to color image
                        cam_intrins = self.camera_variables[sn].color_intrinsics

                        for point in bin_in_camera_local:
                            #project 3d point on to 2d point
                            pt = rs.float3()
                            pt.x = point.astype('float32').tolist()[0][0]
                            pt.y = point.astype('float32').tolist()[1][0]
                            pt.z = point.astype('float32').tolist()[2][0]
                            pixel_coord = cam_intrins.project(pt)
                            pixel_bounds.append([pixel_coord.x, pixel_coord.y])
                    
                    if len(pixel_bounds) > 0:
                        #find smallest x and smallest y
                        pixel_bounds = sorted(pixel_bounds, key=lambda k: k[0])
                        smallestX = pixel_bounds[0][0]
                        largestX = pixel_bounds[7][0]
                        pixel_bounds = sorted(pixel_bounds, key=lambda k: k[1])
                        smallestY = pixel_bounds[0][1]
                        largestY = pixel_bounds[7][1]

                        seg_params.topLeft = (smallestX, largestY)
                        seg_params.topRight = (largestX, largestY)
                        seg_params.botRight = (largestX, smallestY)


            #TODO location away segmentation parameters
            seg_params.k = 250
            seg_params.medianFilterW = 5
            seg_params.minSize = 600
            seg_params.c_rad = 33
            seg_params.sigma = 0.5
            seg_params.sp_rad = 3

            #segment the image
            ret = segmentation.graphSegmentation(d_image, c_image, seg_params)

            self.camera_variables[sn].segmented_image = ret['segmented_image']
            self.camera_variables[sn].box_image = ret['boxes_image']
            self.camera_variables[sn].dl_images = ret['DL_images']
            self.camera_variables[sn].sift_images = ret['small_images']
            self.camera_variables[sn].segments = ret['pixel_locations']

    def infer_objects(self, list_of_serial_nums=None, list_of_locations=None):
        '''
        Runs deep learning inference on all segmented images from cameras in the list of serial numbers
        '''
        if list_of_serial_nums is None:
            list_of_serial_nums = self.camera_variables.keys()
           
        for num, sn in enumerate(list_of_serial_nums):
            #reset the guess and confidences
            self.camera_variables[sn].item_guesses = ['']*len(self.camera_variables[sn].dl_images)
            self.camera_variables[sn].item_confidences = [0]*len(self.camera_variables[sn].dl_images)
            if not list_of_locations is None:
                if num < len(list_of_locations):
                    location = list_of_locations[num]
                else:
                    location = None
            else:
                location = None

            for num, im in enumerate(self.camera_variables[sn].dl_images):
                
                #infer
                confidences, _ = self.objectRecognizer.guessObject(im)

                #restrict the guesses to the items this camera can see
                if self.camera_variables[sn].visible_items != [] and not location is None:
                    vis_item_conf = []
                    vis_item_ind = []
                    for item, loc in self.camera_variables[sn].visible_items:
                        #find the index of this item
                        ind = self.object_names.index(item)
                        if loc == location: #only add this item if it is in the correct location
                            vis_item_ind.append(ind)
                            vis_item_conf.append(confidences[ind])
                    vis_item_conf = np.array(vis_item_conf)
                    #what is the maximum confidence value of the visible_items?
                    index_of_best_guess_in_subset = vis_item_conf.argmax(-1)
                    object_name = self.object_names[vis_item_ind[index_of_best_guess_in_subset]]
                    object_confidence = vis_item_conf[index_of_best_guess_in_subset]
                    self.camera_variables[sn].item_guesses[num] = object_name
                    self.camera_variables[sn].item_confidences[num] = object_confidence
                    logger.info("Camera {} found object {} with confidence of {}".format(sn, object_name, object_confidence))
                else:
                    index_of_best_guess = confidences.argmax(-1)
                    object_name = self.object_names[index_of_best_guess]
                    object_confidence = confidences[index_of_best_guess]
                    self.camera_variables[sn].item_guesses[num] = object_name
                    self.camera_variables[sn].item_confidences[num] = object_confidence
                    logger.info("Camera {} found object {} with confidence of {}".format(sn, object_name, object_confidence))

                #send confidence and point cloud to the database
                self.store.put('/item/'+ object_name + '/id_confidence',object_confidence)
                
                #transform point cloud to local coordinates
                pc_indices = self.camera_variables[sn].segments[num]
                pc = []
                pc_color = []
                if not location is None:
                    if 'bin' in location:
                        #its a bin doesnt have a pose. Use the shelf's
                        origin_2_ref = self.store.get("/shelf/pose")
                    elif 'box' in location:
                        origin_2_ref = self.store.get('/box/' + location + "/pose")
                    elif 'tote' in location:
                        if 'amnesty' in location:
                            origin_2_ref = self.store.get('/tote/amnesty/pose')
                        elif 'stow' in location:
                            origin_2_ref = self.store.get('/tote/stow/pose')
                    else:
                        origin_2_ref = np.eye(4)
                        logger.error('Unrecognized location {}. Pushing point cloud in world coordinates'.format(location))
                else:
                    origin_2_ref = np.eye(4)
                    logger.error('No location provided. Pushing point cloud in world coordinates'.format(location))



                for point in pc_indices:
                    point_in_camera_local = np.append(self.camera_variables[sn].point_cloud[point[0],point[1]], 1)
                    point_in_world = self.camera_variables[sn].world_xform.dot(np.array(point_in_camera_local))
                    #get the point in reference local coordinates
                    point_in_ref_local = np.linalg.inv(origin_2_ref).dot(point_in_world.transpose())
                    pc.append(np.squeeze(np.array(point_in_ref_local[0:3])))
                    pc_color.append(self.camera_variables[sn].aligned_color_image[point[0],point[1]])

                #TODO update if we ever get pose information
                mean_of_pc = np.array(pc).mean(axis=0)
                pc_pose = np.eye(4)
                pc_pose[:3,3] = mean_of_pc
                self.store.put('/item/' + object_name + "/pose", pc_pose)
                #update the point cloud
                self.store.put('/item/' + object_name + "/point_cloud", np.array(pc)- mean_of_pc)
                #update the color
                self.store.put('/item/' + object_name + '/point_cloud_color', np.array(pc_color))
                #update timestamp as well
                self.store.put('/item/' + object_name + "/timestamp",time.time())

    def segment_plus_detect(self, list_of_serial_nums=None, list_of_bins=None):
        self.segment_objects(list_of_serial_nums, list_of_bins)
        self.infer_objects(list_of_serial_nums, list_of_bins)

    def combine_objects(self):
        '''
        Method to combine point clouds of like objects seen from different cameras
        '''
        #TODO
        pass

    def compute_xform_of_objects(self):
        '''
        Find the transform in world coordinates of all of the objects seen by all cameras
        '''
        #TODO
        pass


    def create_world_view(self):
        '''
        create an array that has xyz points with RGB colors of the world based on
        what each camera currently sees   
        returns a tuple of numpy arrays. one for the vertices and one for the color 
        '''
        total_pts = []
        total_color = []

        #loop though all the cameras
        for sn in self.camera_variables.keys():
           
           #check if the camera is connected
            if self.camera_variables[sn].connected:
            
                #get the point cloud
                pc = self.camera_variables[sn].point_cloud

                #get the color image
                color = self.camera_variables[sn].aligned_color_image

                #remove any points and colors that have a z value of zero
                nonzero = pc[:,:,2] != 0
                pc = pc[nonzero] #(numpoints x 3 array)
                color = color[nonzero]

                #transform the points to the real world here
                xformPC = []
                for pts in pc:
                    #do we need to multiply the z by the camera depth scale here to get real world coordinates? No
                    fourArray = np.append(pts, 1)
                    fourArray = self.camera_variables[sn].world_xform.dot(fourArray)
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
        for sn in self.camera_variables.keys():
            cameramat, cameracoeff = self.depthCamera.get_camera_coefs(self.camera_variables[sn].number, rs.stream_color)
            if cameramat is None:
                logger.warning("Unable to get color coefficients/matrix for camera {}. Setting to zeros...".format(sn))
                cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
                cameracoeff = np.zeros((5))
            #set the color coefficients and matrix   
            self.camera_variables[sn].color_coefficients = cameracoeff
            self.camera_variables[sn].color_matrix = cameramat

            cameramat, cameracoeff = self.depthCamera.get_camera_coefs(self.camera_variables[sn].number, rs.stream_depth)
            if cameramat is None:
                logger.warning("Unable to get depth coefficients/matrix for camera {}. Setting to zeros...".format(sn))
                cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
                cameracoeff = np.zeros((5))
            #set the color coefficients and matrix   
            self.camera_variables[sn].depth_coefficients = cameracoeff
            self.camera_variables[sn].depth_matrix = cameramat

            cameraIntrinsics = self.depthCamera.get_camera_intrinsics(self.camera_variables[sn].number, rs.stream_depth)
            if cameraIntrinsics is None:
                logger.warning("Unable to get depth intrinsics for camera {}".format(sn))
            #set the depth intrinsics
            self.camera_variables[sn].depth_intrinsics = cameraIntrinsics

            cameraIntrinsics = self.depthCamera.get_camera_intrinsics(self.camera_variables[sn].number, rs.stream_color)
            if cameraIntrinsics is None:
                logger.warning("Unable to get depth intrinsics for camera {}".format(sn))
            #set the color intrinsics
            self.camera_variables[sn].color_intrinsics = cameraIntrinsics

            cameraScale = self.depthCamera.get_camera_depthscale(self.camera_variables[sn].number)
            if cameraScale is None:
                logger.warning("Unable to get camera depth scale for {}. Setting to 1!".format(sn))
                cameraScale = 1
            self.camera_variables[sn].depth_scale = cameraScale

    def get_camera_extrinsics(self):
        for sn in self.camera_variables.keys():
            cameraEx = self.depthCamera.get_extrinsics(self.camera_variables[sn].number, rs.stream_depth, rs.stream_color)
            if cameraEx is None:
                logger.warning("Unable to get depth to color extrinsics for {}. Setting to None".format(sn))
            self.camera_variables[sn].depth2color_extrinsics = cameraEx

            cameraEx = self.depthCamera.get_extrinsics(self.camera_variables[sn].number, rs.stream_color, rs.stream_depth)
            if cameraEx is None:
                logger.warning("Unable to get color to depth extrinsics for {}. Setting to None".format(sn))
            self.camera_variables[sn].color2depth_extrinsics = cameraEx

    def load_camera_xforms(self):
        '''
        Loads in a dictionary of camera transforms. The key is the serial number and the value
        is the transform from the robot base to the camera. If no file name is given sets all transforms
        to be the identity matrix
        '''

        #get the dictionary from the server of camera names to serial numbers
        cams_2_serial_nums = self.store.get('/system/cameras')
        
        for key, item in cams_2_serial_nums.items():
            self.serial_nums_2_cams[item] = key

        for sn in self.camera_variables.keys():
            xform = self.store.get('/camera/' + self.serial_nums_2_cams[sn] + '/pose')
            if xform is None:
                logger.warning("No camera transform provided. Setting to identity matrix")
                xform = np.zeros((4,4))
                xform[0,0] = 1
                xform[1,1] = 1
                xform[2,2] = 1
                xform[3,3] = 1
                
            self.camera_variables[sn].world_xform = xform


    def update_items_cameras_see(self):
        '''
        Goes through all items in the database and puts each item in the cameras visible_items list
        '''
        items = self.store.get('/item/')
        for key, value in items.items():
            #where is this item? binA/B/C, tote, or box
            location = value['location']
            #shelf0 can see C and B, shelf1 can see B and A
            if 'bin' in location:
                if location[3] == 'A':
                    #shelf1
                    sn = self.store.get('/system/cameras/shelf1')
                    self.camera_variables[sn].visible_items.append((value['name'], 'binA'))
                elif location[3] == 'B':
                    #shelf1 and 0
                    sn = self.store.get('/system/cameras/shelf1')
                    self.camera_variables[sn].visible_items.append((value['name'], 'binB'))

                    sn = self.store.get('/system/cameras/shelf0')
                    self.camera_variables[sn].visible_items.append((value['name'], 'binB'))
                elif location[3] == 'C':
                    #shelf0
                    sn = self.store.get('/system/cameras/shelf0')
                    self.camera_variables[sn].visible_items.append((value['name'], 'binC'))
                else:
                    logger.warning("Invalid bin ({}) for item {}.".format(location, value['name']))
            elif 'tote' in location:
                #tote cam
                if 'amnesty' in location:
                    sn = self.store.get('/system/cameras/amnesty')
                    self.camera_variables[sn].visible_items.append((value['name'], 'amnesty_tote'))
                elif 'stow' in location:
                    sn = self.store.get('/system/cameras/stow')
                    self.camera_variables[sn].visible_items.append((value['name'], 'stow_tote'))
            elif 'box' in location:
                logger.warning("Unimplemented")

class CameraVariables:

    def __init__(self):
        #is the camera connected?
        self.connected = False
        self.number = -1                            #what number do we pass to depthcameras to get this camera?

        #images
        self.aligned_color_image = None                 #color image aligned to depth image. black where there is no depth reading
        self.full_color_image = None                    #full color image 640x480 RGB
        self.box_image = None                           #full color image with boxes drawn around what we think are objects
        self.segmented_image = None                     #bw image that is output from graph cut
        self.depth_image = None                         #depth image. 0 means no reading/invalid is a float
        self.point_cloud = None                         #xyz point cloud

        #information about the objects this camera sees
        self.dl_images = None                           #list of color images to pass to deep learning 
        self.segments = None                            #list of segments used to get the point cloud of objects
        self.sift_images = None                         #list of images that we can use in the sift guesser
        self.item_guesses = None                        #list of the best guess as to what the object in corresponding index of dl_images is
        self.item_confidences = None                    #how confident the guess is
        self.visible_items = []                         #what items can this camera see? What item(s) are in the bin(s) this camera can see?
        
        #parameters
        self.world_xform = np.identity(4)               #where this camera is in the world
        self.depth_intrinsics = None                    #how to project points into 3d or deproject points from 3d to depth
        self.color_intrinsics = None                    #how to project points into 3d or deproject points from 3d to color
        self.color_matrix = None                        #camera matrix for color image
        self.color_coefficients = None                  #camera coefficients for color image
        self.depth_matrix = None                        #camera matrix for depth image
        self.depth_coefficients = None                  #camera coefficients for depth image
        self.color2depth_extrinsics = None              #how to map from color pixels to depth pixels
        self.depth2color_extrinsics = None              #how to map from depth pixels to color pixels
        self.depth_scale = 1                            #scale from depth image value to meters


if __name__ == "__main__":
    #command line parsing of arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == 'acquire':
            t = Perception()
            cameras = []
            for cmdarg in sys.argv[2:]:
                try:
                    cnum = int(cmdarg[0])
                    cameras.append(cmdarg)
                except:
                    pass
            if cameras == []:
                cameras = None
            t.acquire_images(cameras)
        elif sys.argv[1] == 'segment':
            t = Perception()
            cameras = []
            bins = []
            for cmdarg in sys.argv[2:]:
                try:
                    cnum = int(cmdarg[0])
                    cameras.append(cmdarg)
                except:
                    bins.append(cmdarg)
            if cameras == []:
                cameras = None
            if bins == []:
                bins = None
            t.acquire_images(cameras)
            t.segment_plus_detect(cameras, bins)
        else:
            print('Unrecognized option {}'.format(sys.argv[1]))
        
    else:
        print('Usage: [acquire/segment] [camera SN] [locations]')
    
