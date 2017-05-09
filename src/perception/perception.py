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

        else:
            logger.error("Could not connect to the cameras...EVERYBODY PANIC")
        #create a deepLearning object ObjectRecognizer
        self.objectRecognizer = dl.ObjectRecognizer('vgg_norandombackground.pkl',40)

        #list of object names of the items. 
        names = []
        #load in the items.json file
        with open('../../db/items.json') as data_file:    
            jsonnames = json.load(data_file)
        for key,value in jsonnames.items():
            names.append(key)
        names.sort()
        self.object_names = names

        #dicitonary that provides a list of items, and their index for each location
        self.items_in_location = {}
        self.items_in_location['binA'] = []
        self.items_in_location['binB'] = []
        self.items_in_location['binC'] = []
        self.items_in_location['amnesty_tote'] = []
        self.items_in_location['stow_tote'] = []

        #query the database to find which objects each camera can see
        self.update_items_cameras_see()

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
            mask = None
            if not list_of_bins is None:
                if i < len(list_of_bins):
                    #top directory is either shelf, box, or tote
                    if 'box' in list_of_bins[i]:
                        topdir = '/box/'
                        refname = list_of_bins[i]
                    elif 'bin' in list_of_bins[i]:
                        topdir = '/shelf/bin/'
                        refname = list_of_bins[i]
                    elif 'tote' in list_of_bins[i]:
                        topdir = '/tote/'
                        if 'stow' in list_of_bins[i]:
                            refname = 'stow'
                        elif 'amnesty' in list_of_bins[i]:
                            refname = 'amnesty'
                    else:
                        logger.warn("Bad location passed to segmentation {}".format(list_of_bins[i]))
                        topdir = "noooooooooothing"
                    bin_bounds = self.store.get(topdir + refname + '/bounds')
                    if 'bin' in list_of_bins[i]:
                        #its a bin doesnt have a pose. Use the shelf's
                        ref_world_xform = self.store.get("/shelf/pose")
                    else:
                        ref_world_xform = self.store.get(topdir + refname + "/pose")

                    if not bin_bounds is None:
                        depth_img = self.camera_variables[sn].depth_image
                        intrins = self.camera_variables[sn].depth_intrinsics
                        scale = self.camera_variables[sn].depth_scale
                        coeffs = self.camera_variables[sn].depth_coefficients
                        
                        #get the 3d point by deprojecting pixel to get camera local
                        depth_in_3d_cam_local = np.zeros((480,640,3))
                        [xs, ys] = np.meshgrid(range(depth_in_3d_cam_local.shape[1]), range(depth_in_3d_cam_local.shape[0]))
                        xs = (xs - intrins.ppx)/intrins.fx
                        ys = (ys - intrins.ppy)/intrins.fy
                        r2 = xs*xs + ys*ys
                        f = 1 + coeffs[0]*r2 + coeffs[1]*r2*r2 + coeffs[4]*r2*r2*r2
                        ux = xs*f + 2*coeffs[2]*xs*ys + coeffs[3]*(r2 + 2*xs*xs)
                        uy = ys*f + 2*coeffs[3]*xs*ys + coeffs[2]*(r2 + 2*ys*ys)
                        
                        depth_in_3d_cam_local[:, :, 0] = ux
                        depth_in_3d_cam_local[:, :, 1] = uy
                        depth_in_3d_cam_local[:, :, 2] = depth_img*scale
                        
                        #get camera world location
                        cam_pose_world = self.camera_variables[sn].world_xform
                        
                        #get the transform from world to reflocal
                        ref_world_xform = np.linalg.inv(ref_world_xform)

                        #compose these transforms
                        cam_local_to_ref_local = np.array(ref_world_xform.dot(cam_pose_world))

                        # apply transformation to get to ref local coordinates
                        depth_in_ref_local = (depth_in_3d_cam_local.reshape((-1, 3)).dot(cam_local_to_ref_local[:3, :3].T) + cam_local_to_ref_local[:3, 3].T).reshape(depth_in_3d_cam_local.shape)
                        
                    
                        #get min/max x, y, z bounds
                        minx = min(bin_bounds[0][0],bin_bounds[1][0])
                        miny = min(bin_bounds[0][1],bin_bounds[1][1])
                        minz = min(bin_bounds[0][2],bin_bounds[1][2])

                        maxx = max(bin_bounds[0][0],bin_bounds[1][0])
                        maxy = max(bin_bounds[0][1],bin_bounds[1][1])
                        maxz = max(bin_bounds[0][2],bin_bounds[1][2])

                        #check to see if point is within bounds by masking
                        maskx = np.logical_and(depth_in_ref_local[:,:,0] > minx, depth_in_ref_local[:,:,0] < maxx)
                        masky = np.logical_and(depth_in_ref_local[:,:,1] > miny, depth_in_ref_local[:,:,1] < maxy)
                        maskz = np.logical_and(depth_in_ref_local[:,:,2] > minz, depth_in_ref_local[:,:,2] < maxz)
                        mask = np.logical_and(np.logical_and(maskx, masky),maskz)

                    else:
                        logger.warning("No bounds found for reference {}. Database was empty".format(list_of_bins[i]))
            #TODO location aware segmentation parameters
            seg_params.k = 350
            seg_params.medianFilterW = 5
            seg_params.minSize = 600
            seg_params.c_rad = 33
            seg_params.sigma = 1.5
            seg_params.sp_rad = 3
            seg_params.mask = mask
            
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


            #guess all objects at once
            im = np.zeros((224,224,3,len(self.camera_variables[sn].dl_images)))
            for num,img in enumerate(self.camera_variables[sn].dl_images):
                im[:,:,:,num] = img
            #infer
            confidences = self.objectRecognizer.guessObject(im)
            #store all confidences for each image
            for list_of_conf in confidences:
                self.camera_variables[sn].image_confidences.append(list_of_conf)

    def segment_plus_detect(self, list_of_serial_nums=None, list_of_bins=None):
        self.segment_objects(list_of_serial_nums, list_of_bins)
        self.infer_objects(list_of_serial_nums, list_of_bins)
        self.combine_objects(list_of_serial_nums, list_of_bins)

    def combine_objects(self, list_of_serial_nums, list_of_bins):
        '''
        Method to combine point clouds of like objects seen from different cameras
        '''

        #need list of bins and list of serial nums to be the same length. need duplicates

        if len(list_of_serial_nums) != len(list_of_bins):
            logger.error("Function combine objects requires a location for each camera")
            return

        #are any two cameras looking at the same location? TODO later

        #only worry about single camera and single location now. pick the "best" match
        #get non duplicate list of bins
        non_duplicate_locations = list(set(list_of_bins))

        #find all images that are in location
        for location in non_duplicate_locations:
            #find all cameras that see this location
            cameras = []
            for c in range(len(list_of_bins)):
                if list_of_bins[c] == location:
                    cameras.append(list_of_serial_nums[c])

            #get all the confidences and size of segments for this location
            location_segment_sizes = []
            location_conf = []
            camera_idx_tuple = []
            for sn in cameras:
                for n, seg in enumerate(self.camera_variables[sn].segments):
                    location_segment_sizes.append(len(seg))
                    location_conf.append(self.camera_variables[sn].image_confidences[n])
                    camera_idx_tuple.append((sn,n))

            #loop through all the items and find the segment, image, confidence that best fits this item
            used_segments = []
            for item_name, item_idx in self.items_in_location[location]:
            
                #Which segment size weighted by confidence is the largest for this item?
                conf_weighted_by_seg_size = []
                for i, seg_size in enumerate(location_segment_sizes):
                    if not i in used_segments:
                        conf_weighted_by_seg_size.append(seg_size*location_conf[i][item_idx])
                    else:
                        #used index gets negative weight
                        conf_weighted_by_seg_size.append(-10000)
                conf_weighted_by_seg_size = np.array(conf_weighted_by_seg_size)
                
                if len(conf_weighted_by_seg_size) == 0:
                    #all segments have been accounted for quit
                    logger.warning("Some objects have not been assigned segments")
                    break

                #what index has the highest weighted conf*segsize for this item?
                idxmax = conf_weighted_by_seg_size.argmax()
                used_segments.append(idxmax)

                #log this
                cam = camera_idx_tuple[idxmax][0]
                idx_of_img = camera_idx_tuple[idxmax][1]
                object_confidence = location_conf[idxmax][item_idx]
                logger.info("Camera {} found object {} with confidence of {}".format(cam, item_name, object_confidence))

               
                
                #transform point cloud to local coordinates
                pc_indices = self.camera_variables[sn].segments[idx_of_img]
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

                #write this guess out (point cloud, indices, confidence)
                self.store.put('/item/' + item_name + "/pose", pc_pose)
                #update the point cloud
                self.store.put('/item/' + item_name + "/point_cloud", np.array(pc)- mean_of_pc)
                #update the color
                self.store.put('/item/' + item_name + '/point_cloud_color', np.array(pc_color))
                #send out the indices of the segmentation
                self.store.put('/item/' + item_name + "/mask", self.camera_variables[sn].segments[idx_of_img])
                #send confidence and point cloud to the database
                self.store.put('/item/'+ item_name + '/id_confidence',object_confidence)
                #update timestamp
                self.store.put('/item/' + item_name + "/timestamp",time.time())

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
                    self.items_in_location['binA'].append((value['name'],self.object_names.index(value['name'])))
                elif location[3] == 'B':
                    #shelf1 and 0
                    sn = self.store.get('/system/cameras/shelf1')
                    self.camera_variables[sn].visible_items.append((value['name'], 'binB'))

                    sn = self.store.get('/system/cameras/shelf0')
                    self.camera_variables[sn].visible_items.append((value['name'], 'binB'))

                    self.items_in_location['binB'].append((value['name'],self.object_names.index(value['name'])))
                elif location[3] == 'C':
                    #shelf0
                    sn = self.store.get('/system/cameras/shelf0')
                    self.camera_variables[sn].visible_items.append((value['name'], 'binC'))
                    self.items_in_location['binC'].append((value['name'],self.object_names.index(value['name'])))
                else:
                    logger.warning("Invalid bin ({}) for item {}.".format(location, value['name']))
            elif 'tote' in location:
                #tote cam
                if 'amnesty' in location:
                    sn = self.store.get('/system/cameras/amnesty')
                    self.camera_variables[sn].visible_items.append((value['name'], 'amnesty_tote'))
                    self.items_in_location['amnesty_tote'].append((value['name'],self.object_names.index(value['name'])))
                elif 'stow' in location:
                    sn = self.store.get('/system/cameras/stow')
                    self.camera_variables[sn].visible_items.append((value['name'], 'stow_tote'))
                    self.items_in_location['stow_tote'].append((value['name'],self.object_names.index(value['name'])))
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
        self.image_confidences = []                   #list of lists. for each image in dl_images store the output of all confidences for that image
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
    
