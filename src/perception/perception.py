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
import scipy.misc
from pensive.client import PensiveClient
from pensive.coders import register_numpy
import logging
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

        #create a deepLearning object ObjectRecognizer
        self.objectRecognizer = dl.ObjectRecognizer('vgg_finetuned_ARC2017_1.pkl',39)

        #try to connect to the database
        self.store = None
        # try:
        #     self.db_client = PensiveClient(host='http://10.10.1.102:8888')
        #     self.store = self.db_client.default()
        #     register_numpy()
        # except:
        #     logger.warn("Could not connect to the database on {}".format('10.10.1.102:8888'))
        #     self.store = None
        

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

        #list of object names of the items. TODO read this in from database or file
        self.object_names = ['ashland_decorative_fillers', 'band_aid_paper_tape', 'bathery_sponge', 'black_gloves', 'burts_baby_bees', 'clorox_toilet_brush', 'cloth_duct_tape', 'colgate_toothbrush', 'crayola_crayons', 'dr_teals_epsom_salt', 'elmers_glue_sticks', 'expo_eraser', 'greener_clean_sponges', 'hanes_cushion_crew_socks', 'ice_cube_tray', 'irish_spring_soap', 'kleenex', 'lol_joke_book', 'measuring_spoons', 'pink_scissors', 'pink_tablecloth', 'poland_springs_water', 'reynolds_pie_pans', 'reynolds_wrap', 'robots_dvd', 'robots_everywhere_book', 'rolodex_pencil_cup', 'ruled_index_cards', 'speed_stick', 'spritz_balloons', 'tennis_balls', 'ticonderoga_pencil', 'tomcat_mouse_trap', 'two_lb_hand_weight', 'wash_cloth', 'white_three_ring_binder', 'wide_ruled_notebook', 'windex', 'wine_glass']

    def acquire_images(self):
        '''
        Checks what cameras are connected and gets new images from the cameras
        Pushes results to the database
        '''
        for sn in self.camera_variables.keys():
           
           #check if the camera is connected
           if self.camera_variables[sn].connected:

            #get the recent pictures for the connected camera
            picSnTuple = self.depthCamera.acquire_image(self.camera_variables[sn].number)
            if picSnTuple != (None, None):
                images, serialNum = picSnTuple
            
                self.camera_variables[sn].aligned_color_image = images[1]
                self.camera_variables[sn].depth_image = images[3]
                self.camera_variables[sn].full_color_image = images[0]
                self.camera_variables[sn].point_cloud = images[5]

                if not self.store is None:
                    #write stuff out to the database
                    key = 'camera/' + serialNum + "/full_color"
                    self.store.put(key=key,value=images[0])
                    
                    key = 'camera/' + serialNum + "/aligned_color"
                    self.store.put(key=key,value=self.camera_variables[sn].aligned_color_image)

                    key = 'camera/' + serialNum + "/aligned_depth"
                    self.store.put(key=key,value=self.camera_variables[sn].depth_image)

                    key = 'camera/' + serialNum + "/point_cloud"
                    self.store.put(key=key,value=self.camera_variables[sn].point_cloud)
            else:
                #mark that camera as disconnected
                self.camera_variables[sn].connected = False

    def create_world_view(self):
        '''
        create an array that has xyz points with RGB colors of the world based on
        what each camera currently sees   
        returns a tuple of numpy arrays. one for the vertices and one for the color 
        Returns a tuple of points and colors if cameras are connected
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
                    #TODO do we need to multiply the z by the camera depth scale here to get real world coordinates?
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

    def segment_objects(self):
        '''
        Performs segmentation on images from all cameras
        '''
        #loop though all the cameras
        for sn in self.camera_variables.keys():
           
           #check if the camera is connected
            if self.camera_variables[sn].connected:
                d_image = self.camera_variables[sn].depth_image
                c_image = self.camera_variables[sn].full_color_image
                extrin = self.camera_variables[sn].depth2color_extrinsics
                #segment the image
                color_images, depth_images, boxed_color = segmentation.depthSegmentation(d_image, c_image, extrin)
                self.camera_variables[sn].segmented_image = boxed_color
                self.camera_variables[sn].dl_images = color_images
                self.camera_variables[sn].depth_object_images = depth_images

    def infer_objects(self):
        '''
        Runs deep learning inference on all segmented images from all cameras and stores the results
        '''
        for sn in self.camera_variables.keys():
           
           #check if the camera is connected
            if self.camera_variables[sn].connected:

                #reset the guess and confidences
                self.camera_variables[sn].item_guesses = ['']*len(self.camera_variables[sn].dl_images)
                self.camera_variables[sn].item_confidences = [0]*len(self.camera_variables[sn].dl_images)

                for num, im in enumerate(self.camera_variables[sn].dl_images):
                    
                    #convert the image from BGR to RGB
                    rgb_image = np.array(scipy.misc.toimage(im))
                    #infer
                    confidences, _ = self.objectRecognizer.guessObject(rgb_image)

                    #restrict the guesses to the items this camera can see
                    if self.camera_variables[sn].visible_items != []:
                        vis_item_conf = []
                        vis_item_ind = []
                        for item in self.camera_variables[sn].visible_items:
                            #find the index of this item
                            ind = self.object_names.index(item)
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
        for sn in self.camera_variables.keys():
            try:
                xform = np.load(str(sn) + "_xform.npy")
            except:
                logger.warning("No camera transform provided. Setting to identity matrix")
                xform = np.zeros((4,4))
                xform[0,0] = 1
                xform[1,1] = 1
                xform[2,2] = 1
                xform[3,3] = 1
                
            self.camera_variables[sn].world_xform = xform


class CameraVariables:

    def __init__(self):
        #is the camera connected?
        self.connected = False
        self.number = -1                            #what number do we pass to depthcameras to get this camera?

        #images
        self.aligned_color_image = None                 #color image aligned to depth image. black where there is no depth reading
        self.full_color_image = None                    #full color image 640x480 RGB
        self.segmented_image = None                     #full color image with boxes drawn around what we think are objects
        self.depth_image = None                         #depth image. 0 means no reading/invalid is a float
        self.point_cloud = None                         #xyz point cloud

        #information about the objects this camera sees
        self.dl_images = None                           #list of color images to pass to deep learning 
        self.depth_object_images = None                 #list of depth images we can use to find this object in space
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
    t = Perception()
    t.acquire_images()
    t.segment_objects()
    t.infer_objects()
    t.combine_objects()
    t.compute_xform_of_obejcts()