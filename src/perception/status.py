import numpy as np
import sys
sys.path.append('../hardware/SR300/')
import depthCamera
import realsense as rs

class CameraStatus:

    def __init__(self):
        self.connectedCameras = {}
        self.cameraColorImages = {}
        self.cameraFullColorImages = {}
        self.cameraDepthImages = {}
        self.cameraPointClouds = {}
        self.depthCamera = depthCamera.DepthCameras()
        self.depthCamera.connect()
        #TODO need to have transform for all cameras that map realsense pixel coordinates to 
        #real world coordinates
        #self.cameraXforms = {}

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
            pc = pc[nonzero]
            color = color[nonzero]

            #TODO transform the points to the real world here

            total_pts.append(pc)
            total_color.append(color)

        
        res_pc = np.array(total_pts)
        res_c = np.array(total_color)

        return (res_pc, res_c)