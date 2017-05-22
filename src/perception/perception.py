import numpy as np
import math
import sys
sys.path.append('../hardware/SR300/')
sys.path.append('..')
from import depthCamera import DepthCameras
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

'''
Updates the camera parameters (intrinsics, extrinsics, depth scale, etc)
for all cameras in list_of_serial_nums
'''
def update_camera_parameters(list_of_serial_nums):

    if len(list_of_serial_nums) == 0 or list_of_serial_nums is None:
        logger.warning("No serial numbers were passed. Need at least one. Not updating values")
        return

    #try to connect to the database
    store = None
    try:
        db_client = PensiveClient(host='http://10.10.1.60:8888')
        store = db_client.default()
        register_numpy()
    except:
        logger.warn("Could not connect to the database on {}. Not updating params".format('10.10.1.60:8888'))
        return

    #make a dict of sn to names
    sn_to_cam_names = {}
    cam_names_to_sn = store.get('system/cameras')
    if cam_names_to_sn is None:
        logger.warn("Could not get the dictionary of camera names to sn. Not updating params")
        return
    for key, value in cam_names_to_sn.items():
        sn_to_cam_names[value] = key

    #try to connect to the depth cameras
    depthCamera = DepthCameras()
    if depthCamera.connect():
        #get a dictionary that gives sn->camera num
        sn_to_camNum = depthCamera.get_camera_serial_numbers()
    else:
        logger.critical("Could not connect to the cameras...EVERYBODY PANIC")
        return

    for sn in list_of_serial_nums:
        cameramat, cameracoeff = depthCamera.get_camera_coefs(sn_to_camNum[sn], rs.stream_color)
        if cameramat is None:
            logger.warning("Unable to get color coefficients/matrix for camera {}. Setting to zeros...".format(sn))
            cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
            cameracoeff = np.zeros((5))
        #set the color coefficients and matrix by writing to the database
        store.put('/camera/' + sn_to_cam_names[sn] + '/color/matrix', cameramat)
        store.put('/camera/' + sn_to_cam_names[sn] + '/color/coeff', cameracoeff)

        cameramat, cameracoeff = depthCamera.get_camera_coefs(sn_to_camNum[sn].number, rs.stream_depth)
        if cameramat is None:
            logger.warning("Unable to get depth coefficients/matrix for camera {}. Setting to zeros...".format(sn))
            cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
            cameracoeff = np.zeros((5))
        #set the depth coefficients and matrix by writing to the database
        store.put('/camera/' + sn_to_cam_names[sn] + '/depth/matrix', cameramat)
        store.put('/camera/' + sn_to_cam_names[sn] + '/depth/coeff', cameracoeff)

        cameraIntrinsics = depthCamera.get_camera_intrinsics(sn_to_camNum[sn].number, rs.stream_depth)
        if cameraIntrinsics is None:
            logger.warning("Unable to get depth intrinsics for camera {}".format(sn))
        else:
            #set the depth intrinsics
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/ppx', cameraIntrinsics.ppx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/ppy', cameraIntrinsics.ppy)
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/fx', cameraIntrinsics.fx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/fy', cameraIntrinsics.fy)

        cameraIntrinsics = depthCamera.get_camera_intrinsics(sn_to_camNum[sn].number, rs.stream_color)
        if cameraIntrinsics is None:
            logger.warning("Unable to get color intrinsics for camera {}".format(sn))
        else:
            #set the color intrinsics
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/ppx', cameraIntrinsics.ppx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/ppy', cameraIntrinsics.ppy)
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/fx', cameraIntrinsics.fx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/fy', cameraIntrinsics.fy)

        cameraScale = depthCamera.get_camera_depthscale(sn_to_camNum[sn].number)
        if cameraScale is None:
            logger.warning("Unable to get camera depth scale for {}. Setting to 1!".format(sn))
            cameraScale = 1
        store.put('/camera/' + sn_to_cam_names[sn] + '/depth/scale', cameraScale)

        cameraEx = depthCamera.get_extrinsics(sn_to_camNum[sn], rs.stream_depth, rs.stream_color)
        if cameraEx is None:
            logger.warning("Unable to get depth to color extrinsics for {}. Setting to None".format(sn))
        store.put('/camera/' + sn_to_cam_names[sn] + '/depth/colorExtrinsics', cameraEx)

        cameraEx = depthCamera.get_extrinsics(sn_to_camNum[sn], rs.stream_color, rs.stream_depth)
        if cameraEx is None:
            logger.warning("Unable to get color to depth extrinsics for {}. Setting to None".format(sn))
        store.put('/camera/' + sn_to_cam_names[sn] + '/color/depthExtrinsics', cameraEx)

'''
Update camera images only.  Takes in one or more camera serial numbers as a list. 
Takes in one or more urls as output location for the images.
Point clouds, aligned images, color images, and intrinsics/extrinsics etc are updated and pushed to the database
'''
def acquire_images(list_of_urls, list_of_serial_nums):
    #check to make sure the length of the urls == len of serial nums
    if len(list_of_urls) != len(list_of_serial_nums):
        logger.warning("Length mismatch of url list and serial number list. Not acquiring")
        return
    elif len(list_of_urls) == 0 or list_of_urls is None:
        logger.warning("No URLs were passed. Need at least one. Not acquiring")
        return
    elif len(list_of_serial_nums) == 0 or list_of_serial_nums is None:
        logger.warning("No serial numbers were passed. Need at least one. Not acquiring")
        return

    #try to connect to the database
    store = None
    try:
        db_client = PensiveClient(host='http://10.10.1.60:8888')
        store = db_client.default()
        register_numpy()
    except:
        logger.warn("Could not connect to the database on {}. Not acquiring".format('10.10.1.60:8888'))
        return

    #try to connect to the depth cameras
    depthCamera = DepthCameras()
    if depthCamera.connect():
        #get a dictionary that gives sn->camera num
        sn_to_camNum = self.depthCamera.get_camera_serial_numbers()
    else:
        logger.critical("Could not connect to the cameras...EVERYBODY PANIC")
        return

    for sn in list_of_serial_nums:
        #get pictures from camera
        try:
            picSnTuple = depthCamera.acquire_image(sn_to_camNum[sn])
        except:
            logger.error('DID YOU PASS IN AN INVALID SERIAL NUMBER?!?! DAMMIT!')
            continue
        if picSnTuple != (None, None):
            images, serialNum = picSnTuple
        
            aligned_color_image = images[1]
            depth_image = images[4]
            full_color_image = images[0]
            point_cloud = images[5]

            if not store is None:
                #write stuff out to the database
                key = list_of_urls[i] + "/full_color"
                store.put(key=key,value=full_color_image)
                
                key = list_of_urls[i] + "/aligned_color"
                store.put(key=key,value=aligned_color_image)

                key = list_of_urls[i] + "/aligned_depth"
                store.put(key=key,value=depth_image)

                key = list_of_urls[i] + "/point_cloud"
                store.put(key=key,value=point_cloud)

                key = list_of_urls[i] + "/time_stamp"
                store.put(key, time.time())

        else:
            logger.warning("Camera {} did not return any images. Could be disconnected".format(sn))
            if not store is None:
                cam_names_to_sn = store.get('system/cameras')
                if not cam_names_to_sn is None:
                    for key, value in cam_names_to_sn.items():
                        if value == sn:
                            key = '/camera/' + value + "/connected"
                            store.put(key, False)
        
'''
Performs segmentation on images from camera serial numbers in the list.
Crops the image based on the bounds in list of bounds and the pose in 
list of poses
'''
def segment_images(list_of_urls, list_of_bounds_urls, list_of_world_xforms_urls):

    if len(list_of_urls) != len(list_of_bounds_urls) or len(list_of_bounds_urls) != len(list_of_world_xforms_urls):
        logger.warning("Length mismatch of url list and bounds list or bounds list and pose list. Not segmenting")
        return
    elif len(list_of_urls) == 0 or list_of_urls is None:
        logger.warning("No URLs were passed. Need at least one. Not segmenting")
        return
    elif len(list_of_bounds_urls) == 0 or list_of_bounds_urls is None:
        logger.warning("No bounds were passed. Need at least one. Not segmenting")
        return

    #try to connect to the database
    store = None
    try:
        db_client = PensiveClient(host='http://10.10.1.60:8888')
        store = db_client.default()
        register_numpy()
    except:
        logger.warn("Could not connect to the database on {}. Not segmenting".format('10.10.1.60:8888'))
        return

    #loop though all the urls
    for i,url in enumerate(list_of_urls):
        #get the name of the camera that took the image
        #look between the first and last slash of the url for the camera name
        last_slash = url.rfind('/',0, len(url))
        second_last_slash = url.rfind('/',0, last_slash)
        cam_name = url[second_last_slash+1:last_slash]

        d_image = store.get(url + "aligned_depth")
        c_image = store.get(url + "full_color")

        if d_image is None or c_image is None:
            logger.warn("Depth or color image was none. Can't segment")
            return

        seg_params = segmentation.GraphSegmentationParams()
        bounds = store.get(list_of_bounds_urls[i])
        if bounds is None:
            logger.warn("Bounds were None. Can't segment image")
            return

        ref_world_xform = store.get(list_of_world_xforms_urls[i])
        if ref_world_xform is None:
            logger.warn("Reference world transform was None. Can't segment image")
            return

        #crop the image for desired bin 
        intrins_fx = store.get('/camera/' + cam_name + "/depth/intrinsics/fx")
        intrins_fy = store.get('/camera/' + cam_name + "/depth/intrinsics/fy")
        intrins_ppx = store.get('/camera/' + cam_name + "/depth/intrinsics/ppx")
        intrins_ppy = store.get('/camera/' + cam_name + "/depth/intrinsics/ppy")
        if intrins_fx is None or intrins_fy is None or intrins_ppx is None or intrins_ppy is None:
            logger.warning("Could not get the intrinsicds for the camera {}. Not segmenting".format(cam_name))
            return

        scale = store.get('/camera/' + cam_name + "/depth/scale")
        coeffs = store.get('/camera/' + cam_name + "/depth/coeff")
        #get camera world location
        cam_pose_world = store.get(url + "pose")
        if scale is None or coeffs is None or cam_pose_world is None:
            logger.warning("Could not get the depth scale or coeffs for the camera {}. Not segmenting".format(cam_name))
            return
        
        #get the 3d point by deprojecting pixel to get camera local
        depth_in_3d_cam_local = np.zeros((480,640,3))
        [xs, ys] = np.meshgrid(range(depth_in_3d_cam_local.shape[1]), range(depth_in_3d_cam_local.shape[0]))
        xs = (xs - intrins_ppx)/intrins_fx
        ys = (ys - intrins_ppy)/intrins_fy
        r2 = xs*xs + ys*ys
        f = 1 + coeffs[0]*r2 + coeffs[1]*r2*r2 + coeffs[4]*r2*r2*r2
        ux = xs*f + 2*coeffs[2]*xs*ys + coeffs[3]*(r2 + 2*xs*xs)
        uy = ys*f + 2*coeffs[3]*xs*ys + coeffs[2]*(r2 + 2*ys*ys)
        
        depth_in_3d_cam_local[:, :, 0] = ux
        depth_in_3d_cam_local[:, :, 1] = uy
        depth_in_3d_cam_local[:, :, 2] = depth_img*scale
        
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

        #optimized params
        seg_params.k = 3762.97
        seg_params.minSize = 840.067
        seg_params.c_rad = 7
        seg_params.sigma = 1.8066
        seg_params.sp_rad = 3

        seg_params.L_weight = 75.5078
        seg_params.A_weight = 50.6409984
        seg_params.B_weight = 57.456
        seg_params.depth_weight = 17.838
        seg_params.mask = mask
        
        #segment the image
        ret = segmentation.graphSegmentation(d_image, c_image, seg_params)

        #write out results to database
        store.put(url + "labeled_image", ret['labeled_image'])
        store.put(url + "DL_images", ret['DL_images'])
        store.put(url + "segments", ret['pixel_locations'])

    
import argparse
if __name__ == "__main__":
    #command line parsing of arguments
    parser = argparse.ArgumentParser(description='Perform tasks for perception.')
    parser.add_argument('-f', type=str, help="Fucntion to call", choices=['update_cams', 'acquire_images', 'segment_images'], required=True)
    parser.add_argument('-sn', type=str,nargs='+', help="List of serial numbers")
    parser.add_argument('-u', type=str,nargs='+', help="List of URLS")
    parser.add_argument('-b', type=str,nargs='+', help="List of bounds URLs")
    parser.add_argument('-x', type=str,nargs='+', help="List of xforms URLs")
    args = parser.parse_args()
    
    if args.f == "update_cams":
        update_camera_parameters(args.sn)
    elif args.f == 'acquire_images':
        acquire_images(args.u, args.sn)
    elif args.f == "segment_images":
        segment_images(args.u, args.b, args.x)