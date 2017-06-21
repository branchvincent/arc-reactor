import numpy as np
import math
import sys
from hardware.SR300.depthCamera import DepthCameras
import hardware.SR300.realsense as rs
from . import segmentation
import cv2
import os
import scipy.misc
from pensive.client import PensiveClient
from pensive.coders import register_numpy
import logging
import json
import time
logger = logging.getLogger(__name__)

from util.math_helpers import transform

def update_camera_parameters(list_of_serial_nums):
    '''
    Updates the camera parameters (intrinsics, extrinsics, depth scale, etc)
    for all cameras in list_of_serial_nums
    '''

    if list_of_serial_nums is None or len(list_of_serial_nums) == 0:
        raise RuntimeError("No serial numbers were passed. Need at least one. Not updating values")


    #try to connect to the database
    store = None
    try:
        db_client = PensiveClient()
        store = db_client.default()
        register_numpy()
    except:
        raise RuntimeError('Could not connect to the database. Not updating params')


    #make a dict of sn to names
    sn_to_cam_names = {}
    cam_names_to_sn = store.get('system/cameras')
    if cam_names_to_sn is None:
        raise RuntimeError("Could not get the dictionary of camera names to sn. Not updating params")

    for key, value in cam_names_to_sn.items():
        sn_to_cam_names[value] = key
    #try to connect to the depth cameras
    depthCamera = DepthCameras()
    if depthCamera.connect():
        #get a dictionary that gives sn->camera num
        sn_to_camNum = depthCamera.get_camera_serial_numbers()
    else:
        logger.critical("Could not connect to the cameras...EVERYBODY PANIC")
        raise RuntimeError("Could not connect to cameras")


    for sn in list_of_serial_nums:
        cameramat, cameracoeff = depthCamera.get_camera_coefs(sn_to_camNum[sn], rs.stream_color)
        if cameramat is None:
            logger.warning("Unable to get color coefficients/matrix for camera {}. Setting to zeros...".format(sn))
            cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
            cameracoeff = np.zeros((5))
        #set the color coefficients and matrix by writing to the database
        store.put('/camera/' + sn_to_cam_names[sn] + '/color/matrix', cameramat)
        store.put('/camera/' + sn_to_cam_names[sn] + '/color/coeff', cameracoeff)

        cameramat, cameracoeff = depthCamera.get_camera_coefs(sn_to_camNum[sn], rs.stream_depth)
        if cameramat is None:
            logger.warning("Unable to get depth coefficients/matrix for camera {}. Setting to zeros...".format(sn))
            cameramat = np.array([[1,0,0],[0,1,0],[0,0,1]])
            cameracoeff = np.zeros((5))
        #set the depth coefficients and matrix by writing to the database
        store.put('/camera/' + sn_to_cam_names[sn] + '/depth/matrix', cameramat)
        store.put('/camera/' + sn_to_cam_names[sn] + '/depth/coeff', cameracoeff)

        cameraIntrinsics = depthCamera.get_camera_intrinsics(sn_to_camNum[sn], rs.stream_depth)
        if cameraIntrinsics is None:
            logger.warning("Unable to get depth intrinsics for camera {}".format(sn))
        else:
            #set the depth intrinsics
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/ppx', cameraIntrinsics.ppx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/ppy', cameraIntrinsics.ppy)
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/fx', cameraIntrinsics.fx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/depth/intrinsics/fy', cameraIntrinsics.fy)

        cameraIntrinsics = depthCamera.get_camera_intrinsics(sn_to_camNum[sn], rs.stream_color)
        if cameraIntrinsics is None:
            logger.warning("Unable to get color intrinsics for camera {}".format(sn))
        else:
            #set the color intrinsics
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/ppx', cameraIntrinsics.ppx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/ppy', cameraIntrinsics.ppy)
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/fx', cameraIntrinsics.fx)
            store.put('/camera/' + sn_to_cam_names[sn] + '/color/intrinsics/fy', cameraIntrinsics.fy)

        cameraScale = depthCamera.get_camera_depthscale(sn_to_camNum[sn])
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


def acquire_images(list_of_urls, list_of_serial_nums):
    '''
    Update camera images only.  Takes in one or more camera serial numbers as a list.
    Takes in one or more urls as output location for the images.
    Point clouds, aligned images, color images, and intrinsics/extrinsics etc are updated and pushed to the database
    '''
    if list_of_urls is None or list_of_serial_nums is None:
        raise RuntimeError("Arguments cant be None")
    #check to make sure the length of the urls == len of serial nums
    elif len(list_of_urls) != len(list_of_serial_nums):
        raise RuntimeError("Length mismatch of url list and serial number list. Not acquiring")

    elif len(list_of_urls) == 0:
        raise RuntimeError("No URLs were passed. Need at least one. Not acquiring")

    elif len(list_of_serial_nums) == 0:
        raise RuntimeError("No serial numbers were passed. Need at least one. Not acquiring")


    #try to connect to the database
    store = None
    try:
        db_client = PensiveClient()
        store = db_client.default()
        register_numpy()
    except:
        raise RuntimeError('Could not connect to the database on {}. Not acquiring')


    #try to connect to the depth cameras
    depthCamera = DepthCameras()
    if depthCamera.connect():
        #get a dictionary that gives sn->camera num
        sn_to_camNum = depthCamera.get_camera_serial_numbers()
    else:
        logger.critical("Could not connect to the cameras...EVERYBODY PANIC")
        raise RuntimeError("Could not connect to cameras")


    for i,sn in enumerate(list_of_serial_nums):
        #get pictures from camera
        try:
            picSnTuple = depthCamera.acquire_image(sn_to_camNum[sn])
        except:
            logger.error('DID YOU PASS IN AN INVALID SERIAL NUMBER {}?!?! DAMMIT!'.format(sn))
            raise RuntimeError("Invalid serial number")
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


def segment_images(list_of_urls, list_of_bounds_urls, list_of_world_xforms_urls):
    '''
    Performs segmentation on images from camera serial numbers in the list.
    Crops the image based on the bounds in list of bounds and the pose in
    list of poses
    '''
    if list_of_urls is None or list_of_bounds_urls is None or list_of_world_xforms_urls is None:
        raise RuntimeError("Arguments cant be None")
    elif len(list_of_urls) != len(list_of_bounds_urls) or len(list_of_bounds_urls) != len(list_of_world_xforms_urls):
        raise RuntimeError("Length mismatch of url list and bounds list or bounds list and pose list. Not segmenting")
    elif len(list_of_urls) == 0:
        raise RuntimeError("No URLs were passed. Need at least one. Not segmenting")
    elif len(list_of_bounds_urls) == 0:
        raise RuntimeError("No bounds were passed. Need at least one. Not segmenting")

    #try to connect to the database
    store = None
    try:
        db_client = PensiveClient()
        store = db_client.default()
        register_numpy()
    except:
        raise RuntimeError('Could not connect to the database on {}. Not segmenting')


    #loop though all the urls
    for i,url in enumerate(list_of_urls):


        #get the name of the camera that took the image
        cam_name = store.get(url + "/camera")
        if cam_name is None:
            raise RuntimeError("Camrea name not present")
        #get the location
        location = store.get(url + "/location")
        if location is None:
            raise RuntimeError("No location provided to segmentation")

        d_image = store.get(url + "aligned_depth")
        c_image = store.get(url + "full_color")

        if d_image is None or c_image is None:
            raise RuntimeError("Depth or color image was none. Can't segment")


        seg_params = segmentation.GraphSegmentationParams()
        bin_bounds = store.get(list_of_bounds_urls[i])
        if bin_bounds is None:
            raise RuntimeError("Bounds were None. Can't segment image")


        ref_world_xform = store.get(list_of_world_xforms_urls[i])
        if ref_world_xform is None:
            raise RuntimeError("Reference world transform was None. Can't segment image")


        #crop the image for desired bin
        intrins_fx = store.get('/camera/' + cam_name + "/color/intrinsics/fx")
        intrins_fy = store.get('/camera/' + cam_name + "/color/intrinsics/fy")
        intrins_ppx = store.get('/camera/' + cam_name + "/color/intrinsics/ppx")
        intrins_ppy = store.get('/camera/' + cam_name + "/color/intrinsics/ppy")
        extrinsics = store.get('/camera/' + cam_name + "/color/depthExtrinsics")
        if intrins_fx is None or intrins_fy is None or intrins_ppx is None or intrins_ppy is None or extrinsics is None:
            raise RuntimeError("Could not get the intrinsicds for the camera {}. Not segmenting".format(cam_name))


        scale = store.get('/camera/' + cam_name + "/depth/scale")
        #get camera world location
        cam_pose_world = store.get(url + "pose")
        if scale is None or cam_pose_world is None:
            raise RuntimeError("Could not get the depth scale or coeffs for the camera {}. Not segmenting".format(cam_name))


        #get the 3d point by deprojecting pixel to get camera local
        depth_in_3d_cam_local = np.zeros((d_image.shape[0],d_image.shape[1],3))
        [xs, ys] = np.meshgrid(range(depth_in_3d_cam_local.shape[1]), range(depth_in_3d_cam_local.shape[0]))
        xs = (xs - intrins_ppx)/intrins_fx
        ys = (ys - intrins_ppy)/intrins_fy

        depth_in_3d_cam_local[:, :, 2] = d_image*scale
        depth_in_3d_cam_local[:, :, 0] = xs*depth_in_3d_cam_local[:, :, 2]
        depth_in_3d_cam_local[:, :, 1] = ys*depth_in_3d_cam_local[:, :, 2]

        depth_in_3d_cam_local = transform(extrinsics, depth_in_3d_cam_local).astype(np.float64)

        #get the transform from world to reflocal
        ref_world_xform_inv = np.linalg.inv(ref_world_xform)

        #compose these transforms
        cam_local_to_ref_local = np.array(ref_world_xform_inv.dot(cam_pose_world))

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
        maskx_volume = np.logical_and(depth_in_ref_local[:,:,0] > minx, depth_in_ref_local[:,:,0] < maxx)
        masky_volume = np.logical_and(depth_in_ref_local[:,:,1] > miny, depth_in_ref_local[:,:,1] < maxy)
        maskz_volume = np.logical_and(depth_in_ref_local[:,:,2] > minz, depth_in_ref_local[:,:,2] < maxz)
        maskz_volume_invalid = d_image == 0
        mask_volume = np.logical_and(np.logical_and(maskx_volume, masky_volume),maskz_volume)

        bin_bounds_local = []

        bin_bounds_local.append([minx,miny,minz,1])
        bin_bounds_local.append([minx,miny,maxz,1])
        bin_bounds_local.append([minx,maxy,minz,1])
        bin_bounds_local.append([minx,maxy,maxz,1])

        bin_bounds_local.append([maxx,miny,minz,1])
        bin_bounds_local.append([maxx,miny,maxz,1])
        bin_bounds_local.append([maxx,maxy,minz,1])
        bin_bounds_local.append([maxx,maxy,maxz,1])

        #put bin bounds local into camera local
        bin_bounds_in_camera_local = []
        for point in bin_bounds_local:
            bin_bounds_in_camera_local.append(np.linalg.inv(cam_local_to_ref_local).dot(np.array(point)))

        #project these points onto the 2d image
        pixel_bounds = []
        for point in bin_bounds_in_camera_local:
            tx = point[0]/point[2]
            ty = point[1]/point[2]
            p_x = tx * intrins_fx + intrins_ppx
            p_y = ty * intrins_fy + intrins_ppy
            pixel_bounds.append([p_x,p_y])

        hull = cv2.convexHull(np.array(pixel_bounds).astype('float32'))
        mask_project = cv2.fillConvexPoly(np.zeros(d_image.shape), hull.astype('int32'), 1)
        mask_project = np.logical_and(mask_project, d_image == 0)
        mask = np.logical_or(mask_project, mask_volume)
        
        # for bounding box debugging
        store.put(url + '/bounds_mask', mask)

        #optimized params
        seg_params.minSize = 3762.97
        seg_params.k = 840.067
        seg_params.c_rad = 7
        seg_params.sigma = 1.8066
        seg_params.sp_rad = 3

        seg_params.L_weight = 75.5078
        seg_params.A_weight = 50.6409984
        seg_params.B_weight = 57.456
        seg_params.depth_weight = 17.838
        seg_params.mask = mask

        #segment the image if it is not an inspection station

        if location == 'inspect':
            #dont segment just use the mask
            store.put(url + "labeled_image", np.where(mask == True, 1, 0))
            store.put(url + "point_cloud_segmented", depth_in_3d_cam_local)
            #create a DL image
            dl_tuple = segmentation.create_deep_learing_image(c_image, np.where(mask_volume == True,1,0),1, False, False)
            if dl_tuple is None:
                raise RuntimeError("Unable to create image for deep learning.")
            else:
                store.put(url + "DL_images", [dl_tuple[1]])
        else:
            ret = segmentation.graphSegmentation(d_image, c_image, depth_in_3d_cam_local, seg_params)
            #write out results to database
            store.put(url + "labeled_image", ret['labeled_image'])
            store.put(url + "DL_images", ret['DL_images'])
            store.put(url + "point_cloud_segmented", depth_in_3d_cam_local)


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