import realsense as rs
import numpy as np
import time
import logging
logger = logging.getLogger(__name__)

class DepthCameras:

    def __init__(self):
        self.context = None
        self.num_cameras = 0
        self.time_of_last_image = []

    def connect(self):
        if self.context is None:
            try:
                self.context = rs.context()
            except:
                logger.error("Camera is busy. Some other process has already connected")
                return False
        else:
            return False

        self.num_cameras = self.context.get_device_count()
        if self.num_cameras == 0:
            logger.warning("No cameras were detected")
            return False
        else:
            logger.info("Found %i cameras", self.num_cameras)
            for i in range(self.num_cameras):
                self.time_of_last_image.append(0)
                #cycle through all of the cameras and find out what streams they support
                for j in range(rs.capabilities_fish_eye):
                    cam = self.context.get_device(i)
                    if cam.supports_capability(j):
                        logger.info("Camera %i supports %s", i, rs.rs_stream_to_string(j))
            return True
        
    
    def acquire_image(self, camera, ivcam_preset=rs.RS_IVCAM_PRESET_OBJECT_SCANNING):
        if self.connect is None:
            logger.warning("No cameras connected, or connect has not been run")
            return (None, None)
        elif camera >= self.num_cameras or camera < 0:
            logger.warning("Camera requested does not exist")
            return (None, None)
        elif ivcam_preset < 0 or ivcam_preset >= rs.RS_IVCAM_PRESET_COUNT:
            logger.warning("Requested ivcam preset does not exist")
            return (None, None)
        else:
            #get the camera
            cam = self.context.get_device(camera)

            if cam is None:
                logger.error("Tried to get a camera that does not exist")
                return (None, None)
            
            #return the serial num of the camera too
            sn = cam.get_info(rs.camera_info_serial_number)
        
            try:
                #enable the stream
                for s in [rs.stream_color, rs.stream_depth, rs.stream_infrared]:
                    cam.enable_stream(s, rs.preset_best_quality)
            
                #apply the preset
                rs.apply_ivcam_preset(cam, ivcam_preset)
            except:
                logger.exception("Could not enable the stream or ivcam presets")
                return (None, None)

            #start the camera
            try:
                cam.start()
                 #wait for a single frame
                cam.wait_for_frames()
            except:
                logger.exception("Unable to start the camera")
                return (None, None)
           

            #get all the images at once
            try:
                imageFullColor = cam.get_frame_data_u8(rs.stream_color)
                if imageFullColor.size == 1:
                    #something went wrong
                    logger.error("Could not capture the full color image. Size of requested stream did not match")
                    imageFullColor = None
                else:
                    width = cam.get_stream_width(rs.stream_color)
                    height = cam.get_stream_height(rs.stream_color)    
                    imageFullColor = np.reshape(imageFullColor, (height, width, 3) )

                imageRectColor = cam.get_frame_data_u8(rs.stream_rectified_color)
                if imageRectColor.size == 1:
                    #something went wrong
                    logger.error("Could not capture the rectifiec image. Size of requested stream did not match")
                    imageRectColor = None
                else:
                    width = cam.get_stream_width(rs.stream_rectified_color)
                    height = cam.get_stream_height(rs.stream_rectified_color)    
                    imageRectColor = np.reshape(imageRectColor, (height, width, 3) )

                imageAllignedColor = cam.get_frame_data_u8(rs.stream_color_aligned_to_depth)
                if imageAllignedColor.size == 1:
                    #something went wrong
                    logger.error("Could not capture the depth alinged image. Size of requested stream did not match")
                    imageAllignedColor = None
                else:
                    width = cam.get_stream_width(rs.stream_color_aligned_to_depth)
                    height = cam.get_stream_height(rs.stream_color_aligned_to_depth)    
                    imageAllignedColor = np.reshape(imageAllignedColor, (height, width, 3) )

                imageDepth = cam.get_frame_data_u16(rs.stream_depth)
                if imageDepth.size == 1:
                    logger.error("Could not capture the depth image. Size of requested stream did not match")
                    imageDepth = None
                else:
                    width = cam.get_stream_width(rs.stream_depth)
                    height = cam.get_stream_height(rs.stream_depth)    
                    imageDepth = np.reshape(imageDepth, (height, width) )

                
                imageAlignedDepth = cam.get_frame_data_u16(rs.stream_depth_aligned_to_color)
                if imageAlignedDepth.size == 1:
                    logger.error("Could not capture the depth alinged to color image. Size of requested stream did not match")
                    imageAlignedDepth = None
                else:
                    width = cam.get_stream_width(rs.stream_depth)
                    height = cam.get_stream_height(rs.stream_depth)    
                    imageAlignedDepth = np.reshape(imageAlignedDepth, (height, width) )

                points = cam.get_frame_data_f32(rs.stream_points)
                if points.size == 1:
                    #something went wrong
                    logger.error("Could not capture the point cloud. Size of requested stream did not match")
                    points = None
                else:
                    width = cam.get_stream_width(rs.stream_points)
                    height = cam.get_stream_height(rs.stream_points)    
                    points = np.reshape(points, (height, width, 3) )

            except:
                logger.exception("Unable to capture images, unkown reason")
                cam.stop()
                return (None, None)

            #we are done now
            try:
                cam.stop()
            except:
                logger.error("Could not stop the camera {}".format(sn))

            logger.info("Obtained images")
            images = [imageFullColor, imageAllignedColor, imageRectColor,
            imageDepth, imageAlignedDepth, points]

            #check if the color image is all black. If so the camera is not connected
            if np.array_equal(imageFullColor, np.ones((480,640))*255):
                return (None, None)

            #got an image record the time stamp 
            self.time_of_last_image[camera] = time.time()
            return (images, sn)

    def get_online_cams(self):
        if self.context is None:
            logger.warning("Tried to access online cams without connecting")
            return []
        online_cams = []
        for cam in range(self.num_cameras):
            try:
                #only way to tell if a camera is connected is to get images
                res = self.acquire_image(cam)
                if not res[0] is None: 
                   online_cams.append(res[1])
            except:
                logger.exception("Tried to access camera {}, but an error occured".format(cam))
                continue
        
        return online_cams

    def get_camera_coefs(self, camera, stream):
        '''
        retruns camera coefficents and matrix for calibration
        used for opencv
        '''
        if self.context is None:
            logger.warning("Tried to access camera matrix without connecting")
            return (None, None)
        cam = self.context.get_device(camera)
        try:
            cam.enable_stream(stream, rs.preset_best_quality)
            intrinsics = cam.get_stream_intrinsics(stream)
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

    def get_camera_intrinsics(self, camera, stream):
        '''
        retruns camera intrinsics. used to deproject points back into real world coordinates
        '''
        if self.context is None:
            logger.warning("Tried to access camera intrinsics without connecting")
            return (None, None)
        cam = self.context.get_device(camera)
        try:
            cam.enable_stream(stream, rs.preset_best_quality)
            intrinsics = cam.get_stream_intrinsics(stream)
        except:
            logger.exception("Unable to enable stream and get intrinsics")
            return (None, None)
        return intrinsics

    def get_camera_depthscale(self, camera):
        '''
        returns depth scale for camera. multiply this value by the value
        in the depth image to get real distance
        '''
        if self.context is None:
            logger.warning("Tried to access camera depth scale without connecting")
            return None
        cam = self.context.get_device(camera)
        try:
            scale = cam.get_depth_scale()
        except:
            logger.exception("Unable to get depth scale")
            return None
        return scale

    def get_extrinsics(self, camera, streamfrom, streamto):
        '''Retruns extrinsics which convert points from streamfrom to points
        in streamto
        '''
        if self.context is None:
            logger.warning("Tried to access camera extrinsics without connecting")
            return None
        cam = self.context.get_device(camera)
        try:
            extrinsics = cam.get_extrinsics(streamfrom, streamto)
        except:
            logger.exception("Unable to enable stream and get extrinsics")
            return None
        return extrinsics

def test():
    import matplotlib
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    x = DepthCameras()
    if not x.connect():
        print("could not connect to the cameras")
    y = x.acquire_image(0)
    if not y is None:
        plt.imshow(y[0][0])
        plt.show()

if __name__ == "__main__":
    test()