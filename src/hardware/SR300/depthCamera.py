import realsense as rs
import numpy as np

import logging
logger = logging.getLogger(__name__)

class DepthCameras:

    def __init__(self):
        self.context = None
        self.num_cameras = 0
        self.supported_streams = []

    def connect(self):
        #check if the context exists, if not try to connect
        if self.context is None:
            try:
                self.context = rs.context()
            except:
                logger.error("Camera is busy. Some other process has already connected")
                return False
            self.num_cameras = self.context.get_device_count()
            if self.num_cameras == 0:
                logger.warning("No cameras were detected")
                return False
            else:
                logger.info("Found %i cameras", self.num_cameras)
                for i in range(self.num_cameras):
                    self.supported_streams.append([])

                    #cycle through all of the cameras and find out what streams they support
                    for j in range(rs.capabilities_fish_eye):
                        cam = self.context.get_device(i)
                        if cam.supports_capability(j):
                            self.supported_streams[i].append(j)
                            logger.info("Camera %i supports %s", i, rs.rs_stream_to_string(j))
                return True
        else:
            #tried to connect already...you idiot
            logger.warning("Tried to connect to cameras more than once. Ignoring") 
            return False
    
    def acquireImage(self, camera, stream, ivcam_preset=rs.RS_IVCAM_PRESET_OBJECT_SCANNING):
        if self.connect is None:
            logger.warning("No cameras connected, or connect has not been run")
            return None
        elif camera >= self.num_cameras or camera < 0:
            logger.warning("Camera requested does not exist")
            return None
        # elif stream not in self.supported_streams[camera]:
            # logger.warning("Requested stream that is not supported")
            # return None
        elif ivcam_preset < 0 or ivcam_preset >= rs.RS_IVCAM_PRESET_COUNT:
            logger.warning("Requested ivcam preset does not exist")
            return None
        else:
            #get the camera
            cam = self.context.get_device(camera)

            if cam is None:
                logger.error("Tried to get a camera that does not exist")
                return None
            
            #enable the stream
            for s in self.supported_streams[camera]:
                cam.enable_stream(s, rs.preset_best_quality)
           
            #apply the preset
            rs.apply_ivcam_preset(cam, ivcam_preset)

            #start the camera
            cam.start()

            #wait for a single frame
            cam.wait_for_frames()


            #switch on the stream the user requested to select the correct method
            if stream in [rs.RS_STREAM_COLOR, rs.RS_STREAM_RECTIFIED_COLOR, rs.RS_STREAM_COLOR_ALIGNED_TO_DEPTH]:
                try:
                    image = cam.get_frame_data_u8(stream)
                except:
                    logger.error("Unable to capture the image, unkown reason")
                    return None
                
                if(image.size == 1):
                    #something went wrong
                    logger.error("Could not capture the image. Size of requested stream did not match")
                    return None
                width = cam.get_stream_width(stream)
                height = cam.get_stream_height(stream)    
                image = np.reshape(image, (height, width, 3) )

            elif stream in [rs.RS_STREAM_DEPTH, rs.RS_STREAM_INFRARED, rs.RS_STREAM_DEPTH_ALIGNED_TO_COLOR, rs.RS_STREAM_DEPTH_ALIGNED_TO_RECTIFIED_COLOR]:
                try:
                    image = cam.get_frame_data_u16(stream)
                except:
                    logger.error("Unable to capture the image, unkown reason")
                    return None
                if(image.size == 1):
                    #something went wrong
                    logger.error("Could not capture the image. Size of requested stream did not match")
                    return None

                width = cam.get_stream_width(stream)
                height = cam.get_stream_height(stream)    
                image = np.reshape(image, (height, width) )
            
            elif stream == rs.stream_points:
                try:
                    image = cam.get_frame_data_f32(stream)
                except:
                    logger.error("Unable to capture the image, unkown reason")
                    return None

                if(image.size == 1):
                    #something went wrong
                    logger.error("Could not capture the image. Size of requested stream did not match")
                    return None

                width = cam.get_stream_width(stream)
                height = cam.get_stream_height(stream)    
                image = np.reshape(image, (height, width, 3) )

            #we are done now
            cam.stop()

            return image

    def isOnline(camera):
        if camera not int:
            logger.warning("User passed a non integer argument to see if a camera was online")
            return False
        if self.connect is None:
            logger.warning("No cameras connected, or connect has not been run")
            return False
        elif camera >= self.num_cameras or camera < 0:
            logger.warning("Camera requested does not exist")
            return False
        
        try:
            cam = self.context.get_device(camera)
        except:
            logger.error("Tried to access camera {}, but an error occured".format(camera))
            return False
        
        if cam not None:
            return True
        else:
            return False
            

def test():
    import matplotlib.pyplot as plt
    x = DepthCameras()
    if not x.connect():
        print("could not connect to the cameras")
    y = x.acquireImage(0, rs.stream_color_aligned_to_depth)
    plt.imshow(y)
    plt.show()

if __name__ == "__main__":
    test()