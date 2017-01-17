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
            self.context = rs.context()
            self.num_cameras = self.context.get_device_count()
            if self.num_cameras == 0:
                logger.warning("No cameras were detected")
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

        else:
            #tried to connect already...you idiot
            logger.warning("Tried to connect to cameras more than once. Ignoring") 
    
    def acquireImage(self, camera, stream, ivcam_preset=rs.RS_IVCAM_PRESET_OBJECT_SCANNING):
        
    