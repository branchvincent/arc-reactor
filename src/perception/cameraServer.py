import cv2
import time
import sys
import copy
import hardware.SR300.realsense as rs
from PyQt5 import QtGui, QtCore, QtWidgets
import numpy as np
from pensive.client import PensiveClient
from pensive.coders import register_numpy
import logging

logger = logging.getLogger(__name__)

class CameraServer(QtWidgets.QWidget):
    
    def __init__(self):
        super(CameraServer, self).__init__()
        self.initCameras()      
        self.initUI()

        self.current_settings_camera = 0
        self.video_thread = VideoThread()
        self.video_thread.signal.connect(self.receive_images)
        self.video_thread.start()
        self.db_poll_thread = PollDBThread()

        self.db_poll_thread.signal.connect(self.take_images)
        self.video_thread.inspect_below_sn = self.db_poll_thread.inspect_below_sn
        self.video_thread.stow_sn = self.db_poll_thread.stow_sn
        self.db_poll_thread.start()
        self.show()


    def closeEvent(self, closeEv):
        self.video_thread.keepRunning = False
        logger.info("Stopping cameras")
        while self.video_thread.isRunning():
            time.sleep(0.001)
        logger.info("Cameras stopped")

    def initUI(self):
        #split the screen into two halves
        #left half is for color images 
        #right half is for settings
        self.horzLayout = QtWidgets.QHBoxLayout()
        self.setLayout(self.horzLayout)

        #grid layout for camera color images
        self.camera_grid_layout = QtWidgets.QGridLayout()

        self.dictionary_of_camera_displays = {}
        for i in range(len(self.camera_sn)):
            #for each camera create a color image view
            cam_display = QtWidgets.QLabel(self)
            cam_display.show()
            zeroImg = np.zeros((480,640,3),dtype='uint8')
            image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap(image)
            cam_display.setPixmap(pix.scaled(480,360))

            #add it to the list of views
            self.dictionary_of_camera_displays[self.camera_sn[i]] = cam_display

            #put it in the grid
            self.camera_grid_layout.addWidget(cam_display, i%2, i//2)
            
        self.horzLayout.addItem(self.camera_grid_layout)

    def initCameras(self):
        try:
            context = rs.context()
        except:
            raise RuntimeError("Could not acquire context for Realsense cameras. Some other process has already connected")

        self.num_cameras = context.get_device_count()
        if self.num_cameras == 0:
            raise RuntimeError("No cameras were detected")
        
        logger.info("Found %i cameras", self.num_cameras)
        
        
        self.camera_sn = []
        #cycle through and set the camera options
        for num in range(self.num_cameras):
            cam = context.get_device(num)

            self.camera_sn.append(str(cam.get_info(rs.camera_info_serial_number)))

    def receive_images(self, image_dictionary):
        #update the view
        imageFullColor = image_dictionary['full_color']
        sn = image_dictionary['serial_number']
        self.update_camera_view(imageFullColor, sn)


    def take_images(self, camera_serials, urls):
        
        logger.info("Taking image(s) for camera(s) {}".format(camera_serials))
        self.video_thread.camera_sn_q = camera_serials[:] 
        
        for sn,url in zip(camera_serials,urls):
            cur_time = time.time()
            while len(self.video_thread.camera_sn_q) != 0:
                time.sleep(0.01)
                #timeout 
                if(time.time() - cur_time > 3):
                    error_string = "Unable to get new images from camera {}".format(sn)
                    logger.error(error_string)
                    self.db_poll_thread.error_string = error_string
                    self.db_poll_thread.images_acquired = True
                    break
                
            self.db_poll_thread.image_dictionaries.append(copy.deepcopy(self.video_thread.sn_to_image_dictionaries[sn]))
            self.db_poll_thread.urls.append(url)

        self.db_poll_thread.images_acquired = True
        logger.info("Image(s) acquired")

        self.video_thread.keepRunning = False
        logger.info("Stopping camera thread")
        while self.video_thread.isRunning():
            time.sleep(0.001)
        logger.info("Camera thread stopped. Restarting")
        self.video_thread.keepRunning = True

        time.sleep(1.5)
        self.video_thread.start()

    def update_camera_view(self, color_image, sn):
        label = self.dictionary_of_camera_displays[sn]
        image = QtGui.QImage(color_image, color_image.shape[1], color_image.shape[0], color_image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        label.setPixmap(pix.scaled(480,360))
      

class PollDBThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(list, list)
    def __init__(self):
        QtCore.QThread.__init__(self)
        #try to connect to the database
        self.store = None
        try:
            self.db_client = PensiveClient()
            self.store = self.db_client.default()
            register_numpy()
        except:
            raise RuntimeError('Could not connect to the database. Exiting')

        self.images_acquired = False
        self.error_string = None
        self.image_dictionaries = []
        self.urls = []
        self.inspect_below_sn = self.store.get('/system/cameras/inspect_below')
        self.stow_sn = self.store.get('/system/cameras/stow')
    def run(self):
        '''
        Polls the database at a specific URL until a flag is set.
        Once the flag is set run inference and return to polling
        '''
        try:
            while True:
                should_run = self.store.get('/acquire_images/run')
                if should_run:
                    self.error_string = None
                    logger.info("Starting to save images")
                    self.store.put('/acquire_images/run',0)
                    #get the arguments
                    list_of_serial_nums = self.store.get('/acquire_images/serial_numbers')
                    list_of_urls = self.store.get('/acquire_images/urls')
                    #check inputs
                    if list_of_urls is None or list_of_serial_nums is None:
                        self.store.put('/acquire_images/error/',"Arguments cant be None")
                        logger.error("Arguments cant be None")
                        self.store.put('/acquire_images/done', 1)
                        continue
                    #check to make sure the length of the urls == len of serial nums
                    elif len(list_of_urls) != len(list_of_serial_nums):
                        self.store.put('/acquire_images/error/',"Length mismatch of url list and serial number list. Not acquiring")
                        logger.error("Length mismatch of url list and serial number list. Not acquiring")
                        self.store.put('/acquire_images/done', 1)
                        continue

                    elif len(list_of_urls) == 0:
                        self.store.put('/acquire_images/error/',"No URLs were passed. Need at least one. Not acquiring")
                        logger.error("No URLs were passed. Need at least one. Not acquiring")
                        self.store.put('/acquire_images/done', 1)
                        continue

                    elif len(list_of_serial_nums) == 0:
                        self.store.put('/acquire_images/error/',"No serial numbers were passed. Need at least one. Not acquiring")
                        logger.error("No serial numbers were passed. Need at least one. Not acquiring")
                        self.store.put('/acquire_images/done', 1)
                        continue

                    
                    self.images_acquired = False
                    self.signal.emit(list_of_serial_nums, list_of_urls)
                    while not self.images_acquired:
                        time.sleep(0.01)
                    
                    logger.info("Starting write to DB")
                    #image was acquired write stuff out to the database
                    for i, image_dictionary in enumerate(self.image_dictionaries):
                        send_dict = {key: value for (key, value) in image_dictionary.items() if key not in ['error', 'serial_number']}
                        self.store.multi_put(send_dict,root=self.urls[i])
                    

                    #tell the DB we are done
                    self.store.put('/acquire_images/done', 1)
                    self.store.put('acquire_images/error', self.error_string)
                    if not self.error_string is None:
                        #log the error
                        logger.error(self.error_string)

                    logger.info("Ending write to DB")

                    #clear the urls and image dictionaries
                    self.urls = []
                    self.image_dictionaries = []

                    logger.info("Saving images complete")
                else:
                    time.sleep(0.1)
        except:
            logger.exception('Polling DB failed?!')
            raise



class VideoThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(dict)
    def __init__(self):
        QtCore.QThread.__init__(self)
        self.keepRunning = True
        self.sn_to_camera = {}
        self.sn_to_image_dictionaries = {}
        self.camera_sn_q = [] #list of serial numbers to get images from
        self.inspect_below_sn = None #serial number for the inspect below camera
    def run(self):
        logger.info("Starting camera thread")
        try:
            context = rs.context()
        except:
            logger.exception("Unable to aquire context")
            raise RuntimeError("Unable to acquire context")
            #exit gracefully
            sys.exit(-1)

        num_cams = context.get_device_count()            
        if num_cams == 0:
            logger.warn("No cameras attached")
            return

        logger.info("Found {} cameras".format(num_cams))
        #enable all of the cameras
        for i in range(num_cams):
            cam = context.get_device(i)
            try:
                #enable the stream
                for s in [rs.stream_color, rs.stream_depth, rs.stream_infrared]:
                    cam.enable_stream(s, rs.preset_best_quality)
            except:
                logger.exception("Could not enable the stream")
                return
            sn = cam.get_info(rs.camera_info_serial_number)
            #set up settings
            try:
                cam.set_option(rs.option_f200_filter_option, 1)
                cam.set_option(rs.option_f200_confidence_threshold, 4)
                cam.set_option(rs.option_f200_accuracy, 1)
                cam.set_option(rs.option_f200_motion_range, 10)
                cam.set_option(rs.option_f200_laser_power, 16) #turn the laser on for all cameras

                if sn == self.inspect_below_sn:
                    #turn off auto exposure
                    cam.set_option(rs.option_color_exposure, 850)

                if sn == self.stow_sn:
                    cam.set_option(rs.option_f200_motion_range, 70)
            except:
                logger.warning("Unable to set options for camera {}".format(cam.get_info(rs.camera_info_serial_number)))

            #add to class structures
            
            self.sn_to_image_dictionaries[sn] = {}
            self.sn_to_image_dictionaries[sn]['time_stamp'] = time.time()
            self.sn_to_camera[sn] = cam

        while self.keepRunning:
            #check to see if we need to acquire an image from a camera
            if len(self.camera_sn_q) != 0:
                while len(self.camera_sn_q) != 0:
                    sn = self.camera_sn_q[0]
                    logger.info("Taking image from camera {}".format(sn))
                    self.captureImage(sn)
                    logger.info("Image acquired")
                    self.camera_sn_q.remove(sn)
                
            else:
                time.sleep(0.05)

        #clear variables
        self.sn_to_camera = {}
        self.sn_to_image_dictionaries = {}
        self.camera_sn_q = []
        self.context_acquired = False

    def captureImage(self, sn):
        try:
            cam = self.sn_to_camera[sn]
        except KeyError as ke:
            logger.exception("Got a key error for sn {}".format(sn))
            return

        cam.start()
        image_dictionary = {}
        imageFullColor = None
        color_to_depth = None
        depth_to_color = None
        points = None
        error_string = None

        #if there is a new frame send it out
        try:
            cam.wait_for_frames()
            imageFullColor = cam.get_frame_data_u8(rs.stream_color)
            if imageFullColor.size == 1:
                #something went wrong
                error_string = "Could not capture the full color image. Size of requested stream did not match" 
                logger.error(error_string)
                imageFullColor = None
            else:
                width = cam.get_stream_width(rs.stream_color)
                height = cam.get_stream_height(rs.stream_color)
                imageFullColor = np.reshape(imageFullColor, (height, width, 3) )

            color_to_depth = cam.get_frame_data_u8(rs.stream_color_aligned_to_depth)
            if color_to_depth.size == 1:
                #something went wrong
                error_string = "Could not capture the depth alinged image. Size of requested stream did not match"
                logger.error(error_string)
                color_to_depth = None
            else:
                width = cam.get_stream_width(rs.stream_color_aligned_to_depth)
                height = cam.get_stream_height(rs.stream_color_aligned_to_depth)
                color_to_depth = np.reshape(color_to_depth, (height, width, 3) )

            depth_to_color = cam.get_frame_data_u16(rs.stream_depth_aligned_to_color)
            if depth_to_color.size == 1:
                #something went wrong
                error_string = "Could not capture the depth alinged image. Size of requested stream did not match"
                logger.error(error_string)
                depth_to_color = None
            else:
                width = cam.get_stream_width(rs.stream_depth_aligned_to_color)
                height = cam.get_stream_height(rs.stream_depth_aligned_to_color)
                depth_to_color = np.reshape(depth_to_color, (height, width) )

            points = cam.get_frame_data_f32(rs.stream_points)
            if points.size == 1:
                #something went wrong
                error_string = "Could not capture the point cloud. Size of requested stream did not match"
                logger.error(error_string)
                points = None
            else:
                width = cam.get_stream_width(rs.stream_points)
                height = cam.get_stream_height(rs.stream_points)
                points = np.reshape(points, (height, width, 3) )

        except Exception as e:
            error_string = "Error in waiting for frames. {}".format(e)
            logger.exception(error_string)
            
            

        image_dictionary['full_color'] = imageFullColor
        image_dictionary['aligned_color'] = color_to_depth
        image_dictionary['aligned_depth'] = depth_to_color
        image_dictionary['point_cloud'] = points
        image_dictionary['serial_number'] = sn
        image_dictionary['time_stamp'] = time.time()
        image_dictionary['error'] = error_string
        self.signal.emit(image_dictionary)
        self.sn_to_image_dictionaries[sn] = image_dictionary
        cam.stop()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    m3 = CameraServer()
    sys.exit(app.exec_())
