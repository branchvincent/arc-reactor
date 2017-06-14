import cv2
import time
import sys
import hardware.SR300.realsense as rs
from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
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
        self.video_thread = VideoThread(self.context)
        self.video_thread.signal.connect(self.receive_images)
        self.video_thread.start()
        self.db_poll_thread = PollDBThread()

        self.db_poll_thread.signal.connect(self.take_image)
        self.db_poll_thread.start()
        self.show()

    def initUI(self):
        #split the screen into two halves
        #left half is for color images 
        #right half is for settings
        self.horzLayout = QtWidgets.QHBoxLayout()
        self.setLayout(self.horzLayout)

        #grid layout for camera color images
        self.camera_grid_layout = QtWidgets.QGridLayout()

        self.list_of_camera_displays = []
        for i in range(self.num_cameras):
            #for each camera create a color image view
            cam_display = QtWidgets.QLabel(self)
            cam_display.show()
            zeroImg = np.zeros((480,640,3),dtype='uint8')
            image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap(image)
            cam_display.setPixmap(pix.scaled(480,360))

            #add it to the list of views
            self.list_of_camera_displays.append(cam_display)

            #put it in the grid
            self.camera_grid_layout.addWidget(cam_display, i%2, i//2)
            

        #create a settings panel
        self.camera_selector_combo = QtWidgets.QComboBox(self)
        self.cam_serial_nums = []
        for i in range(self.num_cameras):
            cam = self.context.get_device(i)
            #return the serial num of the camera too
            sn = cam.get_info(rs.camera_info_serial_number)
            self.cam_serial_nums.append(sn)

        #fill in the combo box with the cameras attached
        self.camera_selector_combo.addItems(self.cam_serial_nums)
        self.camera_selector_combo.currentIndexChanged.connect(self.change_camera)
        
        self.settings_layout = QtWidgets.QVBoxLayout()
        self.settings_layout.addStretch(1)
        self.settings_layout.addWidget(self.camera_selector_combo)
        self.settings_layout.addSpacing(20)

        self.list_of_sliders = []
        self.list_of_slider_labels = []
        self.settings_names = ['Gain','White Balance', 'Exposure','Auto Exposure', 'Auto White Balance']
        #min and max settings for slider
        setting_ranges = [[0,128],[2800,6500],[40,3000],[0,1],[0,1]]
        for i,setting in enumerate(self.settings_names):
            #make the label
            label = QtWidgets.QLabel(self)
            label.setText(setting + ": " + str(int(self.camera_options[0][setting])))
            label.show()
            self.settings_layout.addWidget(label)
            self.list_of_slider_labels.append(label)

            #make the slider
            slider = QtWidgets.QSlider(self)
            slider.setOrientation(QtCore.Qt.Horizontal)
            slider.setMinimum(setting_ranges[i][0])
            slider.setMaximum(setting_ranges[i][1])
            self.settings_layout.addWidget(slider)
            self.list_of_sliders.append(slider)
            #read the current setting for the camera to set it for the first camera in the list
            slider.setValue(self.camera_options[0][setting])
            slider.valueChanged.connect(self.slider_changed)
            self.settings_layout.addSpacing(20)
        self.settings_layout.addStretch(1)
        self.horzLayout.addItem(self.camera_grid_layout)
        self.horzLayout.addItem(self.settings_layout)

    def initCameras(self):
        try:
            self.context = rs.context()
        except:
            raise RuntimeError("Could not acquire context for Realsense cameras. Some other process has already connected")

        self.num_cameras = self.context.get_device_count()
        if self.num_cameras == 0:
            raise RuntimeError("No cameras were detected")
        
        logger.info("Found %i cameras", self.num_cameras)
        
        
        #list of dictionaries        
        self.camera_options = []
        #dictionary that converts serial number to camera number
        self.camera_sn_to_number = {}
        #cycle through and set the camera options
        for num in range(self.num_cameras):
            cam = self.context.get_device(num)

            self.camera_sn_to_number[str(cam.get_info(rs.camera_info_serial_number))] = num

            try:
                cam.set_option(rs.option_f200_filter_option, 1)
                cam.set_option(rs.option_f200_confidence_threshold, 4)
                cam.set_option(rs.option_f200_accuracy, 1)
                cam.set_option(rs.option_f200_motion_range, 10)
                cam.set_option(rs.option_f200_laser_power, 0) #turn the laser off for all cameras
            except:
                logger.warning("Unable to set options for camera {}".format(cam.get_info(rs.camera_info_serial_number)))


            try:
                options_dict = {}
                
                #get the current options
                wb = cam.get_option(rs.option_color_white_balance)
                gain = cam.get_option(rs.option_color_gain)
                exposure = cam.get_option(rs.option_color_exposure)
                autoexp = cam.get_option(rs.option_color_enable_auto_exposure)
                autowb = cam.get_option(rs.option_color_enable_auto_white_balance)

                options_dict['White Balance'] = wb
                options_dict['Gain'] = gain
                options_dict['Exposure'] = exposure
                options_dict['Auto Exposure'] = autoexp
                options_dict['Auto White Balance'] = autowb
                self.camera_options.append(options_dict)

            except:
                logger.warning("Unable to retrieve options for camera {}".format(cam.get_info(rs.camera_info_serial_number)))


    def slider_changed(self):
        slider = self.sender()
        #get the index of this slider
        ind = self.list_of_sliders.index(slider)

        #get the label
        label = self.list_of_slider_labels[ind]

        #update the label with the setting value
        val = slider.value()
        label.setText(self.settings_names[ind] + ": " + str(val))

        rs_options = [rs.option_color_gain,
                        rs.option_color_white_balance, 
                        rs.option_color_exposure,
                        rs.option_color_enable_auto_exposure,
                        rs.option_color_enable_auto_white_balance]

        #change it on the camera that is currently selected from the drop down box
        cam = self.context.get_device(self.current_settings_camera)
        try:
            cam.set_option(rs_options[ind],val)
            self.camera_options[self.current_settings_camera][self.settings_names[ind]] = val
        except:
            logger.warning("Unable to set options for camera {}".format(cam.get_info(rs.camera_info_serial_number)))


    def change_camera(self, cam_num):
        self.current_settings_camera = cam_num
        cam = self.context.get_device(cam_num)
        logger.info("Changing camera settings for {}".format(cam.get_info(rs.camera_info_serial_number)))
        #get the current options
        wb = cam.get_option(rs.option_color_white_balance)
        gain = cam.get_option(rs.option_color_gain)
        exposure = cam.get_option(rs.option_color_exposure)
        autoexp = cam.get_option(rs.option_color_enable_auto_exposure)
        autowb = cam.get_option(rs.option_color_enable_auto_white_balance)

        self.camera_options[cam_num]['White Balance'] = wb
        self.camera_options[cam_num]['Gain'] = gain
        self.camera_options[cam_num]['Exposure'] = exposure
        self.camera_options[cam_num]['Auto Exposure'] = autoexp
        self.camera_options[cam_num]['Auto White Balance'] = autowb
        for i,setting in enumerate(self.settings_names):
            self.list_of_sliders[i].setValue(self.camera_options[cam_num][setting])

    def receive_images(self, image_dictionary):
        #update the view
        imageFullColor = image_dictionary['full_color']
        num = image_dictionary['number']
        self.update_camera_view(imageFullColor, num)


    def take_image(self, camera_serial, url):
        try:
            cam = self.context.get_device(self.camera_sn_to_number[camera_serial])
        except:
            error_string = "Unable to get camera with serial number {}".format(camera_serial)
            logger.error(error_string)
            self.db_poll_thread.image_aquired = True
            self.db_poll_thread.error_string = error_string
            return
        
        logger.info("Taking image for camera {}".format(cam.get_info(rs.camera_info_serial_number)))
        
        #turn the laser on
        try:
            cam.set_option(rs.option_f200_laser_power,16)
        except:
            error_string = "Could not turn laser on for camera {}".format(cam.get_info(rs.camera_info_serial_number))
            logger.critical(error_string)
            self.db_poll_thread.error_string = error_string
        cur_time = time.time()
        while (self.video_thread.list_of_image_dictionaries[self.current_settings_camera]["time_stamp"] - cur_time) < 0.1:
            time.sleep(0.01)
            #timeout 
            if(time.time() - cur_time > 3):
                error_string = "Unable to get new images from camera {}".format(cam.get_info(rs.camera_info_serial_number))
                logger.critical(error_string)
                self.db_poll_thread.error_string = error_string
                self.db_poll_thread.image_aquired = True
                break
        try:
            cam.set_option(rs.option_f200_laser_power,0)
        except:
            error_string = "Could not turn laser off for camera {}".format(cam.get_info(rs.camera_info_serial_number))
            logger.critical(error_string)
            self.db_poll_thread.error_string = error_string
        
    
            
        self.db_poll_thread.image_dictionary = self.video_thread.list_of_image_dictionaries[self.current_settings_camera]
        self.db_poll_thread.url = url
        self.db_poll_thread.image_aquired = True

    def update_camera_view(self, color_image, camera_num):
        label = self.list_of_camera_displays[camera_num]
        image = QtGui.QImage(color_image, color_image.shape[1], color_image.shape[0], color_image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        label.setPixmap(pix.scaled(480,360))
      

class PollDBThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(str, str)
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

        self.image_aquired = False
        self.error_string = None
        self.image_dictionary = {}
        self.url = "/"

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
                        self.store.put('/acquire_images/done', 1)
                        continue
                    #check to make sure the length of the urls == len of serial nums
                    elif len(list_of_urls) != len(list_of_serial_nums):
                        self.store.put('/acquire_images/error/',"Length mismatch of url list and serial number list. Not acquiring")
                        self.store.put('/acquire_images/done', 1)
                        continue

                    elif len(list_of_urls) == 0:
                        self.store.put('/acquire_images/error/',"No URLs were passed. Need at least one. Not acquiring")
                        self.store.put('/acquire_images/done', 1)
                        continue

                    elif len(list_of_serial_nums) == 0:
                        self.store.put('/acquire_images/error/',"No serial numbers were passed. Need at least one. Not acquiring")
                        self.store.put('/acquire_images/done', 1)
                        continue

                    for i in range(len(list_of_serial_nums)):
                        self.image_aquired = False
                        self.signal.emit(list_of_serial_nums[i], list_of_urls[i])
                        while not self.image_aquired:
                            time.sleep(0.01)
                        #image was acquired write stuff out to the database
                        for key in self.image_dictionary.keys():
                            if key != 'number' and key != 'error':
                                db_key = self.url + "/" + key
                                self.store.put(db_key, self.image_dictionary[key])
                            elif key == 'error' and self.image_dictionary[key] != None
                                self.error_string = self.image_dictionary[key]
                                
                    self.store.put('/acquire_images/done', 1)
                    self.store.put('acquire_images/error', self.error_string)
                    logger.info("Saving images complete")
                else:
                    time.sleep(0.1)
        except:
            logger.exception('Polling DB failed?!')
            raise



class VideoThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(dict)
    def __init__(self, context):
        QtCore.QThread.__init__(self)
        self.context = context
        self.keepRunning = True
        self.list_of_image_dictionaries = []
    def run(self):
        num_cams = self.context.get_device_count()            
        if num_cams == 0:
            logger.warn("No cameras attached")
            return
        #enable all of the cameras
        cameras = []
        for i in range(num_cams):
            cam = self.context.get_device(i)
            try:
                #enable the stream
                for s in [rs.stream_color, rs.stream_depth, rs.stream_infrared]:
                    cam.enable_stream(s, rs.preset_best_quality)
            except:
                logger.exception("Could not enable the stream")
                return

            cam.set_option(rs.option_f200_laser_power, 0)
            cam.start()
            cameras.append(cam)
            self.list_of_image_dictionaries.append({})

        while self.keepRunning:
            for num,cam in enumerate(cameras):
                image_dictionary = {}
                #if there is a new frame send it out
                if cam.poll_for_frames():
                    error_string = None
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

                    image_dictionary['full_color'] = imageFullColor
                    image_dictionary['aligned_color'] = color_to_depth
                    image_dictionary['aligned_depth'] = depth_to_color
                    image_dictionary['point_cloud'] = points
                    image_dictionary['number'] = cameras.index(cam)
                    image_dictionary['time_stamp'] = time.time()
                    image_dictionary['error'] = error_string
                    self.signal.emit(image_dictionary)
                    self.list_of_image_dictionaries[num] = image_dictionary

        for cam in cameras:
            cam.stop()       


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    m3 = CameraServer()
    sys.exit(app.exec_())
