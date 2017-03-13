from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
import sys
import time
from OpenGL import GL, GLU
from perception import Perception
from functools import partial
import math
import numpy as np
import os
 # start logging
import logging
logger = logging.getLogger(__name__)
# set up logging to server for all records
# from log.handler import LogServerHandler
# handler = LogServerHandler('10.10.1.102', 7777)
# logging.getLogger().addHandler(handler)

# configure the root logger to accept all records
logger = logging.getLogger()
logger.setLevel(logging.NOTSET)

formatter = logging.Formatter('%(asctime)s\t[%(name)s] %(pathname)s:%(lineno)d\t%(levelname)s:\t%(message)s')

# set up colored logging to console
from rainbow_logging_handler import RainbowLoggingHandler
console_handler = RainbowLoggingHandler(sys.stderr)
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)


class PerceptionGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(PerceptionGUI, self).__init__()
        self.perception_obj = Perception()
        self.initUI()
        self.map_cameras_to_views()
        
    def initUI(self):
        
        #add a vertical layout
        self.vertLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.vertLayout)

        #add horizontal layouts
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzBot = QtWidgets.QHBoxLayout()
      
        #top horizontal layout
        self.run_button = QtWidgets.QPushButton("Run", self)
        self.run_button.show()
        self.run_button.clicked.connect(self.run_perception_pipeline)
        
        self.continuous_check_box = QtWidgets.QCheckBox("Continuous?", self)
        self.continuous_check_box.show()

        self.horzTop.addStretch(1)
        self.horzTop.addWidget(self.run_button)
        self.horzTop.addSpacing(5)
        self.horzTop.addWidget(self.continuous_check_box)
        self.horzTop.addStretch(1)

        #one for the top and one for the bottom
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzBot)

        #add gl widget to the bottom left
        self.glWidget = GLWidget(self)
        self.gridLayout = QtWidgets.QGridLayout()
        self.horzBot.addWidget(self.glWidget)
        self.horzBot.addItem(self.gridLayout)

        #tuples of combo boxes and qlabels for images
        self.camera_views = []
        #map of cameras to camera views
        self.cams2views = []

        #add N camera views to the grid layout
        num_views = 6
        for i in range(num_views):
            #create layout
            vert_layout = QtWidgets.QVBoxLayout()

            #add combo box
            combo_box = QtWidgets.QComboBox(self)
            combo_box.addItems(['Color', 'Depth', 'Annotated'])
            combo_box.currentIndexChanged.connect(self.change_display_images)

            #add display
            camera_display = QtWidgets.QLabel(self)
            zeroImg = np.zeros((480,640,3),dtype='uint8')
            image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap(image)
            camera_display.setPixmap(pix.scaled(320, 240))

            #add label
            camera_name = QtWidgets.QLabel(self)
            camera_name.setText("Name")

            vert_layout.addWidget(combo_box)
            vert_layout.addWidget(camera_display)
            camera_name.setMaximumHeight(20)
            vert_layout.addWidget(camera_name)
            
            self.camera_views.append((combo_box, camera_display, camera_name))
            row = i//(num_views/2)
            col = i % (num_views/2) + 1
            self.gridLayout.addItem(vert_layout,row, col)


        self.setGeometry(104, 241, 1687, 675)
        self.setWindowTitle('Perception Status')


        #start timer that updates views
        self.timer_on = False
        # QtCore.QTimer.singleShot(2000, self.update_views)


    def map_cameras_to_views(self):
        '''
        Maps cameras to their views in the GUI
        '''
        for i, sn in enumerate(self.perception_obj.camera_variables.keys()):
            #add it to the list
            self.cams2views.append(sn)
            self.camera_views[i][2].setText(sn)
            self.camera_views[i][2].setAlignment(QtCore.Qt.AlignCenter)
            if self.perception_obj.camera_variables[sn].connected:
                #set the label to be green
                self.camera_views[i][2].setStyleSheet('QLabel {background-color: green; color: black;}')
            else:
                self.camera_views[i][2].setStyleSheet('QLabel {background-color: red; color: white;}')

    def run_perception_pipeline(self):
        #run all methods in perce
        self.perception_obj.acquire_images()
        self.perception_obj.segment_objects()
        #self.perception_obj.infer_objects()
        self.perception_obj.combine_objects()
        self.perception_obj.compute_xform_of_objects()

        #update views
        self.change_display_images()
        self.update_world_view()

    #updates the pictures for all the cameras 
    def change_display_images(self):

        #loop through all of the camera views 
        #and update the image based on what the combo box says
        for i, sn in enumerate(self.cams2views):
            cam_img = np.zeros((480,640,3),dtype='uint8')
            image = QtGui.QImage(cam_img, cam_img.shape[1], cam_img.shape[0], cam_img.shape[1] * 3,QtGui.QImage.Format_RGB888)
            if self.camera_views[i][0].currentIndex() == 0:
                #color
                cam_img = self.perception_obj.camera_variables[sn].full_color_image
            elif self.camera_views[i][0].currentIndex() == 1:
                #make rgb image from depth image
                cam_img = self.make_rgb_from_depth(self.perception_obj.camera_variables[sn].depth_image)
            else:
                #annotated
                cam_img = self.perception_obj.camera_variables[sn].segmented_image
        
       
            image = QtGui.QImage(cam_img, cam_img.shape[1], cam_img.shape[0], cam_img.shape[1] * 3,QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap(image)
            self.camera_views[i][1].setPixmap(pix.scaled(320, 240))


    def make_rgb_from_depth(self, depth):
        '''
        Makes an RGB image from a depth image using the intel realsense example 
        'make_depth_histogram' in example.hpp
        '''
        if depth is None:
            rgb = np.zeros((480, 640, 3),dtype='uint8')
            return 
        rgb = np.zeros((depth.shape[0], depth.shape[1], 3),dtype='uint8')
        histogram, _ = np.histogram(depth, 65536)
        for i in range(2,65536):
            histogram[i] += histogram[i-1]
        for row in range(depth.shape[0]):
            for col in range(depth.shape[1]):
                d = depth[row, col]
                if d != 0:
                    f =  255 * (histogram[d] / histogram[65535])
                    rgb[row, col, 0] = 255 - f
                    rgb[row, col, 1] = 0
                    rgb[row, col, 2] = f
                else:
                    rgb[row, col, 0] = 20
                    rgb[row, col, 1] = 5
                    rgb[row, col, 2] = 0
        return rgb


    def update_world_view(self):
        #update 3d world
        points, colors = self.perception_obj.create_world_view()
        self.glWidget.point_cloud = points
        self.glWidget.colors = colors
        self.glWidget.update()



class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None, pc=np.array([]), c=np.array([])):
        super(GLWidget, self).__init__(parent)
        self.point_cloud = pc
        self.colors = c
        self.lastX = 0
        self.lastY = 0
        self.button = None
        self.phi = -0.25*math.pi
        self.theta = math.pi/2
        self.radius = .5
        self.translateX = 0
        self.translateY = 0
        self.translateZ = -0.05

    def initializeGL(self):
        GL.glClearColor(0,0,0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

    def minimumSizeHint(self):
        return QtCore.QSize(300, 300)

    def paintGL(self):
        GL.glClearColor(0, 0, 0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

        GL.glViewport(0, 0, self.width(), self.height())
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GLU.gluPerspective(60, self.width()/self.height(), 0.1, 20)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        #rotates around the y axis
        GLU.gluLookAt(self.radius * math.cos(self.theta) * math.sin(self.phi) + self.translateX,
			  self.radius * math.sin(self.theta) * math.sin(self.phi)    + self.translateY,
			  self.radius * math.cos(self.phi) + self.translateZ,
			  self.translateX, self.translateY, self.translateZ,
			  0, 0, 1)


        if self.point_cloud.size > 0:
            GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
            GL.glEnableClientState(GL.GL_COLOR_ARRAY)
            GL.glVertexPointer(3, GL.GL_FLOAT, 0, self.point_cloud.flatten())
            GL.glColorPointer(3, GL.GL_UNSIGNED_BYTE, 0, self.colors.flatten())
            GL.glDrawArrays(GL.GL_POINTS, 0, self.colors.shape[0])
            GL.glPointSize(1.0)
            GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
            GL.glDisableClientState(GL.GL_COLOR_ARRAY)

    def mousePressEvent(self, event):
        
        self.lastX = event.pos().x()
        self.lastY = event.pos().y()
        self.button = event.buttons()

    def mouseDoubleClickEvent(self, event):
        self.phi = -0.25*math.pi
        self.theta = math.pi/2
        self.radius = .5
        self.translateX = 0
        self.translateY = 0
        self.translateZ = -0.05

    def mouseMoveEvent(self,event):

        dx = event.pos().x() - self.lastX
        dy = event.pos().y() - self.lastY

        if self.button == QtCore.Qt.LeftButton:
            self.theta += -dx/200
            self.phi += dy/200  
            if self.phi < -math.pi:
                self.phi = -math.pi-0.01
            if self.phi >= 0:
                self.phi = -0.001
            self.update()
        elif self.button == QtCore.Qt.RightButton:
            self.translateX -= dx*0.001
            self.translateY -= dy*0.001
            self.update()

        self.lastX = event.pos().x()
        self.lastY = event.pos().y()

    def wheelEvent(self, event):
        # if event.delta() < 0 :#towards user
        #     self.radius -= 0.1
        # else:
        #     self.radius += 0.1
	    

        self.update()


def main():
    app = QtWidgets.QApplication(sys.argv)

    #set up the window
    pm = PerceptionGUI()
    pm.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()