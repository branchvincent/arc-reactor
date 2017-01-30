from PyQt4 import QtGui, QtOpenGL, QtCore
import sys
from OpenGL import GL, GLU
from status import CameraStatus
from functools import partial
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import math
import numpy as np
import logging
 # start logging
logger = logging.getLogger('RS_Camera')
logging.basicConfig(filename="RS_test.log", level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%m/%d/%Y %I:%M:%S %p', filemode='w')
logging.info('Start log')

console = logging.StreamHandler()
console.setLevel(logging.INFO)
# set a format which is simpler for console use
formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
# tell the handler to use this format
console.setFormatter(formatter)

class PerceptionMonitor(QtGui.QWidget):
    
    def __init__(self):
        super(PerceptionMonitor, self).__init__()
        self.cam_status = CameraStatus()
        self.cam_indicators = {}    #dictionary of sn to Qbuttons
        self.timer = QtCore.QTimer()
        self.currentView = None
        self.initUI()

    def initUI(self):
        
        #add a vertical layout
        self.vertLayout = QtGui.QVBoxLayout()
        self.setLayout(self.vertLayout)

        #add horizontal layouts
        self.horzTop = QtGui.QHBoxLayout()
        self.horzBot = QtGui.QHBoxLayout()
        self.horzButton = QtGui.QHBoxLayout()

        #one for the top and one for the bottom
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzBot)


        #add gl widget to the bottom left
        self.glWidget = GLWidget(self)
        self.horzBot.addWidget(self.glWidget)

        #add a plotter to the bottom right 
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumSize(300,300)
        self.canvas.setParent(self)
        axes = self.figure.add_subplot(111)
        axes.imshow(np.zeros((480,640,3),dtype='uint8'))
        self.canvas.draw()
        self.horzBot.addWidget(self.canvas)

        #add spacer to horzTop
        self.horzTop.addStretch(1)
        self.horzTop.addItem(self.horzButton)
        self.horzTop.addStretch(1)

        self.setGeometry(800, 600, 600, 200)
        self.setWindowTitle('Perception Status')


        #start timer that updates views
        self.timer.timeout.connect(self.update_views)
        self.timer.start(3000)
        self.show()

    #updates the pictures for all the cameras 
    def update_views(self):
        self.cam_status.poll()
        self.update_connected_cams()
        self.update_world_view()



    #updated the list of connected cams and their buttons
    def update_connected_cams(self):
    
        # loop through the dictionary of connected cams
        # and add buttons for each
        for key, value in self.cam_status.connectedCameras.items():
            
            #if the camera doesn't have a button make one
            if not key in self.cam_indicators:
                
                #make button
                btn = QtGui.QPushButton(key, self)

                #set its background color
                btn.setStyleSheet('QPushButton {background-color: green; color: black;}')
                btn.clicked.connect(partial(self.change_view, btn.text()))
                btn.show()
                #add the button to the horizontal top layout
                self.horzButton.addWidget(btn)

                #add the button to the dictionary
                self.cam_indicators[key] = btn

                #last camera is current view
                self.currentView = key

            else:
                #update its status
                btn = self.cam_indicators[key]
                if value:
                    btn.setStyleSheet('QPushButton {background-color: green; color: black;}')
                else:
                    btn.setStyleSheet('QPushButton {background-color: red; color: black;}')


  
    #updates the 3D world view
    def update_world_view(self):
        #update 3d world
        points, colors = self.cam_status.create_current_world_view()
        self.glWidget.point_cloud = points
        self.glWidget.colors = colors
        self.glWidget.update()

        #update 2d image
        self.update_camera_view(self.currentView)

    def update_camera_view(self, serialn):
        #show the image from the serial number btn
        if not serialn is None:
            image = self.cam_status.cameraFullColorImages[serialn]
            self.figure.clear()
            axes = self.figure.add_subplot(111)
            axes.imshow(image)
            self.canvas.draw()

    #change the view of the picture to the button that was clicked
    def change_view(self, btn):
        self.currentView = btn


class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None, pc=np.array([]), c=np.array([])):
        super(GLWidget, self).__init__(parent)
        self.point_cloud = pc
        self.colors = c
        self.lastX = 0
        self.lastY = 0
        self.pitch = 0
        self.yaw = 0
        self.button = None
        self.phi = math.pi/2
        self.theta = -math.pi/2
        self.radius = .1
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
        
        GLU.gluLookAt(self.radius * math.cos(self.theta) * math.sin(self.phi) + self.translateX,
			  self.radius * math.cos(self.phi) + self.translateY,
			  self.radius * math.sin(self.theta) * math.sin(self.phi) + self.translateZ,
			  self.translateX, self.translateY, self.translateZ,
			  0, -1, 0)


        if self.point_cloud.size > 0:
            GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
            GL.glEnableClientState(GL.GL_COLOR_ARRAY)
            #point cloud size and colors size are not the same
            GL.glVertexPointer(3, GL.GL_FLOAT, 0, self.point_cloud.flatten())
            GL.glColorPointer(3, GL.GL_UNSIGNED_BYTE, 0, self.colors.flatten())
            GL.glDrawArrays(GL.GL_POINTS, 0, self.colors.shape[1])
            GL.glPointSize(1.0)
            GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
            GL.glDisableClientState(GL.GL_COLOR_ARRAY)

    def mousePressEvent(self, event):
        
        self.lastX = event.pos().x()
        self.lastY = event.pos().y()
        self.button = event.buttons()

    def mouseMoveEvent(self,event):

        dx = event.pos().x() - self.lastX
        dy = event.pos().y() - self.lastY

        if self.button == QtCore.Qt.LeftButton:
            self.theta += -dx/200
            self.phi += dy/200    
            self.update()
        elif self.button == QtCore.Qt.RightButton:
            self.translateX -= dx*0.01
            self.translateY -= dy*0.01
            self.update()

        self.lastX = event.pos().x()
        self.lastY = event.pos().y()

    def wheelEvent(self, event):
        if event.delta() < 0 :#towards user
            self.radius -= 0.1
        else:
            self.radius += 0.1
	    

        self.update()


def main():
    app = QtGui.QApplication(sys.argv)

    #set up the window
    pm = PerceptionMonitor()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()