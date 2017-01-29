from PyQt4 import QtGui, QtOpenGL, QtCore
import sys
from OpenGL import GL, GLU
from status import CameraStatus
from functools import partial
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np

class PerceptionMonitor(QtGui.QWidget):
    
    def __init__(self):
        super(PerceptionMonitor, self).__init__()
        self.cam_status = CameraStatus()
        self.cam_indicators = {}    #dictionary of sn to Qbuttons
        self.initUI()
        
    def initUI(self):
        
        #add a vertical layout
        self.vertLayout = QtGui.QVBoxLayout()
        self.setLayout(self.vertLayout)

        #add horizontal layouts
        self.horzTop = QtGui.QHBoxLayout()
        self.horzBot = QtGui.QHBoxLayout()

        #one for the top and one for the bottom
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzBot)

        #add a plotter to the bottom left 
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumSize(300,300)
        self.canvas.setParent(self)
        axes = self.figure.add_subplot(111)
        axes.imshow(np.zeros((480,640,3),dtype='uint8'))
        self.horzBot.addWidget(self.canvas)

        #add gl widget to the bottom right
        self.glWidget = GLWidget(self)
        self.horzBot.addWidget(self.glWidget)

        #add spacer to horzTop
        self.horzTop.addStretch(1)

        self.setGeometry(800, 600, 600, 200)
        self.setWindowTitle('Perception Status')

        self.show()

    #updated the list of connected cams and their buttons
    def update_connected_cams(self):
        
        #ask which cameras are connected 
        self.cam_status.poll()

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
                self.horzTop.addWidget(btn)

                #add the button to the dictionary
                self.cam_indicators[key] = btn

            else:
                #update its status
                btn = self.cam_indicators[key]
                if value:
                    btn.setStyleSheet('QPushButton {background-color: green; color: black;}')
                else:
                    btn.setStyleSheet('QPushButton {background-color: red; color: black;}')


    #updates the pictures for all the cameras 
    def update_views(self):
        self.cam_status.poll()


    #updates the 3D world view
    def update_world_view(self):
        self.cam_status.poll()
        points, colors = self.cam_status.create_current_world_view()
        self.glWidget.point_cloud = points
        self.glWidget.colors = colors
        self.glWidget.update()

    #change the view of the picture to the button that was clicked
    def change_view(self, btn):
        #show the image from the serial number btn
        image = self.cam_status.cameraColorImages[btn]

        self.figure.clear()
        axes = self.figure.add_subplot(111)
        axes.imshow(image)
        self.canvas.draw()
        self.update_world_view()


class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None, pc=None, c=None):
        super(GLWidget, self).__init__(parent)
        self.point_cloud = pc
        self.colors = c

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
        GLU.gluPerspective(40, self.width()/self.height(), 0.1, 1000)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

        if not self.point_cloud is None:
            GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
            GL.glEnableClientState(GL.GL_COLOR_ARRAY)
            #point cloud size and colors size are not the same
            GL.glVertexPointer(3, GL.GL_FLOAT, 0, self.point_cloud)
            GL.glColorPointer(3, GL.GL_UNSIGNED_BYTE, 0, self.colors)
            GL.glDrawArrays(GL.GL_POINTS, 0, self.colors.size)
            GL.glPointSize(1.0)
            GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
            GL.glDisableClientState(GL.GL_COLOR_ARRAY)

def main():
    
    app = QtGui.QApplication(sys.argv)

    #set up the window
    pm = PerceptionMonitor()
    pm.update_connected_cams()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()