from PyQt5 import QtGui, QtOpenGL, QtCore, QtWidgets
import sys
import time
from OpenGL import GL, GLU
import numpy as np
from perceptionGUI import GLWidget


class MakeModelsGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(MakeModelsGUI, self).__init__()
        self.openGLWidget = GLWidget(self)
        self.openGLWidget.setMinimumSize(640,480)

        self.initUI()
        
        self.show()

    def initUI(self):

        #3 levels
        #take image    undo image     save cloud
        #object rotation x, y, z
        #3d view  camera view
        
        self.vertLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.vertLayout)

        #add horizontal layouts
        self.horzTop = QtWidgets.QHBoxLayout()
        self.horzMid = QtWidgets.QHBoxLayout()
        self.horzBot = QtWidgets.QHBoxLayout()

        #top
        self.take_image_button = QtWidgets.QPushButton("Take image", self)
        self.remove_last_image_button = QtWidgets.QPushButton("Remove last image", self)
        self.save_button = QtWidgets.QPushButton("Save", self)
        self.take_image_button.setMaximumWidth(75)
        self.remove_last_image_button.setMaximumWidth(125)
        self.save_button.setMaximumWidth(75)
        self.take_image_button.show()
        self.remove_last_image_button.show()
        self.save_button.show()
        self.horzTop.addStretch(1)
        self.horzTop.addWidget(self.take_image_button)
        self.horzTop.addSpacing(20)
        self.horzTop.addWidget(self.remove_last_image_button)
        self.horzTop.addSpacing(20)
        self.horzTop.addWidget(self.save_button)
        self.horzTop.addStretch(1)


        #mid
        self.rotx_text = QtWidgets.QTextEdit(self)
        self.roty_text = QtWidgets.QTextEdit(self)
        self.rotz_text = QtWidgets.QTextEdit(self)
        self.rotx_text.setMaximumHeight(25)
        self.roty_text.setMaximumHeight(25)
        self.rotz_text.setMaximumHeight(25)

        self.horzMid.addStretch(1)

        label = QtWidgets.QLabel(self)
        label.setText("X rotation")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.rotx_text)
        self.horzMid.addSpacing(10)

        label = QtWidgets.QLabel(self)
        label.setText("Y rotation")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.roty_text)
        self.horzMid.addSpacing(10)
        
        label = QtWidgets.QLabel(self)
        label.setText("Z rotation")
        self.horzMid.addWidget(label)
        self.horzMid.addWidget(self.rotz_text)

        self.horzMid.addStretch(1)
        

        #bot
        self.camera_display = QtWidgets.QLabel(self)
        self.camera_display.show()
        zeroImg = np.ones((480,640,3),dtype='uint8')*255
        image = QtGui.QImage(zeroImg, zeroImg.shape[1], zeroImg.shape[0], zeroImg.shape[1] * 3,QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap(image)
        self.camera_display.setPixmap(pix)

        self.horzBot.addWidget(self.openGLWidget)
        self.horzBot.addSpacing(20)
        self.horzBot.addWidget(self.camera_display)

        #add all layouts to the vertical layout
        self.vertLayout.addItem(self.horzTop)
        self.vertLayout.addItem(self.horzMid)
        self.vertLayout.addItem(self.horzBot)



def main():
    app = QtWidgets.QApplication(sys.argv)

    #set up the window
    m3 = MakeModelsGUI()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()