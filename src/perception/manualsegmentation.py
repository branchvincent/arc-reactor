from PyQt5 import QtCore, QtWidgets, QtGui, QtOpenGL
import numpy as np
import sys
import os
import logging
import cv2
import json
logger = logging.getLogger()
class ManualSegmentationGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(ManualSegmentationGUI, self).__init__()
        
        #load in the json file 
        with open('binrandom.txt', 'r') as f:
            x = json.load(f)
        self.jsonfile = x
        self.image_num = -1
        self.bin = 0            #bin is either left 0 or right 1

        self.initUI()
        self.show()

    def initUI(self):
        self.vertical_layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.vertical_layout)
        
        #top
        self.horz_top = QtWidgets.QHBoxLayout()
        self.load_button = QtWidgets.QPushButton(self)
        self.load_button.setText("Load")
        self.load_button.pressed.connect(self.load_file)
        self.save_button = QtWidgets.QPushButton(self)
        self.save_button.setText("Save")
        self.save_button.pressed.connect(self.save_file)
        self.current_image_name = QtWidgets.QLabel(self)
        self.left_radio = QtWidgets.QRadioButton(self)
        self.left_radio.setText("Left/Mid")
        self.left_radio.toggled.connect(self.bin_changedL)
        self.right_radio = QtWidgets.QRadioButton(self)
        self.right_radio.setText("Mid/Right")
        self.right_radio.toggled.connect(self.bin_changedR)
        self.current_image_name.setText("No image loaded")
        self.horz_top.addStretch(1)
        self.horz_top.addWidget(self.load_button)
        self.horz_top.addSpacing(20)
        self.horz_top.addWidget(self.save_button)
        self.horz_top.addSpacing(20)
        self.horz_top.addWidget(self.left_radio)
        self.horz_top.addWidget(self.right_radio)
        self.horz_top.addWidget(self.current_image_name)
        self.horz_top.addStretch(1)
        self.load_button.show()
        self.save_button.show()
        self.current_image_name.show()
        #set to left bin
        self.left_radio.setChecked(True)

        self.setGeometry(200,200,800,600)

        #bot
        self.horz_bot = QtWidgets.QHBoxLayout()
        #right half is just the image
        self.image_visualization = ImageGLWidget(self)

        #left half has 22 buttons in a column for each of the objects
        self.left_vert_widget = QtWidgets.QVBoxLayout()
        self.left_vert_widget.addStretch(1)
        self.buttons = []
        for i in range(22):
            self.buttons.append(QtWidgets.QRadioButton(self))
            self.buttons[i].setMaximumWidth(200)
            self.buttons[i].setText("Item " + str(i))
            self.buttons[i].pressed.connect(self.button_pressed)
            self.left_vert_widget.addWidget(self.buttons[i])
            self.left_vert_widget.addSpacing(10)
            self.buttons[i].show()


        self.left_vert_widget.addStretch(1)

        self.horz_bot.addItem(self.left_vert_widget)
        self.horz_bot.addSpacing(30)
        self.horz_bot.addWidget(self.image_visualization)

        self.vertical_layout.addItem(self.horz_top)
        self.vertical_layout.addItem(self.horz_bot)

    def load_file(self):
        fname = QtWidgets.QFileDialog.getOpenFileName(self, 'Open Image', "","*.npy" )
        if fname[0] != "":
            image = np.load(fname[0])
            #what directory is this image in will tell us what json entry to use
            last_slash = fname[0].rfind('/',0, len(fname[0]))
            second_last_slash = fname[0].rfind('/',0, last_slash)
            directory = fname[0][second_last_slash+1:last_slash]
            self.image_num = int(directory)
            self.image_visualization.set_image(image)
            self.current_image_name.setText(fname[0])
            self.update_button_labels()


    def update_button_labels(self):
        #which data set are we looking at?
        try:
            dict = self.jsonfile["config_" + str(self.image_num)]
        except:
            return
        #which side? left or right?
        if self.bin == 0:
            cnt = 0
            for string in dict['bin1']:
                self.buttons[cnt].setText(string)
                cnt += 1
            for string in dict['bin2']:
                self.buttons[cnt].setText(string)
                cnt += 1
        else:
            cnt = 0
            for string in dict['bin2']:
                self.buttons[cnt].setText(string)
                cnt += 1
            for string in dict['bin3']:
                self.buttons[cnt].setText(string)
                cnt += 1
    def save_file(self):
        fname = QtWidgets.QFileDialog.getSaveFileName(self, 'Save segmentation', "","*.npy" )
        if fname[0] != "":
            seg_data = np.array(self.image_visualization.segmented_points)
            np.save(fname[0],seg_data)
            print("Segmentation data saved to {}".format(fname[0]))

    def button_pressed(self):
        #which button was pressed?
        num = self.buttons.index(self.sender())
        self.image_visualization.segment_index = num

    def bin_changedL(self, ison):
        if ison:
            self.bin = 0
            self.update_button_labels()

    def bin_changedR(self, ison):
        if ison:
            self.bin = 1
            self.update_button_labels()

    def keyPressEvent(self, event):
        modifiers = QtWidgets.QApplication.keyboardModifiers()
        if modifiers == QtCore.Qt.ControlModifier and event.key() == 90:
            self.image_visualization.undo()


class ImageGLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        super(ImageGLWidget, self).__init__(parent)
        self.image = QtGui.QImage()
        self.gold_copy = np.zeros((480,640))
        self.numpy_display_image = np.zeros((480, 640))
        self.show()
        self.segmented_points = []          #list of segmented points
        for i in range(22):
            self.segmented_points.append([])
        self.segment_index = -1             #which object is being segmented

        self.undo_list = []
        self.redo_list = []
        self.mask_points_removed = np.zeros((480,640))

        self.brushW = 5

        self.lastX = 0
        self.lastY = 0

    def set_image(self, image):
        self.gold_copy = image.copy()
        self.numpy_display_image = image.copy()
        self.undo_list = []
        self.redo_list = []
        self.mask_points_removed = np.zeros((480,640))
        self.segmented_points = []
        for i in range(22):
            self.segmented_points.append([])
        self.update()

    def initializeGL(self):
        pass

    def minimumSizeHint(self):
        return QtCore.QSize(640, 480)

    def maximumSizeHint(self):
        return QtCore.QSize(640, 480)

    def paintGL(self):
        pass

    def paintEvent(self, paintevent):

        self.image = QtGui.QImage(self.numpy_display_image, self.numpy_display_image.shape[1], self.numpy_display_image.shape[0], self.numpy_display_image.shape[1] * 3,QtGui.QImage.Format_RGB888)
        painter = QtGui.QPainter(self)
        brush = QtGui.QBrush(QtCore.Qt.black)
        painter.setBackground(brush)
        painter.eraseRect(0,0,self.width(), self.height())
        painter.drawImage(0,0, self.image)

    def mousePressEvent(self, event):
        self.button = event.buttons()
        self.undo_list = []
        self.redo_list = []
        self.lastX = event.pos().x()
        self.lastY = event.pos().y()

    def mouseDoubleClickEvent(self, event):
        #flood fill
        x = event.pos().x()
        y = event.pos().y()
        im = self.mask_points_removed.copy().astype('uint8')
        mask = np.zeros((482,642)).astype('uint8')
        retval, image, mask, rect= cv2.floodFill(im, mask, (x,y), 1)

        #what new points were added?
        ind = np.nonzero(np.logical_and(self.mask_points_removed == 0, image == 1))
        self.undo_list = []
        #add these points to the undo list and to the current segmentation
        for n in range(ind[0].shape[0]):
            ypt = ind[0][n]
            xpt = ind[1][n]
            self.undo_list.append((xpt,ypt))
            if self.segment_index >= 0:
                self.segmented_points[self.segment_index].append((xpt,ypt))
                self.mask_points_removed[ypt][xpt] = 1
                self.numpy_display_image[ypt][xpt][0] = 0
                self.numpy_display_image[ypt][xpt][1] = 0
                self.numpy_display_image[ypt][xpt][2] = 0
        self.button == QtCore.Qt.LeftButton
        self.update()

    def mouseMoveEvent(self,event):
        x = event.pos().x()
        y = event.pos().y()
        #remove points
        if self.button == QtCore.Qt.LeftButton:    
            points = self.draw_line(self.lastX, self.lastY, x, y)
            points_large = []
            for p in points:
                for n in range(p[0]-self.brushW//2,p[0]+self.brushW//2):
                    for m in range(p[1]-self.brushW//2,p[1]+self.brushW//2):
                        if n >= 0 and n < 640 and m >= 0 and m < 480:
                            points_large.append((n,m))

            for p in points_large:
                #add this point to the mask_points_removed
                if self.mask_points_removed[p[1]][p[0]] == 0 and self.segment_index >= 0:
                    self.mask_points_removed[p[1]][p[0]] = 1
                    #add this point to the current list of points for particular object
                    self.segmented_points[self.segment_index].append((p[0],p[1]))
                    #add to the undo list
                    self.undo_list.append((p[0],p[1]))
                    #remove the point from the image
                    self.numpy_display_image[p[1]][p[0]] = 0
            self.update()


        #add points back in
        elif self.button == QtCore.Qt.RightButton:
            points = []
            for n in range(x-self.brushW//2,x+self.brushW//2):
                for m in range(y-self.brushW//2,y+self.brushW//2):
                    if n >= 0 and n < 640 and m >= 0 and m < 480:
                        points.append((n,m))
            
            for p in points:
                #add this point to the mask_points_removed
                if self.mask_points_removed[p[1]][p[0]] == 1 and self.segment_index >= 0:
                    self.mask_points_removed[p[1]][p[0]] = 0
                    #add this point to the current list of points for particular object
                    try:
                        self.segmented_points[self.segment_index].remove((p[0],p[1]))
                    except:
                        pass
                    #add to the undo list
                    self.undo_list.append((p[0],p[1]))
                    #remove the point from the image
                    self.numpy_display_image[p[1]][p[0]] = self.gold_copy[p[1]][p[0]]
        
            self.update()

        self.lastX = event.pos().x()
        self.lastY = event.pos().y()

    def draw_line(self, x1, y1, x2, y2):
        dx = x2-x1
        dy = y2-y1

        if abs(dx) > abs(dy):
            steps = int(abs(dx))
        else:
            steps = int(abs(dy))

        xinc = dx / float(steps)
        yinc = dy / float(steps)
        x = x1
        y = y1
        points = []
        for i in range(steps):
            x = x + xinc
            y = y + yinc
            points.append([round(x),round(y)])

        return points

    def undo(self):
        #remove all the points in the undo list and put then in the redo list
        #also add those points back to the image (or remove them)
        for p in self.undo_list:
            if self.button == QtCore.Qt.LeftButton:
                #was removing add them back
                self.mask_points_removed[p[1]][p[0]] = 0
                try:
                    self.segmented_points[self.segment_index].remove((p[0],p[1]))
                except:
                    pass
                self.numpy_display_image[p[1]][p[0]] = self.gold_copy[p[1]][p[0]]

            # elif self.button == QtCore.Qt.RightButton:
            #     #was adding remove them
            #     self.mask_points_removed[p[1]][p[0]] = 1
            #     self.segmented_points[self.segment_index].append((p[0],p[1]))
            #     self.numpy_display_image[p[1]][p[0]] = 0
        self.redo_list = self.undo_list
        self.undo_list = []
        self.update()
    def redo(self):
        pass

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    #set up the window
    m3 = ManualSegmentationGUI()
    sys.exit(app.exec_())