# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'grasp.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_GraspWindow(object):
    def setupUi(self, GraspWindow):
        GraspWindow.setObjectName(_fromUtf8("GraspWindow"))
        GraspWindow.resize(902, 665)
        self.centralwidget = QtGui.QWidget(GraspWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.view = QtGLWindow(parent=self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        #sizePolicy.setHeightForWidth(self.view.sizePolicy().hasHeightForWidth())
        self.view.setSizePolicy(sizePolicy)
        self.view.setObjectName(_fromUtf8("view"))
        self.verticalLayout.addWidget(self.view)
        self.scrollArea = QtGui.QScrollArea(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scrollArea.sizePolicy().hasHeightForWidth())
        self.scrollArea.setSizePolicy(sizePolicy)
        self.scrollArea.setMinimumSize(QtCore.QSize(0, 200))
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName(_fromUtf8("scrollArea"))
        self.grasp_panel = QtGui.QWidget()
        self.grasp_panel.setGeometry(QtCore.QRect(0, 0, 900, 198))
        self.grasp_panel.setObjectName(_fromUtf8("grasp_panel"))
        self.grasp_panel_layout = QtGui.QGridLayout(self.grasp_panel)
        self.grasp_panel_layout.setObjectName(_fromUtf8("grasp_panel_layout"))
        self.scrollArea.setWidget(self.grasp_panel)
        self.verticalLayout.addWidget(self.scrollArea)
        GraspWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(GraspWindow)
        QtCore.QMetaObject.connectSlotsByName(GraspWindow)

    def retranslateUi(self, GraspWindow):
        GraspWindow.setWindowTitle(_translate("GraspWindow", "MainWindow", None))

from klampt.vis.qtbackend import QtGLWindow
