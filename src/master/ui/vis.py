# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'vis.ui'
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

class Ui_VisWindow(object):
    def setupUi(self, VisWindow):
        VisWindow.setObjectName(_fromUtf8("VisWindow"))
        VisWindow.resize(735, 491)
        self.centralwidget = QtGui.QWidget(VisWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_2.setMargin(0)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.view = QtGLWindow(parent=self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        # sizePolicy.setHeightForWidth(self.view.sizePolicy().hasHeightForWidth())
        self.view.setSizePolicy(sizePolicy)
        self.view.setObjectName(_fromUtf8("view"))
        self.horizontalLayout_2.addWidget(self.view)
        self.scrollArea = QtGui.QScrollArea(self.centralwidget)
        self.scrollArea.setMinimumSize(QtCore.QSize(150, 0))
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName(_fromUtf8("scrollArea"))
        self.scrollAreaWidgetContents = QtGui.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 148, 489))
        self.scrollAreaWidgetContents.setObjectName(_fromUtf8("scrollAreaWidgetContents"))
        self.verticalLayout = QtGui.QVBoxLayout(self.scrollAreaWidgetContents)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.label = QtGui.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout.addWidget(self.label)
        self.pc_area = QtGui.QWidget(self.scrollAreaWidgetContents)
        self.pc_area.setObjectName(_fromUtf8("pc_area"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.pc_area)
        self.verticalLayout_2.setMargin(0)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.verticalLayout.addWidget(self.pc_area)
        self.label_7 = QtGui.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.verticalLayout.addWidget(self.label_7)
        self.bb_area = QtGui.QWidget(self.scrollAreaWidgetContents)
        self.bb_area.setObjectName(_fromUtf8("bb_area"))
        self.verticalLayout_8 = QtGui.QVBoxLayout(self.bb_area)
        self.verticalLayout_8.setMargin(0)
        self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
        self.verticalLayout.addWidget(self.bb_area)
        self.label_2 = QtGui.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.verticalLayout.addWidget(self.label_2)
        self.pose_area = QtGui.QWidget(self.scrollAreaWidgetContents)
        self.pose_area.setObjectName(_fromUtf8("pose_area"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.pose_area)
        self.verticalLayout_3.setMargin(0)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.verticalLayout.addWidget(self.pose_area)
        self.label_3 = QtGui.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout.addWidget(self.label_3)
        self.trace_area = QtGui.QWidget(self.scrollAreaWidgetContents)
        self.trace_area.setObjectName(_fromUtf8("trace_area"))
        self.verticalLayout_7 = QtGui.QVBoxLayout(self.trace_area)
        self.verticalLayout_7.setMargin(0)
        self.verticalLayout_7.setObjectName(_fromUtf8("verticalLayout_7"))
        self.verticalLayout.addWidget(self.trace_area)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.horizontalLayout_2.addWidget(self.scrollArea)
        VisWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(VisWindow)
        QtCore.QMetaObject.connectSlotsByName(VisWindow)

    def retranslateUi(self, VisWindow):
        VisWindow.setWindowTitle(_translate("VisWindow", "MainWindow", None))
        self.label.setText(_translate("VisWindow", "Point Cloud", None))
        self.label_7.setText(_translate("VisWindow", "Bounding Box", None))
        self.label_2.setText(_translate("VisWindow", "Pose", None))
        self.label_3.setText(_translate("VisWindow", "Trace", None))

from klampt.vis.qtbackend import QtGLWindow
