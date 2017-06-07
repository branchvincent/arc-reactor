# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'motion_plan.ui'
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

class Ui_MotionPlanWindow(object):
    def setupUi(self, MotionPlanWindow):
        MotionPlanWindow.setObjectName(_fromUtf8("MotionPlanWindow"))
        MotionPlanWindow.resize(785, 665)
        self.centralwidget = QtGui.QWidget(MotionPlanWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.view = QtGLWindow(parent=self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        # sizePolicy.setHeightForWidth(self.view.sizePolicy().hasHeightForWidth())
        self.view.setSizePolicy(sizePolicy)
        self.view.setObjectName(_fromUtf8("view"))
        self.verticalLayout.addWidget(self.view)
        self.widget = QtGui.QWidget(self.centralwidget)
        self.widget.setObjectName(_fromUtf8("widget"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.widget)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.run_button = QtGui.QPushButton(self.widget)
        self.run_button.setObjectName(_fromUtf8("run_button"))
        self.horizontalLayout_2.addWidget(self.run_button)
        self.time_slider = QtGui.QSlider(self.widget)
        self.time_slider.setEnabled(False)
        self.time_slider.setMaximum(0)
        self.time_slider.setSingleStep(100)
        self.time_slider.setPageStep(1000)
        self.time_slider.setOrientation(QtCore.Qt.Horizontal)
        self.time_slider.setObjectName(_fromUtf8("time_slider"))
        self.horizontalLayout_2.addWidget(self.time_slider)
        self.time_label = QtGui.QLabel(self.widget)
        self.time_label.setObjectName(_fromUtf8("time_label"))
        self.horizontalLayout_2.addWidget(self.time_label)
        self.approve_button = QtGui.QPushButton(self.widget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.approve_button.setFont(font)
        self.approve_button.setStyleSheet(_fromUtf8("color: green;\n"
""))
        self.approve_button.setObjectName(_fromUtf8("approve_button"))
        self.approve_button.setCheckable(True)
        self.horizontalLayout_2.addWidget(self.approve_button)
        self.reject_button = QtGui.QPushButton(self.widget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.reject_button.setFont(font)
        self.reject_button.setStyleSheet(_fromUtf8("color: red;"))
        self.reject_button.setObjectName(_fromUtf8("reject_button"))
        self.reject_button.setCheckable(True)
        self.horizontalLayout_2.addWidget(self.reject_button)
        self.verticalLayout.addWidget(self.widget)
        self.checker_results = QtGui.QPlainTextEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.checker_results.sizePolicy().hasHeightForWidth())
        self.checker_results.setSizePolicy(sizePolicy)
        self.checker_results.setReadOnly(True)
        self.checker_results.setObjectName(_fromUtf8("checker_results"))
        self.verticalLayout.addWidget(self.checker_results)
        MotionPlanWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MotionPlanWindow)
        QtCore.QMetaObject.connectSlotsByName(MotionPlanWindow)

    def retranslateUi(self, MotionPlanWindow):
        MotionPlanWindow.setWindowTitle(_translate("MotionPlanWindow", "MainWindow", None))
        self.run_button.setText(_translate("MotionPlanWindow", "Pause", None))
        self.time_label.setText(_translate("MotionPlanWindow", "0s of 0s (0)", None))
        self.approve_button.setText(_translate("MotionPlanWindow", "Approve", None))
        self.reject_button.setText(_translate("MotionPlanWindow", "Reject", None))

from klampt.vis.qtbackend import QtGLWindow
