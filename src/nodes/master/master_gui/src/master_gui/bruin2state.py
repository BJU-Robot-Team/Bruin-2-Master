# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/bruin2/bruin_2_code/src/nodes/master/master_gui/resource/bruin2state.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_DebuggingInfo(object):
    def setupUi(self, DebuggingInfo):
        DebuggingInfo.setObjectName("DebuggingInfo")
        DebuggingInfo.resize(615, 455)
        DebuggingInfo.setStyleSheet("")
        self.lbl_black = QtWidgets.QLabel(DebuggingInfo)
        self.lbl_black.setGeometry(QtCore.QRect(-10, -20, 651, 491))
        self.lbl_black.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.lbl_black.setText("")
        self.lbl_black.setScaledContents(True)
        self.lbl_black.setObjectName("lbl_black")
        self.btn_log = QtWidgets.QPushButton(DebuggingInfo)
        self.btn_log.setGeometry(QtCore.QRect(200, 20, 211, 21))
        self.btn_log.setObjectName("btn_log")
        self.scrollArea = QtWidgets.QScrollArea(DebuggingInfo)
        self.scrollArea.setGeometry(QtCore.QRect(50, 60, 501, 361))
        self.scrollArea.setAutoFillBackground(True)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.sArea_log = QtWidgets.QWidget()
        self.sArea_log.setGeometry(QtCore.QRect(0, 0, 499, 359))
        self.sArea_log.setObjectName("sArea_log")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.sArea_log)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.sArea_log)
        self.label.setText("")
        self.label.setTextFormat(QtCore.Qt.PlainText)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.scrollArea.setWidget(self.sArea_log)

        self.retranslateUi(DebuggingInfo)
        QtCore.QMetaObject.connectSlotsByName(DebuggingInfo)

    def retranslateUi(self, DebuggingInfo):
        _translate = QtCore.QCoreApplication.translate
        DebuggingInfo.setWindowTitle(_translate("DebuggingInfo", "Form"))
        self.btn_log.setText(_translate("DebuggingInfo", "Display live information"))

