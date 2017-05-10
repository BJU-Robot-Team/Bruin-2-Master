# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/bruin2/bruin_2_code/src/nodes/master/master_gui/resource/gostopscreen.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(400, 300)
        self.lbl_black = QtWidgets.QLabel(Form)
        self.lbl_black.setGeometry(QtCore.QRect(-30, -20, 461, 341))
        self.lbl_black.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.lbl_black.setText("")
        self.lbl_black.setScaledContents(True)
        self.lbl_black.setObjectName("lbl_black")
        self.lbl_directiondata = QtWidgets.QLabel(Form)
        self.lbl_directiondata.setGeometry(QtCore.QRect(220, 180, 121, 31))
        self.lbl_directiondata.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_directiondata.setObjectName("lbl_directiondata")
        self.lbl_latittudedata = QtWidgets.QLabel(Form)
        self.lbl_latittudedata.setGeometry(QtCore.QRect(220, 120, 121, 31))
        self.lbl_latittudedata.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_latittudedata.setObjectName("lbl_latittudedata")
        self.lbl_direction = QtWidgets.QLabel(Form)
        self.lbl_direction.setGeometry(QtCore.QRect(50, 180, 151, 31))
        self.lbl_direction.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_direction.setObjectName("lbl_direction")
        self.lbl_longitudedata = QtWidgets.QLabel(Form)
        self.lbl_longitudedata.setGeometry(QtCore.QRect(220, 150, 121, 31))
        self.lbl_longitudedata.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_longitudedata.setObjectName("lbl_longitudedata")
        self.lbl_longitude = QtWidgets.QLabel(Form)
        self.lbl_longitude.setGeometry(QtCore.QRect(50, 150, 151, 31))
        self.lbl_longitude.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_longitude.setObjectName("lbl_longitude")
        self.lbl_lattitude = QtWidgets.QLabel(Form)
        self.lbl_lattitude.setGeometry(QtCore.QRect(50, 120, 151, 31))
        self.lbl_lattitude.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_lattitude.setObjectName("lbl_lattitude")
        self.lbl_description = QtWidgets.QLabel(Form)
        self.lbl_description.setGeometry(QtCore.QRect(50, -10, 281, 61))
        self.lbl_description.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_description.setObjectName("lbl_description")
        self.btn_go = QtWidgets.QPushButton(Form)
        self.btn_go.setGeometry(QtCore.QRect(100, 50, 161, 61))
        self.btn_go.setStyleSheet("background-color: rgb(85, 255, 0);")
        self.btn_go.setObjectName("btn_go")
        self.lbl_speed = QtWidgets.QLabel(Form)
        self.lbl_speed.setGeometry(QtCore.QRect(50, 210, 151, 31))
        self.lbl_speed.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_speed.setObjectName("lbl_speed")
        self.lbl_speedData = QtWidgets.QLabel(Form)
        self.lbl_speedData.setGeometry(QtCore.QRect(220, 210, 121, 31))
        self.lbl_speedData.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_speedData.setObjectName("lbl_speedData")
        self.lbl_turndirection = QtWidgets.QLabel(Form)
        self.lbl_turndirection.setGeometry(QtCore.QRect(50, 240, 161, 31))
        self.lbl_turndirection.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_turndirection.setObjectName("lbl_turndirection")
        self.lbl_turnDirectiondata = QtWidgets.QLabel(Form)
        self.lbl_turnDirectiondata.setGeometry(QtCore.QRect(220, 240, 121, 31))
        self.lbl_turnDirectiondata.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_turnDirectiondata.setObjectName("lbl_turnDirectiondata")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.lbl_directiondata.setText(_translate("Form", "0"))
        self.lbl_latittudedata.setText(_translate("Form", "0"))
        self.lbl_direction.setText(_translate("Form", "Current Heading:"))
        self.lbl_longitudedata.setText(_translate("Form", "0"))
        self.lbl_longitude.setText(_translate("Form", "Current Longitude:"))
        self.lbl_lattitude.setText(_translate("Form", "Current Lattitude:"))
        self.lbl_description.setText(_translate("Form", "Displaying Bruin 2 State information:"))
        self.btn_go.setText(_translate("Form", "Are you sure you \n"
"want to go here?"))
        self.lbl_speed.setText(_translate("Form", "Current Speed:"))
        self.lbl_speedData.setText(_translate("Form", "0"))
        self.lbl_turndirection.setText(_translate("Form", "Current Turn Direction:"))
        self.lbl_turnDirectiondata.setText(_translate("Form", "Neither"))

