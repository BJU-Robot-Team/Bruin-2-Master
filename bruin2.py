# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/bruin2/bruin_2_code/src/nodes/master/master_gui/resource/bruin2.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.lbl_black = QtWidgets.QLabel(self.centralwidget)
        self.lbl_black.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.lbl_black.setText("")
        self.lbl_black.setScaledContents(True)
        self.lbl_black.setObjectName("lbl_black")
        self.gridLayout_2.addWidget(self.lbl_black, 0, 0, 1, 1)
        self.lbl_Title = QtWidgets.QLabel(self.centralwidget)
        self.lbl_Title.setGeometry(QtCore.QRect(320, 0, 171, 51))
        self.lbl_Title.setStyleSheet("color: rgb(255, 255, 255);\n"
"font: 75 9pt \"Courier 10 Pitch\";\n"
"font: 14pt \"Sans Serif\";")
        self.lbl_Title.setScaledContents(False)
        self.lbl_Title.setObjectName("lbl_Title")
        self.cB_WayPointNum = QtWidgets.QComboBox(self.centralwidget)
        self.cB_WayPointNum.setGeometry(QtCore.QRect(10, 10, 111, 22))
        self.cB_WayPointNum.setObjectName("cB_WayPointNum")
        self.cB_WayPointNum.addItem("")
        self.cB_WayPointNum.addItem("")
        self.cB_WayPointNum.addItem("")
        self.lbl_lattitude = QtWidgets.QLabel(self.centralwidget)
        self.lbl_lattitude.setGeometry(QtCore.QRect(150, 10, 61, 21))
        self.lbl_lattitude.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_lattitude.setObjectName("lbl_lattitude")
        self.lbl_longitude = QtWidgets.QLabel(self.centralwidget)
        self.lbl_longitude.setGeometry(QtCore.QRect(140, 40, 71, 21))
        self.lbl_longitude.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_longitude.setObjectName("lbl_longitude")
        self.lbl_lattitudeNum = QtWidgets.QLabel(self.centralwidget)
        self.lbl_lattitudeNum.setGeometry(QtCore.QRect(230, 40, 61, 21))
        self.lbl_lattitudeNum.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_lattitudeNum.setObjectName("lbl_lattitudeNum")
        self.lbl_longitudeNum = QtWidgets.QLabel(self.centralwidget)
        self.lbl_longitudeNum.setGeometry(QtCore.QRect(230, 10, 61, 21))
        self.lbl_longitudeNum.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_longitudeNum.setObjectName("lbl_longitudeNum")
        self.btn_debug = QtWidgets.QPushButton(self.centralwidget)
        self.btn_debug.setGeometry(QtCore.QRect(620, 10, 161, 31))
        self.btn_debug.setAutoFillBackground(False)
        self.btn_debug.setObjectName("btn_debug")
        self.lbl_map = QtWidgets.QLabel(self.centralwidget)
        self.lbl_map.setGeometry(QtCore.QRect(30, 70, 721, 491))
        self.lbl_map.setStyleSheet("color: rgb(255, 255, 255);")
        self.lbl_map.setObjectName("lbl_map")
        self.lbl_black.raise_()
        self.lbl_Title.raise_()
        self.cB_WayPointNum.raise_()
        self.lbl_lattitude.raise_()
        self.lbl_longitude.raise_()
        self.lbl_lattitudeNum.raise_()
        self.lbl_longitudeNum.raise_()
        self.btn_debug.raise_()
        self.lbl_map.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.lbl_Title.setText(_translate("MainWindow", "Bruin 2 Navigator"))
        self.cB_WayPointNum.setItemText(0, _translate("MainWindow", "Waypoint 1"))
        self.cB_WayPointNum.setItemText(1, _translate("MainWindow", "Waypoint 2"))
        self.cB_WayPointNum.setItemText(2, _translate("MainWindow", "Waypoint 3"))
        self.lbl_lattitude.setText(_translate("MainWindow", "Lattitude:"))
        self.lbl_longitude.setText(_translate("MainWindow", "Longitude:"))
        self.lbl_lattitudeNum.setText(_translate("MainWindow", "0"))
        self.lbl_longitudeNum.setText(_translate("MainWindow", "0"))
        self.btn_debug.setText(_translate("MainWindow", "Live Debugging Data"))
        self.lbl_map.setText(_translate("MainWindow", "Insert Photo Here"))

