#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import rospy
import roslaunch
import ast
from ardrone_manager.srv import *

def manager_treeWidget(window):
	rospy.wait_for_service('/ardrone_manager/get_list')
	myService = rospy.ServiceProxy('/ardrone_manager/get_list', GetList)	
	myMessage = myService()
	print(myMessage.list)
	try:
		myData = ast.literal_eval(myMessage.list)
		i = 0
		for myGroup in sorted(myData.keys()):
			item = QtWidgets.QTreeWidgetItem(window.treeWidget)
			item.setText(0, myGroup)
			j = 0
			for myDrone in sorted(myData[myGroup].keys()):		
				item2 = QtWidgets.QTreeWidgetItem(item)
				item2.setText(0, myDrone)	
				ip = myData[myGroup][myDrone]['ip']
				isConnected = myData[myGroup][myDrone]['isConnected']
				j += 1
			i += 1
				
	except Exception as error:
		print("Something went wrong when acquiring the groups and drones")
		print(error)
	
	
	

class Ui_MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.setupUi()

    def setupUi(self):
        self.setObjectName("MainWindow")
	self.setWindowTitle("Drone Manager")
        self.resize(1280, 720)
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.treeWidget = QtWidgets.QTreeWidget(self.centralwidget)
        self.treeWidget.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.treeWidget.sizePolicy().hasHeightForWidth())
        self.treeWidget.setSizePolicy(sizePolicy)
        self.treeWidget.setObjectName("treeWidget")
        self.treeWidget.headerItem().setText(0, "Groups and Drones")
        self.horizontalLayout.addWidget(self.treeWidget)
        self.widget = QtWidgets.QWidget(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget.sizePolicy().hasHeightForWidth())
        self.widget.setSizePolicy(sizePolicy)
        self.widget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.widget.setObjectName("widget")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 933, 21))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate




if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	ex = Ui_MainWindow()
	ex.show()
	manager_treeWidget(ex)
	sys.exit(app.exec_())


