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
		print("Something went wrong when acquiring the groups and drones:")
		print(error)

class AddGroupPopup(QtWidgets.QWidget):
	def __init__(self):
		QtWidgets.QWidget.__init__(self)
		self.setup()
	
	def setup(self):
		self.setObjectName("Form")
		self.resize(346, 52)
		self.setWindowTitle("Name your new group")
		self.horizontalLayout = QtWidgets.QHBoxLayout(self)
		self.horizontalLayout.setObjectName("horizontalLayout")
		self.label = QtWidgets.QLabel(self)
		self.label.setObjectName("label")
		self.label.setText("Group Name:")
		self.horizontalLayout.addWidget(self.label)
		self.lineEdit = QtWidgets.QLineEdit(self)
		self.lineEdit.setObjectName("lineEdit")
		self.horizontalLayout.addWidget(self.lineEdit)
		self.pushButton = QtWidgets.QPushButton(self)
		self.pushButton.setObjectName("pushButton")
		self.pushButton.clicked.connect(self.on_ok_clicked)
		self.pushButton.setText("OK")
		self.horizontalLayout.addWidget(self.pushButton)
	
	def on_ok_clicked(self):
		add_group(self.lineEdit.text())
		self.close()

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

		menuAdd = self.menuBar().addMenu("Add")
		actionGroup = menuAdd.addAction("Group", self.addGroupHandler)
		actionDrone = menuAdd.addAction("Drone", self.addDroneHandler)
	        #QtCore.QMetaObject.connectSlotsByName(self)

		self.treeWidget.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
		self.treeWidget.customContextMenuRequested.connect(self.treeWidgetContextMenuHandler)	

	def addGroupHandler(self):
		print("i wanna add a group")
		self.popup = AddGroupPopup()
		self.popup.show()

	def addDroneHandler(self):
		print("i wanna add a drone")
	
	def treeWidgetContextMenuHandler(self, pos):
		globalPos = self.mapToGlobal(pos)
		item = self.treeWidget.itemAt(pos)
		if item != None:
			menu = QtWidgets.QMenu()
			if item.parent() == None:
				action_add_drone = menu.addAction("Add Drone")
			action_remove = menu.addAction("Remove")
			action_triggered = menu.exec_(globalPos)	
			if action_triggered == action_remove:
				if item.parent() != None:
					remove_drone(item.text(0))
				else:
					remove_group(item.text(0))	
			try:
				if action_triggered == action_add_drone:
					add_drone()
			except:
				pass

def add_drone():
	print("i want to add a new drone")

def add_group(name):
	print("adding new group: %s" %name)

def remove_drone(name):
	print("removing drone:")
	print(name)


def remove_group(name):
	print("removing group:")
	print(name)


if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	ex = Ui_MainWindow()
	ex.show()
	manager_treeWidget(ex)
	sys.exit(app.exec_())


