#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import rospy
import roslaunch
import ast
from ardrone_manager.srv import *
from std_msgs.msg import String

def windowManager():
	rospy.Subscriber('ardrone_manager/list', String, get_list_callback)

def get_list_callback(data):
	global ex
	global myData
	rospy.wait_for_service('ardrone_manager/get_list')
	x = ex.treeWidget.takeTopLevelItem(0)
	while x != None:
		x = ex.treeWidget.takeTopLevelItem(0)
	myData = ast.literal_eval(data.data)
	for myGroup in sorted(myData.keys()):
		item = QtWidgets.QTreeWidgetItem(ex.treeWidget)
		item.setText(0, myGroup)
		for myDrone in sorted(myData[myGroup].keys()):		
			item2 = QtWidgets.QTreeWidgetItem(item)
			item2.setText(0, myDrone)	
			ip = myData[myGroup][myDrone]['ip']
			isStarted = myData[myGroup][myDrone]['isStarted']
	

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

def canIpify(string):
	try:	
		x = string
		string = string.strip(".")
		string = string.replace(" ", "")
		string = string.split(".")
		ok = True
		if len(string) == 4:
			for y in string:
				z = int(y)	
				if z < 0 or z > 255:
					ok = False
			if ok:
				if x[-1] == ".":
					return False
			return ok	
		else:
			return False
	except:
		return False

class AddDronePopup(QtWidgets.QWidget):
	def __init__(self):
		QtWidgets.QWidget.__init__(self)
		self.setup()
	
	def setup(self):
		self.setObjectName("Form")
		self.resize(346, 52)
		self.setWindowTitle("Provide the infos for your new drone")
		self.horizontalLayout = QtWidgets.QHBoxLayout(self)
        	self.horizontalLayout.setObjectName("horizontalLayout")
	        self.widget = QtWidgets.QWidget(self)
        	self.widget.setObjectName("widget")
        	self.gridLayout = QtWidgets.QGridLayout(self.widget)
        	self.gridLayout.setContentsMargins(0, 0, 0, 0)
        	self.gridLayout.setObjectName("gridLayout")
        	self.label = QtWidgets.QLabel(self.widget)
        	self.label.setObjectName("Group")
        	self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        	self.label_2 = QtWidgets.QLabel(self.widget)
        	self.label_2.setObjectName("Name")
        	self.gridLayout.addWidget(self.label_2, 0, 1, 1, 1)
        	self.label_3 = QtWidgets.QLabel(self.widget)
        	self.label_3.setObjectName("IP")
        	self.gridLayout.addWidget(self.label_3, 0, 2, 1, 1)
        	self.label_4 = QtWidgets.QLabel(self.widget)
        	self.label_4.setObjectName("Type")
        	self.gridLayout.addWidget(self.label_4, 0, 3, 1, 1)
        	self.comboBox = QtWidgets.QComboBox(self.widget)
        	self.comboBox.setObjectName("comboBox")
		i = 0
		for group in sorted(myData.keys()):
			self.comboBox.addItem("")
			self.comboBox.setItemText(i, group)
			i += 1
        	self.gridLayout.addWidget(self.comboBox, 1, 0, 1, 1)
        	self.lineEdit = QtWidgets.QLineEdit(self.widget)
        	self.lineEdit.setObjectName("lineEdit")
        	self.gridLayout.addWidget(self.lineEdit, 1, 1, 1, 1)
        	self.lineEdit_2 = QtWidgets.QLineEdit(self.widget)
        	self.lineEdit_2.setObjectName("lineEdit_2")
		self.lineEdit_2.setMinimumSize(QtCore.QSize(150, 0))
        	self.gridLayout.addWidget(self.lineEdit_2, 1, 2, 1, 1)
        	self.comboBox_2 = QtWidgets.QComboBox(self.widget)
        	self.comboBox_2.setObjectName("comboBox_2")
		self.comboBox_2.addItem("")
		self.comboBox_2.setItemText(0, "simulation")
		self.comboBox_2.addItem("")
		self.comboBox_2.setItemText(1, "drone")
        	self.gridLayout.addWidget(self.comboBox_2, 1, 3, 1, 1)
		self.horizontalLayout.addWidget(self.widget)
		self.widget_2 = QtWidgets.QWidget(self)
		self.widget_2.setObjectName("widget_2")
		self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_2)
		self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
		self.horizontalLayout_2.setObjectName("horizontalLayout_2")
		self.pushButton = QtWidgets.QPushButton(self.widget_2)
		self.pushButton.setObjectName("OK")
		self.horizontalLayout_2.addWidget(self.pushButton)
		self.horizontalLayout.addWidget(self.widget_2)
		self.label.setText("Group")
		self.label_2.setText("Name")
		self.label_3.setText("IP")
		self.label_4.setText("Type")
		self.pushButton.setText("OK")	
		self.pushButton.clicked.connect(self.on_ok_clicked)
		self.pushButton.setEnabled(False)
		self.lineEdit_2.textChanged.connect(self.on_change)

	def on_change(self):
		self.pushButton.setEnabled(canIpify(self.lineEdit_2.text()) and (self.lineEdit.text() != ""))

	def on_ok_clicked(self):
		add_drone(self.comboBox.currentText(), self.lineEdit.text(), self.lineEdit_2.text(), self.comboBox_2.currentText())
		self.close()

def init_treeWidget(treeWidget):
	global myData
	rospy.wait_for_service('ardrone_manager/get_list')
	myService = rospy.ServiceProxy('ardrone_manager/get_list', GetList)	
	myData = ast.literal_eval(myService().list)
	for myGroup in sorted(myData.keys()):
		item = QtWidgets.QTreeWidgetItem(treeWidget)
		item.setText(0, myGroup)
		for myDrone in sorted(myData[myGroup].keys()):		
			item2 = QtWidgets.QTreeWidgetItem(item)
			item2.setText(0, myDrone)	
			drone = myData[myGroup][myDrone]
			ip = drone['ip']
			isStarted = drone['isStarted']
			state = drone['state']
			droneType = drone['droneType']

class Ui_MainWindow(QtWidgets.QMainWindow):
	def __init__(self):
	        QtWidgets.QMainWindow.__init__(self)
	        self.setupUi()

	def setupUi(self):

		#main window
	        self.setObjectName("MainWindow")
		self.setWindowTitle("Drone Manager")
	        self.resize(1280, 720)
	
		#first container
	        self.centralwidget = QtWidgets.QWidget(self)
	        self.centralwidget.setObjectName("centralwidget")
	        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
	        self.horizontalLayout.setContentsMargins(4, 0, 4, 4)
	        self.horizontalLayout.setSpacing(3)
	        self.horizontalLayout.setObjectName("horizontalLayout")
		self.setCentralWidget(self.centralwidget)

		#tree widget on the left
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
		self.treeWidget.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
		self.treeWidget.customContextMenuRequested.connect(self.treeWidgetContextMenuHandler)	
		self.treeWidget.currentItemChanged.connect(self.treeWidgetSelection)
		init_treeWidget(self.treeWidget)		

		#navdata widget
		self.tableWidget = QtWidgets.QTableWidget(self.centralwidget)
		#self.tableWidget.setGeometry(QtCore.QRect(120, 190, 256, 192))
	        self.tableWidget.setObjectName("tableWidget")
        	self.tableWidget.setColumnCount(2)
	        self.tableWidget.setRowCount(1)
		self.tableWidget.setShowGrid(False)
		
		#adding headers
        	item = QtWidgets.QTableWidgetItem()
		item.setText("Parameter")
	        self.tableWidget.setHorizontalHeaderItem(0, item)
        	item = QtWidgets.QTableWidgetItem()
		item.setText("Value")
		self.tableWidget.setHorizontalHeaderItem(1, item)
		self.tableWidget.verticalHeader().setVisible(False)

		#adding items
	        item = QtWidgets.QTableWidgetItem()
		self.tableWidget.setItem(0, 0, item)
		item.setText("x")
		item.setTextAlignment(QtCore.Qt.AlignCenter)
	        item = QtWidgets.QTableWidgetItem()
	        self.tableWidget.setItem(0, 1, item)
		item.setText("y")
		item.setTextAlignment(QtCore.Qt.AlignCenter)

		self.tableWidget.resizeColumnsToContents()

		self.horizontalLayout.addWidget(self.tableWidget)
		self.tableWidget.horizontalHeader().setStretchLastSection(True)
		self.tableWidget.resizeColumnsToContents()
		
		#menu bar
		self.menuAdd = self.menuBar().addMenu("Add")
		self.actionGroup = self.menuAdd.addAction("New Group", self.addGroupHandler)
		self.actionDrone = self.menuAdd.addAction("New Drone", self.addDroneHandler)
	
		#toolbar
		self.toolBar = QtWidgets.QToolBar(self)
	        self.toolBar.setObjectName("toolBar")
	        self.toolBar.setMovable(False)
		self.toolBar.setSizePolicy(sizePolicy)
		self.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
	        self.toolBar.addAction(self.actionGroup)
	        self.toolBar.addAction(self.actionDrone)
		self.toolBar.addSeparator()
		self.actionDelGroup = self.toolBar.addAction("Remove Group", self.removeGroupHandler)
		self.actionDelDrone = self.toolBar.addAction("Remove Drone", self.removeDroneHandler)
		self.toolBar.addSeparator()
		self.actionStartDrone = self.toolBar.addAction("Start Drone", self.startDroneHandler)

		if self.treeWidget.topLevelItemCount() == 0:
			self.actionDrone.setEnabled(False)
		self.actionDelGroup.setEnabled(False)
		self.actionDelDrone.setEnabled(False)
		self.actionStartDrone.setEnabled(False)

	def addGroupHandler(self):
		self.groupPopup = AddGroupPopup()
		self.groupPopup.show()

	def addDroneHandler(self):
		self.dronePopup = AddDronePopup()
		self.dronePopup.show()

	def removeGroupHandler(self):
		for item in self.treeWidget.selectedItems():
			print(item.text(0))
			if self.treeWidget.indexOfTopLevelItem(item) != -1:
				remove_group(item.text(0))

	def removeDroneHandler(self):
		for item in self.treeWidget.selectedItems():	
			print(item.text(0))
			if self.treeWidget.indexOfTopLevelItem(item) == -1:
				remove_drone(item.text(0))
	
	def startDroneHandler(self):
		for item in self.treeWidget.selectedItems():	
			if self.treeWidget.indexOfTopLevelItem(item) == -1:
				start_drone(item.text(0))

	def treeWidgetSelection(self, item):
		if self.treeWidget.indexOfTopLevelItem(item) == -1:
			self.actionStartDrone.setEnabled(True)
			self.actionDelDrone.setEnabled(True)
			self.actionDelGroup.setEnabled(False)
		if self.treeWidget.indexOfTopLevelItem(item) != -1:
			self.actionStartDrone.setEnabled(False)
			self.actionDelDrone.setEnabled(False)
			self.actionDelGroup.setEnabled(True)
			
	def treeWidgetContextMenuHandler(self, pos):
		globalPos = self.mapToGlobal(pos)
		item = self.treeWidget.itemAt(pos)
		if item != None:
			menu = QtWidgets.QMenu()
			if item.parent() == None:
				action_add_drone = self.actionDrone
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

def add_group(name):
	rospy.wait_for_service('ardrone_manager/add_group')
	myService = rospy.ServiceProxy('ardrone_manager/add_group', AddGroup)	
	myService(name)
	global ex
	ex.actionDrone.setEnabled(True)

def add_drone(group_name, drone_name, ip, drone_type):
	rospy.wait_for_service('ardrone_manager/add_drone')
	myService = rospy.ServiceProxy('ardrone_manager/add_drone', AddDrone)
	myService(group_name, drone_name, ip, drone_type)

def remove_drone(name):
	rospy.wait_for_service('ardrone_manager/del_drone')
	myService = rospy.ServiceProxy('ardrone_manager/del_drone', DelDrone)
	myService(name)

def remove_group(name):
	rospy.wait_for_service('ardrone_manager/del_group')
	myService = rospy.ServiceProxy('ardrone_manager/del_group', DelGroup)
	myService(name)

def start_drone(name):
	rospy.wait_for_service('ardrone_manager/start_drone')
	myService = rospy.ServiceProxy('ardrone_manager/start_drone', StartDrone)
	myService(name)

if __name__ == '__main__':
	myData = {}
	rospy.init_node("ardrone_manager_gui")
	app = QtWidgets.QApplication(sys.argv)
	windowManager()
	ex = Ui_MainWindow()
	ex.show()
	sys.exit(app.exec_())

