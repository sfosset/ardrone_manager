#!/usr/bin/env python

import rospy
import roslaunch

class Drone:
	def __init__(self, name, ip):
		self.name = name
		self.ip = ip
		self.isConnected = False

	def connect(self):
		#Launching the drone node
		#self.node = roslaunch.core.Node('ardrone_autonomy', 'ardrone_driver', self.name, '/', None, '-ip '+self.ip)
		self.node = roslaunch.core.Node('rqt_gui', 'rqt_gui')
		
		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()

		self.process = launch.launch(self.node)
		#print process.is_alive()
		#process.stop()

		#Subscribing topics
		

	def disconnect(self):
		self.process.stop()

	#def __del__(self):
	#	self.disconnect()
