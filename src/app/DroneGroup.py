#!/usr/bin/env python
from Drone import Drone

class DroneGroup:
	def __init__(self, name):
		self.name=name
		self.droneList={}

	def addDrone(self, drone):
		self.droneList[drone.name]=drone

