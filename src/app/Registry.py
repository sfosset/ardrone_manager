#!/usr/bin/env python
from DroneGroup import DroneGroup
from Drone import Drone

class Registry:
	def __init__(self):
		self.groupList = {}
		
	def addGroup(self, groupName):
		if(not self.groupList.has_key(groupName)):
			self.groupList[groupName]=DroneGroup(groupName)
			return self.groupList[groupName]
		else:
			print("A group with this name already exists")
			return None

	def addDrone(self, groupName, droneName, ip):		
		if(self.groupList.has_key(groupName)):			
			group = self.groupList[groupName]
			if(not group.droneList.has_key(droneName)):
				drone = Drone(droneName, ip)
				group.addDrone(drone)
				return drone
			else:
				print("A drone with this name already exists in this group")
				return None
		else:
			print("This group doesn't exists")
			return None

