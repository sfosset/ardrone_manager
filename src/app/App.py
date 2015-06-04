#!/usr/bin/env python
import rospy
import roslaunch
from ardrone_manager.srv import *

class Drone:
	def __init__(self, name, ip):
		self.name = name
		self.ip = ip
		self.isConnected = False

	def connect(self):
		#Launching the drone node
		self.node = roslaunch.core.Node('ardrone_autonomy', 'ardrone_driver', self.name, '/', None, '-ip '+self.ip)
		#self.node = roslaunch.core.Node('rqt_gui', 'rqt_gui')
		
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


class DroneGroup:
	def __init__(self, name):
		self.name=name
		self.droneList={}

	def addDrone(self, drone):
		self.droneList[drone.name]=drone


class Registry:
	def __init__(self):
		self.groupList = {}

	def addGroup(self, req):
		groupName=req.groupName
		if not self.groupList.has_key(groupName):
			self.groupList[groupName]=DroneGroup(groupName)
			return self.groupList[groupName].name
		else:
			print("A group with the name '"+groupName+"' already exists")
			return ""

	def delGroup(self, req):
		groupName=req.groupName
		if self.groupList.has_key(groupName):
			for drone in self.groupList[groupName].droneList.values:
				self.delDrone(group.name, drone.name) #For disconnecting the drones
			del self.groupList[groupName] 
			print("Group '"+groupName+"' deleted")
		else:
			print("Can't delete the group '"+groupName+"', no group with this name")
 
	def addDrone(self, req):		
		groupName=req.groupName
		droneName=req.droneName
		ip=req.ip
		if self.groupList.has_key(groupName):			
			for group in self.groupList.values():
				for drone in group.droneList.values():
					if drone.name == droneName:
						print("A drone with the name '"+drone.name+"'  already exists in the group '"+group.name+"'")
						return drone
	
			group = self.groupList[groupName]
			drone = Drone(droneName, ip)
			group.addDrone(drone)
			print("Drone '"+drone.name+"' added in group '"+group.name+"'")
		else:
			print("Group '"+groupName+"' doesn't exists")
			return None

	def delDrone(self, req):
		groupName=req.groupName
		droneName=req.droneName
		if self.groupList.has_key(groupName, droneName):
			group = self.groupList[groupName]
			if group.droneList.has_key(droneName):
				groupe.droneList[droneName].disconnect()
				del groupe.droneList[droneName]
				print("Drone '"+droneName+"' from group '"+group.name+"' deleted")
			else:
				print("Can't delete the drone '"+droneName+"', no drone with this name in group '"+group.name+"'")

	def moveDrone(self, req):
		droneName=req.droneName
		oldGroupName=req.oldGroupName
		newGroupName=req.newGroupName
		if self.groupList.has_key(oldGroupName):
			oldGroup=self.groupList[oldGroupName]
			if oldGroup.droneList.has_key(droneName):
				drone = oldGroup.droneList[droneName]
				if self.groupList.has_key(newGroupName):
					newGroup = self.groupList[newGroupName]
					newGroup.droneList[drone.name]=drone
					del oldGroup.droneList[drone.name]
					print("Drone '"+drone.name+"' moved from group '"+oldGroupName+"' to group '"+newGroupName+"'")
				else:
					print("Can't move drone '"+drone.name+"' from group '"+oldGroupName+"' to group '"+newGroupName+"', no destination group with this name")
			else: 
				print("Can't move drone '"+drone.name+"' from group '"+oldGroupName+"' to group '"+newGroupName+"', no drone with this name in the departure group")
		else:
			print("Can't move drone '"+drone.name+"' from group '"+oldGroupName+"' to group '"+newGroupName+"', no departure group with this name")
	def getList(self, req):
		liste={}
		for group in self.groupList.values():
			liste[group.name]={}
			for drone in group.droneList.values():
				liste[group.name][drone.name]={'ip':drone.ip, 'isConnected':drone.isConnected}
		return str(liste)

class DroneInterface:
	def __init__(self, registry):
		self.registry = registry

	def connectDrone(self, req):
		groupName=req.groupName
		droneName=req.droneName
		if self.registry.groupList.has_key(groupName, droneName):
			group = self.registry.groupList[groupName]
			if group.droneList.has_key(droneName):
				print("Trying to connect drone '"+droneName+"' from group '"+group.name+"")
				groupe.droneList[droneName].connect()
			else:
				print("Can't connect the drone '"+droneName+"', no drone with this name in group '"+group.name+"'")
	def disconnectDrone(self, req):
		groupName=req.groupName
		droneName=req.droneName
		if self.registry.groupList.has_key(groupName, droneName):
			group = self.registry.groupList[groupName]
			if group.droneList.has_key(droneName):
				print("Trying to disconnect drone '"+droneName+"' from group '"+group.name+"")
				groupe.droneList[droneName].disconnect()
			else:
				print("Can't disconnect the drone '"+droneName+"', no drone with this name in group '"+group.name+"'")




def main():
	
	rospy.loginfo('Initializing the ARDrone manager node')
	rospy.init_node('ardrone_manager')

	rospy.loginfo('Creating new drone registry')
	registry=Registry()
	
	rospy.loginfo('Creating new drone interface linked to the registry')
	droneInterface=DroneInterface(registry)

	rospy.loginfo('Creating "/ardrone_manager/add_group" service')
	addGroupService = rospy.Service('/ardrone_manager/add_group', AddGroup, registry.addGroup)
	rospy.loginfo('Creating "/ardrone_manager/del_group" service')
	delGroupService = rospy.Service('/ardrone_manager/del_group', DelGroup, registry.delGroup)

	rospy.loginfo('Creating "/ardrone_manager/add_drone" service')
	addDroneService = rospy.Service('/ardrone_manager/add_drone', AddDrone, registry.addDrone)
	rospy.loginfo('Creating "/ardrone_manager/del_drone" service')
	delDroneService = rospy.Service('/ardrone_manager/del_drone', DelDrone, registry.delDrone)
	rospy.loginfo('Creating "/ardrone_manager/move_drone" service')
	moveDroneService = rospy.Service('/ardrone_manager/move_drone', MoveDrone, registry.moveDrone)
	rospy.loginfo('Creating "/ardrone_manager/get_list" service')
	getListService = rospy.Service('/ardrone_manager/get_list', GetList, registry.getList)
	
	rospy.loginfo('Creating "/ardrone_manager/connect_drone" service')
	connectDroneService = rospy.Service('/ardrone_manager/connect_drone', ConnectDrone, droneInterface.connectDrone)
	rospy.loginfo('Creating "/ardrone_manager/disconnect_drone" service')
	disconnectDroneService = rospy.Service('/ardrone_manager/disconnect_drone', DisconnectDrone, droneInterface.disconnectDrone)



	rospy.spin()
#	group1 = registry.addGroup('groupe1')
#	drone1 = registry.addDrone('groupe1', 'drone1', '192.168.25.10')
	#drone1.connect()

if __name__=='__main__':
	main()
