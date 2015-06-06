#!/usr/bin/env python
import shlex
import rospy
import roslaunch
import rospkg
import subprocess
import os
from ardrone_manager.srv import *
from std_msgs.msg import *
def launchFileLaunch2(pkg_name, launch_file_name):
	rospack=rospkg.RosPack()
	
	pkg_path = rospack.get_path(pkg_name)
	folder_name = 'launch' #default
	
	#loading launchfile
	print launchfile	
	roscore_master_uri = rospy.get_param("/roslaunch/uris").values()[0]
	roscore_port = int(roscore_master_uri.replace('/', ':').split(':')[4]) #quick and dirty a master uri looks like http://mia:13111/

	config = roslaunch.config.load_config_default([launchfile], 11311) #TODO configurable port
	processes = []
	for node in config.nodes:	
		processes.append(roslaunch.nodeprocess.create_node_process(None, node, config.master.uri))
		
	for process in processes:
		process.start()

	return processes

def launchFileLaunch(pkg_name, launch_file_name, args=''):
	return subprocess.Popen(shlex.split('roslaunch '+pkg_name+' '+launch_file_name+' '+args))


class RealDrone: #TODO:abstract class
	def __init__(self, name, ip):
		self.name = name
		self.ip = ip
		self.droneType='real'
		self.isStarted = False
	def start(self):
		return None		
	def stop(self):
		return None
class SimDrone:
	worldProcess=None
	def __init__(self, name):
		self.name = name
		self.droneType='simulation'
		self.isStarted = False
		self.ip="0.0.0.0"
		self.process=None
		if self.__class__.worldProcess == None:
			self.__class__.worldProcess=launchFileLaunch('ardrone_manager','gazebo_world.launch')
	def start(self):
		if self.process==None:
			self.process=launchFileLaunch('ardrone_manager', 'spawn_quadrotor.launch', 'sim_name:="'+self.name+'"')
			return None
	def stop(self):
		self.process.kill()
		return None 


class DroneGroup:
	def __init__(self, name):
		self.name=name
		self.droneList={}

	def addDrone(self, drone):
		self.droneList[drone.name]=drone


class Registry:
	def __init__(self):
		self.groupList = {}

		rospy.loginfo('Creating "/ardrone_manager/list" topic')
		self.list_pub=rospy.Publisher('/ardrone_manager/list', String, queue_size=5)
	def addGroup(self, req):
		groupName=req.groupName
		group=self.getGroup(groupName)
		if not group:
			self.groupList[groupName]=DroneGroup(groupName)
			self.list_pub.publish(self.getList())
			return groupName
		else:
			print("A group with the name '"+groupName+"' already exists")
			return ""

	def delGroup(self, req):
		groupName=req.groupName
		group=self.getGroup(groupName)
		if group:
			for drone in group.droneList.values:
				self.delDrone(group.name, drone.name) #For disconnecting the drones
			del self.groupList[groupName]
			print("Group '"+groupName+"' deleted")
			self.list_pub.publish(self.getList())
			return groupName
		else:
			print("Can't delete the group '"+groupName+"', no group with this name")
 			return ""

	def addDrone(self, req):		
		groupName=req.groupName
		droneName=req.droneName
		droneType=req.droneType
		ip=req.ip	
		for group in self.groupList.values():
			if self.getDrone(group.name, droneName):
				print("A drone with the name '"+droneName+"'  already exists in the group '"+groupName+"'")
				return ""
	
		group = self.getGroup(groupName)
		if group:
			if droneType=='real':
				drone = RealDrone(droneName, ip)
			elif droneType=='simulation':
				drone = SimDrone(droneName)
			else:
				print('Wrong drone type given')
			
			group.addDrone(drone)
			self.list_pub.publish(self.getList())
			print("Drone '"+droneName+"' ("+droneType+") added in group '"+groupName+"'")
			return droneName
		else:
			print("Group '"+groupName+"' doesn't exist")
			return ""

	def delDrone(self, req):
		groupName=req.groupName
		droneName=req.droneName
		drone = getDrone(groupName, droneName)
		if drone:
			drone.stop()
			del self.groupList[groupName].droneList[droneName]
			self.list_pub.publish(self.getList())
			print("Drone '"+droneName+"' from group '"+groupName+"' deleted")
			return droneName
		else:
			print("Can't delete the drone, no drone with name '"+droneName+"' or group with name '"+group.name+"'")
			return ""

	def moveDrone(self, req):
		droneName=req.droneName
		oldGroupName=req.oldGroupName
		newGroupName=req.newGroupName
		drone=self.getDrone(oldGroupName, droneName)
		if drone:
			newGroup=self.getGroup(newGroupName)
			if newGroup:
				newGroup.droneList[droneName]=drone
				del oldGroup.droneList[droneName]
				self.list_pub.publish(self.getList())
				print("Drone '"+drone.name+"' moved from group '"+oldGroupName+"' to group '"+newGroupName+"'")
				return drone.name
			else:
				print("Can't move drone '"+drone.name+"' from group '"+oldGroupName+"' to group '"+newGroupName+"', no destination group with this name")
				return ""
		else: 
			print("Can't move drone '"+drone.name+"' from group '"+oldGroupName+"' to group '"+newGroupName+"', no drone with this name or departure group with this name")
			return ""
		
	def getList(self):
		liste={}
		for group in self.groupList.values():
			liste[group.name]={}
			for drone in group.droneList.values():
				liste[group.name][drone.name]={'droneType':drone.droneType, 'ip':drone.ip, 'isStarted':drone.isStarted}
		return str(liste)
	def getGroup(self, groupName):
		if self.groupList.has_key(groupName):
			return self.groupList[groupName]
		else:
			return False
	
	def getDrone(self, groupName, droneName):
		group = self.getGroup(groupName)
		if group:
			if group.droneList.has_key(droneName):
				return group.droneList[droneName]
			else:
				 return False
		else:
			return False
class DroneInterface:
	def __init__(self, registry):
		self.registry = registry

	def startDrone(self, req):
		groupName=req.groupName
		droneName=req.droneName
		drone=self.registry.getDrone(groupName, droneName)
		if drone:
			print("Trying to start drone '"+droneName+"' from group '"+groupName+"")
			drone.start()
			return droneName
		else:
			print("Can't start the drone '"+droneName+"', no drone with this name or group '"+groupName+"'")
			return ""

	def stopDrone(self, req):
		groupName=req.groupName
		droneName=req.droneName
		drone = self.registry.getDrone(groupName, droneName)
		if drone:
			print("Trying to stop drone '"+droneName+"' from group '"+groupName+"")
			drone.stop()
			return droneName
		else:
			print("Can't disconnect the drone '"+droneName+"', no drone with this name or group '"+groupName+"'")
			return ""




def main():
	
	rospy.loginfo('Initializing the ARDrone manager node')
	rospy.init_node('ardrone_manager', None, False, None, False, False, True)

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
	
	rospy.loginfo('Creating "/ardrone_manager/start_drone" service')
	startDroneService = rospy.Service('/ardrone_manager/start_drone', StartDrone, droneInterface.startDrone)
	rospy.loginfo('Creating "/ardrone_manager/stop_drone" service')
	disconnectDroneService = rospy.Service('/ardrone_manager/stop_drone', StopDrone, droneInterface.stopDrone)


	

	rospy.spin()
#	group1 = registry.addGroup('groupe1')
#	drone1 = registry.addDrone('groupe1', 'drone1', '192.168.25.10')
	#drone1.connect()

main()
