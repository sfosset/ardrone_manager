#!/usr/bin/env python
import signal
import shlex
import rospy
import roslaunch
import rospkg
import subprocess
import os
from gazebo_msgs.srv import*
from ardrone_manager.srv import *
from ardrone_manager.msg import *
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

class Drone:
	def __init__(self):	
		return None
	
	def start(self):
		return None
	def stop(self):
		return None
	
class RealDrone: #TODO:abstract class
	def __init__(self, name, ip):
		self.name = name
		self.ip = ip
		self.droneType='real'
		self.isStarted = False
		self.state='independent'
	def start(self):
		if self.isStarted == False:
			self.process=launchFileLaunch('ardrone_manager', 'real_drone.launch', 'ip:="'+self.name+'" droneName:="'+self.name+'"')
			self.isStarted=True
		return None		
	def stop(self):
		self.isStarted=False
		return None
class SimDrone:
	worldProcess=None
	def __init__(self, name):
		self.name = name
		self.droneType='simulation'
		self.isStarted = False
		self.ip="0.0.0.0"
		self.process=None
		self.state='independent'
		if self.__class__.worldProcess == None:
			self.__class__.worldProcess=launchFileLaunch('ardrone_manager','gazebo_world.launch')
	def start(self):
		if self.isStarted == False:
			self.process=launchFileLaunch('ardrone_manager', 'spawn_quadrotor.launch', 'sim_name:="'+self.name+'"')
			self.isStarted=True
			return None
	def stop(self):
		if self.isStarted==True:
			rospy.wait_for_service('gazebo/delete_model')
			delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
			result=delete_model(self.name)
			self.process.terminate()
			self.process.kill()
			self.isStarted = False
			return str(result)


class DroneGroup:
	def __init__(self, name):
		self.name=name
		self.droneList={}
	
		rospy.loginfo('Creating "ardrone_manager/'+self.name+'/state" topic')
		self.state_pub=rospy.Publisher( "ardrone_manager/"+self.name+"/state", StateChange, queue_size=5)
		
	def addDrone(self, drone):
		self.droneList[drone.name]=drone
	
	def delDrone(self, drone):
		del self.droneList[drone.name]

	def setState(self, drone, state):
		msg=StateChange(drone.name, state)
		self.state_pub.publish(msg)
		#TODO link with follower node
		if state=='leader':
			for droneInGroup in self.droneList.values():
				if droneInGroup.state == 'leader' and droneInGroup.name!=drone.name:
					droneInGroup.state='follower'
		drone.state = state
		return None


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
			for drone in group.droneList.values():
				drone.stop()
				group.delDrone(drone) 
		
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

		drone=self.getDrone(droneName)
		if drone:
			print("A drone with the name '"+droneName+"'  already exists")
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
			launchFileLaunch('ardrone_manager', 'connect_drone.launch', 'drone_name:="'+drone.name+'" group_name:="'+group.name+'"')
			print("Drone '"+droneName+"' ("+droneType+") added in group '"+groupName+"'")
			return droneName
		else:
			print("Group '"+groupName+"' doesn't exist")
			return ""

	def delDrone(self, req):
		droneName=req.droneName
		drone = self.getDrone(droneName)
		if drone:
			group = self.getGroupOfDrone(drone)
			drone.stop()
			group.delDrone(drone)
			self.list_pub.publish(self.getList())
			print("Drone '"+droneName+"' from group '"+group.name+"' deleted")
			return droneName
		else:	
			print("Can't delete the drone, no drone with name '"+droneName+"'")
			return ""

	def moveDrone(self, req):
		droneName=req.droneName
		newGroupName=req.newGroupName
		

		drone = self.getDrone(droneName)
		newGroup=self.getGroup(newGroupName)
		if newGroup and drone:
			oldGroup = getGroupOfDrone(drone)
			oldGroup.setState(drone, 'independent') #forcing the drone state to stop the follower and avoir conflict with new group existing leader
			
			#give change to following node
			rospy.wait_for_service(drone.name+'/follower/change_group')
			change_group = rospy.ServiceProxy(drone.name+'/follower/change_group', DeleteModel)
			change_group(newGroup.name)


			oldGroup.delDrone(drone)
			newGroup.addDrone(drone)
				
			self.list_pub.publish(self.getList())
			print("Drone '"+drone.name+"' moved from group '"+oldGroup.name+"' to group '"+newGroup.name+"'")
			return drone.name
		else: 
			print("Can't move drone '"+droneName+"' to '"+newGroupName+"' no drone or destination group with this names")
			return ""
		
	def getList(self, req = None):
		liste={}
		for group in self.groupList.values():
			liste[group.name]={}
			for drone in group.droneList.values():
				liste[group.name][drone.name]={'droneType':drone.droneType, 'ip':drone.ip, 'isStarted':drone.isStarted, 'state':drone.state, 'ip':drone.ip}
		return str(liste)

	def getGroup(self, groupName):
		if self.groupList.has_key(groupName):
			return self.groupList[groupName]
		else:
			return False
	
	def getDrone(self, droneName):
		for group in self.groupList.values():
			if group.droneList.has_key(droneName):
				return group.droneList[droneName]
			else:
				return False
		else:
			return False

	def getGroupOfDrone(self, drone):
		for group in self.groupList.values():
			if group.droneList.has_key(drone.name):
				return group
			else:
				return False

class DroneInterface:
	def __init__(self, registry):
		self.registry = registry

	def startDrone(self, req):
		droneName=req.droneName
		drone=self.registry.getDrone(droneName)
		if drone:
			print("Trying to start drone '"+droneName+"'")
			drone.start()
			self.registry.list_pub.publish(self.registry.getList())	
			return droneName
		else:
			print("Can't start the drone '"+droneName+"', no drone with this name")
			return ""

	def stopDrone(self, req):
		droneName=req.droneName
		drone = self.registry.getDrone(droneName)
		if drone:
			print("Trying to stop drone '"+droneName+"'")
			drone.stop()
			self.registry.list_pub.publish(self.registry.getList())	
			return droneName
		else:
			print("Can't disconnect the drone '"+droneName+"', no drone with this name")
			return ""

	def setState(self, req):
		droneName=req.droneName
		state=req.state
		if state not in ('leader', 'follower', 'independent'):
			print("'"+state+"' is not a possible state")
			return ''

		drone = self.registry.getDrone(droneName)
		if drone:
			if drone.state == state:
				print("Drone '"+droneName+"' already in state '"+state+"'")
				return ''
			else:
				group= self.registry.getGroupOfDrone(drone)
				print("Setting drone '"+droneName+"' state to '"+state+"'")
				group.setState(drone, state)
				self.registry.list_pub.publish(self.registry.getList())	
				return droneName
		else:
			print("No drone with name '"+droneName+"'")
			return ''

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
	
	rospy.loginfo('Creating "/ardrone_manager/start_drone" service')
	startDroneService = rospy.Service('/ardrone_manager/start_drone', StartDrone, droneInterface.startDrone)
	rospy.loginfo('Creating "/ardrone_manager/stop_drone" service')
	disconnectDroneService = rospy.Service('/ardrone_manager/stop_drone', StopDrone, droneInterface.stopDrone)	
	
	rospy.loginfo('Creating "/ardrone_manager/get_list" service')
	disconnectDroneService = rospy.Service('/ardrone_manager/get_list', GetList, registry.getList)	

	rospy.loginfo('Creating "/ardrone_manager/set_state" service')
	disconnectDroneService = rospy.Service('/ardrone_manager/set_state', SetState, droneInterface.setState)	


	rospy.spin()
#	group1 = registry.addGroup('groupe1')
#	drone1 = registry.addDrone('groupe1', 'drone1', '192.168.25.10')
	#drone1.connect()
if __name__ == '__main__':
	main()
