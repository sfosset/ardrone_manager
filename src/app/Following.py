#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from ardrone_manager.srv import *
from ardrone_manager.msg import *
from geometry_msgs.msg import *
class Follower():
	def __init__(self, drone, group):
		self.drone = drone
		self.group = group
		self.leader = None
		self.state = 'independent'
		self.currentController = None
		self.isFollowing = False
		self.leaderTwist = Quaternion()
		self.myTwist = Quaternion()
		self.cmd = Twist()
		self.leaderOdometrySub=None
		self.myOdometrySub = rospy.Subscriber(self.drone+"/imu", Quaternion, self.callbackMyOdometry)
		self.cmd_vel_pub=rospy.Publisher("/"+self.drone+"/cmd_vel", Twist)
		self.groupSub = rospy.Subscriber("/ardrone_manager/"+self.group+"/state", StateChange, self.stateSub)
		self.r = rospy.Rate(1)
	#receive the state changement from the group publisher and process it
	def stateSub(self, data):
		changingDrone = data.droneName
		newState = data.newState
		
		if changingDrone == self.drone:
			if newState=='leader':
				self.isFollowing=False
				self.state=newState
				self.leader=self.drone
			elif newState=='follower':
				if self.state == 'leader':
					self.isFollowing=False
					self.leader==None
					self.state==newState
				else:
					if self.leader!=None:
						if self.leaderOdometrySub!=None:
							self.leaderOdometrySub.unregister()
						self.leaderOdometrySub=rospy.Subscriber(self.leader+"/imu", Quaternion, self.callbackLeaderOdometry)
						self.isFollowing=True
						self.state=newState
					else:
						self.isFollowing=False
						self.state=newState
			elif newState=='independent':
				if self.state == 'leader':
					self.isFollowing=False
					self.leader=None
					self.state=newState
				else:
					self.isFollowing=False
					self.state=newState				
		else:
			if newState=='leader':
				if self.state=='leader':
					self.leader = changingDrone
					self.state = 'follower' 
					if self.leaderOdometrySub!=None:
						self.leaderOdometrySub.unregister()
					self.leaderOdometrySub=rospy.Subscriber(self.leader+"/imu", Quaternion, self.callbackLeaderOdometry)
					self.isFollowing=True	
				elif self.state=='follower':
					self.leader = changingDrone
					if self.leaderOdometrySub!=None:
						self.leaderOdometrySub.unregister()
					self.leaderOdometrySub=rospy.Subscriber(self.leader+"/imu", Quaternion, self.callbackLeaderOdometry)
					self.isFollowing=True
				elif self.state=='independant':
					self.leader = changingDrone
					self.isFollowing=False
			else:
				if self.leader == changingDrone:
					self.leader=None
					self.isFollowing=False
	
	def follow(self):
		while(True):
			if self.isFollowing:
				self.cmd.angular.z=self.leaderTwist.orientation.z
				self.cmd_vel_pub.publish(self.cmd)
			self.r.sleep()
	def callbackLeaderOdometry(self, data):
		self.leaderTwist = data
	def callbackMyOdometry(self, data):
		self.myTwist = data
	
	def changeGroup(self, req):
		self.leader = None
		self.group = req.group
		self.groupSub.unregister()
		self.groupSub = rospy.Subscriber("/ardrone_manager/"+self.group+"/state", StateChange, self.stateSub)



def main():
	rospy.init_node('osef')
	droneName=rospy.get_param('~drone_name',False)
	groupName=rospy.get_param('~group_name',False)
	if not (droneName and groupName):
		print("Parameters _drone_name and _group_name must be set")
		exit()

	rospy.loginfo("Initializing "+droneName+"/follower node")
	
	follower=Follower(droneName, groupName)
	
	changeGroupService = rospy.Service("change_group", ChangeGroup, follower.changeGroup)
	follower.follow()
	exit()	
if __name__=='__main__':
	main()
