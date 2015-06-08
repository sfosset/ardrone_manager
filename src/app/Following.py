#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from ardrone_manager.msg import *

class Follower():
	def __init__(self, drone):
		self.drone = drone
		self.leader = None
		self.state = 'independent'
		self.currentController = None
		self.isFollowing = False
		self.leaderTwist = Twist()
		self.myTwist = Twist()
		self.cmd = Twist()
		self.leaderOdometrySub=None
		self.myOdometrySub = rospy.Subscriber(self.drone+"/odometry", Odometry, callbackMyOdometry)
		self.cmd_vel_pub=rospy.Publisher(self.drone+"/cmd_vel", Twist)
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
						self.leaderOdometrySub=rospy.Subscriber(self.leader+"/odometry", Odometry, callbackLeaderOdometry)
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
					self.leaderOdometrySub=rospy.Subscriber(self.leader+"/odometry", Odometry, callbackLeaderOdometry)
					self.isFollowing=True	
				elif self.state=='follower':
					self.leader = changingDrone
					self.leaderOdometrySub=rospy.Subscriber(self.leader+"/odometry", Odometry, callbackLeaderOdometry)
					self.isFollowing=True
				elif self.state=='independant':
					self.leader = changingDrone
					self.isFollowing=False
			else:
				if self.leader == changingDrone:
					self.leader=None
					self.isFollowing=False
	
	def follow(self, leader):
		while(true):
			if self.isFollowing:
				self.cmd.vertical.
		
	def callbackLeaderOdometry(data):
		self.leaderTwist = data.twist
	def callbackMyOdometry(data)
		self.myTwist = data.twist

def main():
	droneName=rospy.get_param('~drone_name',False)
	groupName=rospy.get_param('~group_name',False)
	if !droneName:
		print("Parameters _drone_name and _group_name must be set")
		exit()

	rospy.loginfo("Initializing "+droneName+"/follower node")
	rospy.init_node(droneName+'/follower')
	
	follower=Follower(droneName)
	
	rospy.Subscriber("/ardrone_manager/"+groupName+"/state", StateChange, follower.stateSub)


if __name__=='__main__':
	main()
