#!/usr/bin/env python
import rospy
import roslaunch
import rospkg
import os
from ardrone_manager.srv import *
from std_msgs.msg import *

def launchFileLaunch(pkg_name, launch_file_name):
	rospack=rospkg.RosPack()
	
	pkg_path = rospack.get_path(pkg_name)
	folder_name = 'launch' #default
	
	#loading launchfile
	launchfile = os.path.join(pkg_path, folder_name, launch_file_name)
	print launchfile	
	roscore_master_uri = rospy.get_param("/roslaunch/uris").values()[0]
	roscore_port = int(roscore_master_uri.replace('/', ':').split(':')[4]) #quick and dirty a master uri looks like http://mia:13111/

	config = roslaunch.config.load_config_default([launchfile], 11311)
	processes = []
	for node in config.nodes:	
		processes.append(roslaunch.nodeprocess.create_node_process(None, node, config.master.uri))
		
	for process in processes:
		process.start()

	return processes


def main():
	launchFileLaunch("cvg_sim_gazebo","ardrone_testworld.launch") 

if __name__=='__main__':
	main()
