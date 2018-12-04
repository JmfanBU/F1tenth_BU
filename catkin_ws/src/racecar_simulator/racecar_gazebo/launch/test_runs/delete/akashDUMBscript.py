#####################################################
# This program automates the testing process for the adaptive testing environment on the f1/10 robot
# - Takes number of iterations(loops) as an argument (e.g to run with 5 loops, run: "python master_test_script.py 5" )
#
# 1) run java map world generator - o/p ==> 2D array(.png file) and .world file
# 2) run roslaunch files. These should terminate once goal is reached
#		o/p ==> .csv files , path coordinates , completion time
# 3) run AI to adjust i/p parameters to java map world generator
# 4) back to step 1. (Define number of loops)
# 
# TODO:  - see TODOs thoughout code below
#####################################################

#!/usr/bin/env python
import os
import subprocess
import sys
import time
import threading
import signal
import logging
from setproctitle import *
from os.path import expanduser


setproctitle('master_test') # set process name for ROS to identify

TIMEALLOWED = 5000 # change to a large number(e.g. 5 minutes) for actual tests

""" Clear all existing generated files"""
""" TODO: implement the Initialize function once the AI code is made"""
def Initialize():
	#delete bag files
	home = expanduser("~")
	os.chdir(home+'/catkin_ws/src/racecar_simulator/racecar_gazebo/launch/test_runs/test_results')
	subprocess.call('rm *.bag', shell=True)	

""" runs roslaunch file to implement test run in gazebo and terminates once the robot reaches its goal"""


def ROS():
	try:
		cmd1 = 'roslaunch racecar_gazebo racecar_tunnel.launch'
		simulation1 = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 
		ros_node_pid1 = os.getpgid(simulation1.pid)

		time.sleep(5)

		cmd2 = 'roslaunch hector_slam_launch tutorial.launch'
		simulation2 = subprocess.Popen(cmd2, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 
		ros_node_pid2 = os.getpgid(simulation2.pid)

		#kill after allowed time is exceeded
		time.sleep(TIMEALLOWED)
		logging.info('Timout - Goal not reached within 5 minutes...')

		subprocess.call('killall gzserver', shell=True)

		time.sleep(2)
		os.killpg(ros_node_pid1, signal.SIGTERM)  # Send the signal to all the process groups
		time.sleep(2)
		os.killpg(ros_node_pid2, signal.SIGTERM)  # Send the signal to all the process groups
	except KeyboardInterrupt:
		print "KILL CALLED"
		subprocess.call('killall gzserver', shell=True)

		time.sleep(2)
		os.killpg(ros_node_pid1, signal.SIGTERM)  # Send the signal to all the process groups
		time.sleep(2)
		os.killpg(ros_node_pid2, signal.SIGTERM)  # Send the signal to all the process groups
		print "KILLED ALL"

	logging.info('ROS ending...')


""" Output is .png file (2D array of path) and .world file."""
# TODO: - standardize location of .jar file and outputs 
def WorldGenerator():
	home = expanduser("~")
	os.chdir(home+'/catkin_ws/src/gzbo2_generator')
	subprocess.call('java Generator1', shell=True)
	logging.info('WorldGenerator ending...')

def PathGenerator():
	home = expanduser("~")
	os.chdir(home+'/catkin_ws/src/gzbo2_generator')
	subprocess.call('./Planning_Gazebo_Customized', shell=True)
	logging.info('Path Generated...')


def AI(csv_file, completion_time):
	subprocess.call('run AI code')

def main():
	home = expanduser("~")
	logging.basicConfig(filename=home+'/catkin_ws/src/test.log', level=logging.DEBUG,format='%(asctime)s:%(levelname)s:%(message)s')


	''' number of iterations'''
	if(len(sys.argv) == 2):
		try:
			iterations = int(sys.argv[1])
		except ValueError:
			print('Input is not an integer')
			sys.exit()
	else:
		print('Provide number of iterations')
		sys.exit()


	''' main loop'''
	
	for i in range(0,iterations):
		'''STEP 1'''
		WorldGenerator()
		time.sleep(2)
		PathGenerator()
		time.sleep(30)
		#logging.info('WorldGenerator ended')

		'''STEP 2'''
		ROS()
		logging.info('ROS ended')
		time.sleep(2)

		'''STEP 3'''
		#AI()
		#time.sleep(2)

	print('program ended')

if __name__ == "__main__":
   main()
