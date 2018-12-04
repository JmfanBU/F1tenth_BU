#!/usr/bin/python

"""from subprocess import check_output
def get_pid(name):
    return check_output(["pidof",name])"""

##############################
import psutil

PROCNAME = 'master_test'

for proc in psutil.process_iter():
    # check whether the process name matches
    if proc.name == PROCNAME:
    	a = proc.pid
    	break
print a
############################################
"""
flag = 0
while 1
   try:
       if flag == 0
			subprocess.call('source $HOME/catkin/devel/setup.bash', shell=True)
			cmd = 'roslaunch racecar_gazebo gazebo_test_run.launch'
			simulation = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 
			flag = 1
	except signalInterrupt:
		subprocess.call('killall gzserver', shell=True)
		exit(0)


"""