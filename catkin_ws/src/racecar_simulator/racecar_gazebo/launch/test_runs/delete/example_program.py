#!/usr/bin/python
import time
import os
from setproctitle import *
##################################################
setproctitle('master_test')
for x in range(0, 100):
	time.sleep(3)
	print os.getpid()
##########################################
