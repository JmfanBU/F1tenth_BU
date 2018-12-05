#!/usr/bin/env python

import rospy
from racecar_control.msg import drive_param
import curses

forward = 0;
left = 0;


stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyop', anonymous=True)

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)


stdscr.refresh()

key = ''
while key != ord('q'):

	key = stdscr.getch()
	stdscr.refresh()

	if key == curses.KEY_UP: 
		forward = forward + 1;
		if forward >= 40:
			forward = 40
		elif forward < -40:
			forward = -40
		stdscr.addstr(2, 20, "Up  ")
		stdscr.addstr(2, 25, '%.2f' % forward)
		stdscr.addstr(5, 20, "    ")
	elif key == curses.KEY_DOWN:
		forward = forward - 1; 
		if forward >= 40:
			forward = 40
		elif forward < -40:
			forward = -40
		stdscr.addstr(2, 20, "Down")
		stdscr.addstr(2, 25, '%.2f' % forward)
		stdscr.addstr(5, 20, "    ")
	if key == curses.KEY_LEFT:  			
		left = left + 0.1; 
		if left >= 0.78:
			left = 0.78
		elif left < -0.78:
			left = -0.78
		stdscr.addstr(3, 20, "left")
		stdscr.addstr(3, 25, '%.2f' % left)
		stdscr.addstr(5, 20, "    ")
	elif key == curses.KEY_RIGHT:
		left = left - 0.1; 
		if left >= 0.78:
			left = 0.78
		elif left < -0.78:
			left = -0.78
		stdscr.addstr(3, 20, "rgt ")
		stdscr.addstr(3, 25, '%.2f' % left)
		stdscr.addstr(5, 20, "    ")
	if key == curses.KEY_DC:
		left = 0
		forward = 0
		stdscr.addstr(5, 20, "Stop")
	msg = drive_param()
	msg.velocity = forward
	msg.angle = left
	pub.publish(msg)
curses.endwin()
