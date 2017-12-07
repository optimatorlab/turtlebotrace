#!/usr/bin/env python

import sys, select, tty, termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':

	# Need the user to specify the ID of the robot to manually control:
	if (len(sys.argv) == 2):
		robotID = int(sys.argv[1])
		print "Controlling Robot %d" % (robotID)
	else:
		print "You must provide an integer-valued robot ID to control.  Sorry things didn't work out.  Bye."
		exit()
	
	myKeyTopic = "keys%d" % (robotID)
	key_pub = rospy.Publisher(myKeyTopic, String, queue_size=1)
	rospy.init_node("keyboard_driver", anonymous=True)
	rate = rospy.Rate(100)
	old_atr = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())
	print "Publishing keystrokes.  Press Ctrl-Z to exit..."
	while not rospy.is_shutdown():
		if (select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]):
			print sys.stdin.read(1)
			key_pub.publish(sys.stdin.read(1))
		rate.sleep()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
	

