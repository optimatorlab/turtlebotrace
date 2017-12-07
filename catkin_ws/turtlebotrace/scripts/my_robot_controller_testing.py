#!/usr/bin/env python
import numpy
import math
import rospy
import socket

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from turtlebotrace.msg import telem
from turtlebotrace.srv import get_id
from turtlebotrace.msg import start_race


# This is only used for manual control:	
key_mapping = { 'w': [ 0, 1], 'x': [0, -1],
				'a': [-1, 0], 'd': [1,  0],
				's': [ 0, 0] }

		
class myController():
	def __init__(self):
		# Initialize our current location (unknown right now):
		self.x = None
		self.y = None

		# Also initialize our initial location (for offset purposes):
		self.init_x = None
		self.init_y = None
		
		# Initialize our clearance ahead:
		self.range_ahead = 0.0
		self.range_ahead_angle = 0.0
		
		# Initialize the flag to tell us if the race has started:
		self.isRaceStart = False
		
		# Wait for the the "get_id" service:
		print 'Waiting for "get_id" service to become available...'
		rospy.wait_for_service('get_id')
		print 'DONE'
	
		# Capture the name of our computer:
		myHostname = socket.gethostname()	# This should be your UB userid
		print myHostname
		
		# Request an ID and a starting location:
		print "Requesting ID and starting location..."
		call_get_id = rospy.ServiceProxy('get_id', get_id)
		myResponse = call_get_id(myHostname)
		myID = int(myResponse.robotID)
		self.init_x = float(myResponse.init_x)
		self.init_y = float(myResponse.init_y)
		self.isRaceStart = myResponse.isRaceStarted
		
		print "DONE."
		# print myResponse
		print "\trobotID = ", myID
		print "\tinit_x  = ", self.init_x
		print "\tinit_y  = ", self.init_y
		

		# Define the subscribers to receive info from gazebo, based on robotID:
		myOdomTopic = "/robot%d/odom" % (myID)
		rospy.Subscriber(myOdomTopic, Odometry, self.odom_callback)

		myScanTopic = "/robot%d/scan" % (myID)
		rospy.Subscriber(myScanTopic, LaserScan, self.scan_callback)
		
		rospy.Subscriber("start_race", start_race, self.start_race_callback)

		# Define our publisher for the telemetry info:
		pubTelem = rospy.Publisher('telem', telem, queue_size=1)

		# Define the twist publisher based on robotID:
		myTwistTopic = "/robot%d/cmd_vel_mux/input/teleop" % (myID)
		self.twist_pub = rospy.Publisher(myTwistTopic, Twist, queue_size=1)
		
		# Subscribe to the keys topic (ONLY FOR MANUAL CONTROL):
		myKeyTopic = "keys%d" % (myID)
		rospy.Subscriber(myKeyTopic, String, self.keys_callback)

	
		rospy.init_node('talker', anonymous=True)
		rate = rospy.Rate(1) # 1hz
		while not rospy.is_shutdown():

			# FIXME -- Use "self.range_ahead" to guide the robot.
	
			# Publish our telem info
			if (self.x is not None):
				telem_msg = telem()
				telem_msg.id = myID
				telem_msg.x = self.x
				telem_msg.y = self.y
				
				# rospy.loginfo(telem_msg)
				pubTelem.publish(telem_msg)
				# print "x = ", self.x, "y = ", self.y
			rate.sleep()
	
	def keys_callback(self, msg):
		# This callback is only used for manual control (for testing purposes only).
		# DO NOT USE THIS CALLBACK FOR THE RACE.
		if ((len(msg.data) == 0) or (not key_mapping.has_key(msg.data[0]))):
			return 	# unknown key
		if (self.isRaceStart):	
			vels = key_mapping[msg.data[0]]
			# print vels
			t = Twist()
			t.angular.z = vels[0]
			t.linear.x 	= vels[1] 
			self.twist_pub.publish(t)
	
	def odom_callback(self, data):
		#rospy.loginfo(rospy.get_caller_id() + " I heard: %s", data.data)
		#rospy.loginfo(data.pose.pose.position.x)
		self.x = data.pose.pose.position.x + self.init_x
		self.y = data.pose.pose.position.y + self.init_y
		# print "callback x = ", self.x
	
	def scan_callback(self, data):
		# rospy.loginfo(data)
		
		# Find the middle element of the ranges array. 
		'''
		# FIXME -- This might not be the best way to detect obstacles!
		self.range_ahead = data.ranges[len(data.ranges)/2]
		print "range = ", self.range_ahead
		'''
		
		# A better approach, courtesy of Andrew...
		# Find (1) the minimum distance detected by our scanner, and 
		# (2) the angle corresponding to that obstacle.
		matrx = data.ranges
		# print matrx
		self.range_ahead = numpy.nanmin(matrx)
		#print self.range_ahead
		print self.range_ahead
		if (numpy.isnan(self.range_ahead)):
			self.range_ahead = -1 	# This means that we didn't detect an obstacle
			self.range_ahead_angle = -math.atan(1.0)*4.0 
		else:
			rangeIdx = matrx.index(self.range_ahead)
			self.range_ahead_angle = data.angle_min+rangeIdx*(data.angle_max -data.angle_min)/len(matrx) 
		                
		print "range = ", self.range_ahead, " angle = ", self.range_ahead_angle*180/3.14
	
	
	def start_race_callback(self, data):
		print "I got start race info:"
		rospy.loginfo(data)
		
		self.isRaceStart = True


if __name__ == '__main__':
	try:
		myController()
	except rospy.ROSInterruptException:
		pass
        
        
