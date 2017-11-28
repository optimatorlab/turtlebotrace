#!/usr/bin/env python

# license removed for brevity
# This code is copied from
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

import rospy
import sys
from parseCSVstring import *

from kobuki_msgs.msg import BumperEvent

from demo_0.srv import get_id
from demo_0.msg import telem
from demo_0.msg import start_race

	
class make_turtle:
	def __init__(self, hostname):
		# Set self.turtle[robotID]
		self.hostname 	= str(hostname)
		self.x			= None	# FIXME -- Tower needs to initialize this
		self.y			= None	# FIXME -- Tower needs to initialize this
		self.yaw		= None	# FIXME -- Tower needs to initialize this
		self.finishTime	= None	# Remains None until this robot finishes.
		self.collisions = 0
		
class listener():
	def __init__(self):
		
		# Need the user to specify the game to play:
		if (len(sys.argv) == 2):
			thisRace = str(sys.argv[1])
			print "Starting the game using the  %s  track" % (thisRace)
		else:
			print "You must provide a track name.  Sorry things didn't work out.  Bye."
			exit()
			
		# Capture the parameters for this track:
		self.init_x = {}
		self.init_y = {}
		trackFile = "races/race_params_%s.txt" % (thisRace)
		try:
			rawInfo = parseCSVstring(trackFile, returnJagged=True, fillerValue=-1, delimiter=',') 
			# Convert all values to floats
			for i in range(0,len(rawInfo)):
				rawInfo[i] = map(float, rawInfo[i])
			
			# 1) Verify the values in the race params file:
			#	a) 10 x's
			if (len(rawInfo[0]) != 10):
				print "Sorry, you need to provide exactly 10 x coordinates for the robot starting positions."
				exit()
					
			#	b) 10 y's
			if (len(rawInfo[1]) != 10):
				print "Sorry, you need to provide exactly 10 y coordinates for the robot starting positions."
				exit()

			#	c) either all x's are the same or all y's are the same.
			if ( (min(rawInfo[0]) != max(rawInfo[0])) and (min(rawInfo[1]) != max(rawInfo[1])) ): 
				print "Sorry, either all of the x or all of the y coordinates need to be the same."
				exit()
				
			#	d) finish rectangle is properly defined.
			if (len(rawInfo[2]) != 2):
				print "Sorry, you need to provide 2 values for the x coordinates of the finish rectangle."
				exit()
			if (len(rawInfo[3]) != 2):
				print "Sorry, you need to provide 2 values for the y coordinates of the finish rectangle."
				exit()
			if (rawInfo[2][0] >= rawInfo[2][1]):
				print "Sorry, the max x value of the finish rectangle must be larger than the min x value."
				exit()
			if (rawInfo[3][0] >= rawInfo[3][1]):
				print "Sorry, the max y value of the finish rectangle must be larger than the min y value."
				exit()
					
			# 2) Get the x and y coordinates for 10 robots.
			#	 This info is in the first two lines of the file (rows 0 and 1, columns 0 through 9).
			for j in range(1,10+1):
				self.init_x[j] = float(rawInfo[0][j-1])
				self.init_y[j] = float(rawInfo[1][j-1])

			# 3) Get the coordinates for the finish "rectangle".	
			#	 This infor is in the 3rd and 4th lines of the file (rows 2 and 3)
			self.MIN_X = float(rawInfo[2][0])
			self.MAX_X = float(rawInfo[2][1])
			self.MIN_Y = float(rawInfo[3][0])
			self.MAX_Y = float(rawInfo[3][1])

		except:
			print "There was an error reading the %s track file.  Perhaps this file doesn't exist?" % (trackFile)
			exit()
						
			
		# In ROS, nodes are uniquely named. If two nodes with the same
		# node are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('listener', anonymous=True)
		
		rate = rospy.Rate(10)	# Is 10 Hz fast enough?

		
		rospy.Subscriber("telem", telem, self.telemCallback)

		# Initialize our robotID counter:
		self.nextID = 0
	
		# Initialize our turtle data structure
		self.turtle = {}	# An empty Python dictionary
		
		# Initialize the race to be unstarted
		self.isRaceStarted = False
		
		# Define the publisher that will initiate the race:
		self.pub_start_race = rospy.Publisher('start_race', start_race, queue_size=10)
		
		# Define the service that will provide ID numbers:
		print "Initializing 'get_id' service...",
		self.id_service = rospy.Service('get_id', get_id, self.get_id_callback)			
		print "DONE"
		
		# WAIT HERE FOR THE USER TO RELEASE THE GAME:
		print '*****HIT ENTER TO RELEASE THE GAME*****'
		raw_input()
		print 'HERE WE GO...'		
		self.isRaceStarted = True
		
		# Establish the start time of the race:
		self.startTime = rospy.get_time() 
		
		# Publish a message so our turtlebots will start the race.
		start_msg = start_race()

		start_msg.goal_x		= [self.MIN_X, self.MAX_X]
		start_msg.goal_y		= [self.MIN_Y, self.MAX_Y]
		
		self.pub_start_race.publish(start_msg)		
		
	
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
		
	def get_id_callback(self, request):	
		# print request
		print "Received ID request from ", request.hostname
		
		# Have we already received a request from this hostname?
		foundMatch = False
		for robotID in self.turtle:
			if (self.turtle[robotID].hostname == request.hostname):
				foundMatch = True
				respondRobotID = robotID
				print "\tThis hostname is already associated with robotID %d" % (robotID)
				break
		
		if not foundMatch:		
			self.nextID += 1
			respondRobotID = self.nextID
			print "\tThis hostname is new.  It will now have robotID %d" % (respondRobotID)

			self.turtle[self.nextID] = make_turtle(request.hostname)
		
		print "\t%s will be robotID %d" % (request.hostname, respondRobotID)
		
		# Subscribe to the bumper topic for this robot:
		myRobotBumperTopic = "/robot%d/mobile_base/events/bumper" % (respondRobotID)
		rospy.Subscriber(myRobotBumperTopic, BumperEvent, self.robotBumperCallback, (respondRobotID))

		return {'robotID': respondRobotID, 'init_x': self.init_x[respondRobotID], 'init_y': self.init_y[respondRobotID], 'isRaceStarted': self.isRaceStarted}
		
	def telemCallback(self, data):
		# rospy.loginfo(rospy.get_caller_id() + " I heard: %s", data)
		
		# Update the turtle's current location
		self.turtle[data.id].x = data.x
		self.turtle[data.id].y = data.y
		
		# Did this turtle cross the finish line?
		if ((self.MIN_X <= data.x <= self.MAX_X) and (self.MIN_Y <= data.y <= self.MAX_Y)): 	
			if (self.turtle[data.id].finishTime is not None):
				self.turtle[data.id].finishTime = rospy.get_time() - self.startTime		# This is in [seconds].  It doesn't specify a date.
				print "Robot %d finished the race in %d seconds" % (data.id, self.turtle[data.id].finishTime)
		
	def robotBumperCallback(self, data, (robotID)):
		# rospy.loginfo(data)
		# print robotID
		
		# Log Collisions
		if (data.state == 1):
			self.turtle[robotID].collisions += 1	
			print "\tRobot %d has %d collisions" % (robotID, self.turtle[robotID].collisions)
	
if __name__ == '__main__':
    listener()
