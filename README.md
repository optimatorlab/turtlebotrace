# Networking Instructions

## Changing the hostname on your Ubuntu machine
By default, the hostname is "ROS-EDU".  Follow these steps to change it to your UB username.

1. Open a terminal window and issue the following commands, one at a time:
	
	- Enter the following command and change `127.0.1.1	ROS-EDU` to `127.0.1.1	yourUBusername`:
	```
	sudo pico /etc/hosts
	```
	
	- Enter the following command and change from `ROS-EDU` to `yourUBusername`:
	```
	sudo pico /etc/hostname
	```
		
		*You'll probably get an error message...don't worry*

	- Finally, tell Ubuntu about the changes you made:
	```
	sudo /etc/init.d/hostname.sh
	```
		
		*You'll probably get an error message...don't worry*

2. Now, reboot your Ubuntu machine

3. When the system reboots, open a terminal window.  
	
	You should see "user@yourUBusername:"
	
	If not, go back to Step 1 and repeat.

## Edit the list of IP addresses for everyone in class.
- **Murray will post a message to Slack with more info.  Still waiting for everyone to get connected to the router.**

---


# Install the "turtlebotrace" Package 
*You only need to do this when there's new code to download/install*

1. Clone the master branch from github to your Ubuntu machine.  We will remove any existing clones first.
	```
	cd ~/Downloads
	rm -rf turtlebotrace
	git clone -b master https://github.com/optimatorlab/turtlebotrace.git
	```

2. Run the automated installer script:
	```
	cd ~/Downloads/turtlebotrace
	chmod +x install_turtlebotrace.sh
	./install_turtlebotrace.sh
	chmod -x install_turtlebotrace.sh
	```
	
3. Go ahead and delete the clone from your Downloads:
	```
	cd ~/Downloads
	rm -rf turtlebotrace
	```
	
---

# Create a Race:

You will need to create the following 4 files (examples have been provided):
1. `course_yourUBusername.world`
	
	This is where you define the racetrack.

2. `robots_yourUBusername.launch`

	Edit this file to specify the starting locations of 10 robots.

3. `race_yourUBusername.launch`

	Edit this file to make sure it references your versions of `course_yourUBusername.world` and `robots_yourUBusername.launch`.
	
4. `race_params_yourUBusername.txt`
	```
	% FILE: race_params_murray.txt
	%
	% Each line starting with a percent sign is a comment.
	% x coordinates for 10 robots' initial positions:
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9 
	% y coordinates for 10 robots' initial positions:
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	% NOTE 1:  The starting line must be either vertical (same x coordinates) or horizontal (same y coordinates). 
	% NOTE 2:  These x and y coordinates must match EXACTLY what is specified in robots_yourUBusername.launch.
	%
	% Range of x coordinates for the finish rectangle (min, max):
	8, 9
	% Range of y coordinates for the finish rectangle (min, max):
	5, 6
	% NOTE:  The min and max values for the finish should be different (so we have a rectangle, not a line).
	```

We also need the `one_robot.launch` file, but **do not edit it**.  This launcher will be the same for all races.

See Brayton's tutorial (LINK) for detailed insructions.

---

# Standalone Version (Not Networked)
1. Open Gazebo and place the turtlebots on the track:
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	roslaunch turtlebotrace race_murray.launch
	```
	
	NOTE: Replace `murray` with a different UBusername to run a different race.
	
2. Start the tower: 
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace tower.py murray
	```

	NOTE: This script requires one input argument, which is the name of the race.  Replace `murray` with the UBusername matching what you used in Step 1.

3. Launch your controller node:
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace my_robot_controller_testing.py 
	```

	NOTE: Each student will create their own controller script.  Replace `testing` with your UBusername to run your control algorithm.  This doesn't have to match the name of the race (you will be using your controller to race on tracks created by other users).

4. Launch your keyboard controller (only for testing purposes):
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace key_publisher.py ROBOTID
	```
	
	NOTE: Replace `ROBOTID` with the ID number of the robot you wish to control.
	
---

# Networked Version 
This version is currently incomplete.  When it is working:

## On the Tower computer (server):
1. Set master
	```
	export ROS_MASTER_URI=http://darkstar:11311
	```
	
2. Launch gazebo
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	roslaunch turtlebotrace race_murray.launch
	```
	
	NOTE: Replace `murray` with a different UBusername to run a different race.
	
3. Run tower
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace tower.py murray
	```

	NOTE: This script requires one input argument, which is the name of the race.  Replace `murray` with the UBusername matching what you used in Step 2.


## On your computer (client):
- Set master
	```
	export ROS_MASTER_URI=http://darkstar:11311
	```
- Run controller
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace my_robot_controller_testing.py 
	```

	NOTE: Each student will create their own controller script.  Replace `testing` with your UBusername to run your control algorithm.  This doesn't have to match the name of the race (you will be using your controller to race on tracks created by other users).

