*Updated Thursday, Dec. 7, 2017*

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

2. Now, reboot your Ubuntu machine.

3. When the system reboots, open a terminal window.  
	
	You should see "user@yourUBusername:"
	
	If not, go back to Step 1 and repeat.

## Edit the list of IP addresses for everyone in class.

1. Make a backup of your file:
	`sudo cp /etc/hosts /etc/hosts.bak`

2. Open the /etc/hosts file for editing:
	`sudo pico /etc/hosts`

3. Paste the following lines into the **bottom** of the /etc/hosts file:
	```
	192.168.0.100   darkstar
	192.168.0.102   darkstar-wifi
	192.168.0.105   braytonc
	192.168.0.106   ianmarre
	192.168.0.107   kylevand
	192.168.0.108   leurysme
	192.168.0.109   mpreasor
	192.168.0.110   shanegam
	192.168.0.111   ntrichte
	192.168.0.112   andrewko
	192.168.0.113   qzhang42
	```

4. Save the /etc/hosts file by hitting `ctrl-o`.

5. Exit pico by hitting `ctrl-x`.

6. Reboot your virtual machine.

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
	
	This is where you define the racetrack.  See the section below on "Creating a .world File".

2. `robots_yourUBusername.launch`

	Edit this file to specify the starting locations of 10 robots.

3. `race_yourUBusername.launch`

	Edit this file to make sure it references your versions of `course_yourUBusername.world` and `robots_yourUBusername.launch`.
	
4. `race_params_yourUBusername.txt`

	For example, here's the file for a racetrack named "murray":

	```
	% FILE: race_params_murray.txt
	% 
	% Each line starting with a percent sign is a comment.
	% x coordinates for 10 robots' initial positions:
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10 
	% y coordinates for 10 robots' initial positions:
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	% NOTE 1:  The starting line must be either vertical (same x coordinates) or horizontal (same y coordinates). 
	% NOTE 2:  These x and y coordinates must match EXACTLY what is specified in robots_yourUBusername.launch.
	%
	% Range of x coordinates for the finish rectangle (min, max):
	15, 19
	% Range of y coordinates for the finish rectangle (min, max):
	0, 3
	% NOTE:  The min and max values for the finish should be different (so we have a rectangle, not a line).
	```

We also need the `one_robot.launch` file, but **do not edit it**.  This launcher will be the same for all races.

## Creating a .world File
1. Open the `sample_racetrack.xls` spreadsheet in LibreOffice Calc (on your Ubuntu machine).
2. Change the x and y coordinates as you like (as long as they are integer valued).  A plot is provided so you can visualize your track. You should assume that each turtlebot occupies a 1x1 unit of space.
3. When you're done editing the spreadsheet, save it as `sample_racetrack.csv`.  This is in a **comma-delimited** format. 
	- NOTE 1:  This should be saved in `~/catkin_ws/src/turtlebotrace/scripts/races/`.
	- NOTE 2:  Cell A1 should start with a % sign.  This indicates that this row is a comment (not an actual x,y coordinate).
	- NOTE 3:  You should open the .csv file with a text editor (e.g., gedit) to verify that it looks like this (*your numbers may differ, but the format should be the same*):
	```
	% x,y,description (optional)
	0,-1,south west wall (starting line)
	1,-1,
	2,-1,
	```
4. A Python script has been written to convert your `.csv` file into a `.world` file.  Enter these commands in the terminal:
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts/races
	python create_world.py
	```
5. Rename the resulting `.world` file to be `course_yourUBusername.world` (replacing "yourUBusername" with your actual UB username).

6. Take a look at your world in gazebo:
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts/races
	gazebo course_yourUBusername.world
	```
	(of course by now you know to replace "yourUBusername" with your actual UB username).
	
	
---

# Coloring your turtlebots

1.  Use gimp to open `~/catkin_ws/src/turtlebotrace/robots/meshes/images/3f_stack_template.xcf`.

	**Do not overwrite this file.**  In the next step you will **export** to .jpg images.

2.  **Export** the following .jpg images with the appropriate number:
	- 3f_stack_green.jpg			#3
	- 3f_stack_orange.jpg			#4
	- 3f_stack_purple.jpg			#5
	- 3f_stack_pink.jpg				#6
	- 3f_stack_yellow.jpg			#7
	- 3f_stack_black.jpg			#8
	- 3f_stack_white.jpg			#9
	- 3f_stack_brown.jpg			#10

	All of these files should be saved in `~/catkin_ws/src/turtlebotrace/robots/meshes/images/`.  Red and blue versions are already there.

	If necessary, change the font color of the ID number.  Try to choose your colors/shading for maximum contrast.  We want to be able to easily identify our robots.
	
	
3. Use your text editor to create these files (editing them so they reference the appropriate color):
	- plate_top_green.dae
	- plate_top_orange.dae
	- plate_top_purple.dae
	- plate_top_pink.dae
	- plate_top_yellow.dae
	- plate_top_white.dae
	- plate_top_black.dae
	- plate_top_brown.dae

	See line 140 of `plate_top_red.dae`.  This is the line you need to edit, from `<init_from>./images/3f_stack_red.jpg</init_from>` to the appropriate color name.
	
	All of these files should be saved in `~/catkin_ws/src/turtlebotrace/robots/meshes`.  Red and blue versions are already there.

4. Use your text editor to create these files (editing them so they reference the appropriate color):
	- hexagons_green.urdf.xacro
	- hexagons_orange.urdf.xacro
	- hexagons_purple.urdf.xacro
	- hexagons_pink.urdf.xacro
	- hexagons_yellow.urdf.xacro
	- hexagons_white.urdf.xacro
	- hexagons_black.urdf.xacro
	- hexagons_brown.urdf.xacro

	Find line 220 of `hexagons_red.urdf.xacro`.  This is the line you need to edit, from `<mesh filename="package://turtlebotrace/robots/meshes/plate_top_blue.dae"/>` to the appropriate color name.

	All of these files should be saved in `~/catkin_ws/src/turtlebotrace/robots/urdf`.  Red and blue versions are already there.
	
5.  Use your text editor to create these files (editing them so they reference the appropriate color):
	- kobuki_hexagons_kinect_green.urdf.xacro:
	- kobuki_hexagons_kinect_orange.urdf.xacro:
	- kobuki_hexagons_kinect_purple.urdf.xacro:
	- kobuki_hexagons_kinect_pink.urdf.xacro:
	- kobuki_hexagons_kinect_yellow.urdf.xacro:
	- kobuki_hexagons_kinect_white.urdf.xacro:
	- kobuki_hexagons_kinect_black.urdf.xacro:
	- kobuki_hexagons_kinect_brown.urdf.xacro:

	The line you need to edit should be obvious.  Change the color appropriately.
	
	All of these files should be saved in `~/catkin_ws/src/turtlebotrace/robots/`.  Red and blue versions are already there.

6. Create the `robots_yourUBusername.launch` file, using `robots_murray.launch` as a template.  Create the 10 robots and place them in the appropriate locations for your racetrack.


---

# Running the Code
For testing purposes, it is recommended that you start with the "standalone" version.  This will require you to run gazebo on your machine.  

For the in-class contest, we will run in "networked" mode.

## Standalone Version (not networked)
1. **Terminal 1** -- Open Gazebo and place the turtlebots on the track:
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	roslaunch turtlebotrace race_murray.launch
	```
	
	NOTE: Replace `murray` with a different UBusername to run a different race.
	
2. **Terminal 2** -- Start the tower: 
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace tower.py murray
	```

	NOTE: This script requires one input argument, which is the name of the race.  Replace `murray` with the UBusername matching what you used in Step 1.

3. **Terminal 3** -- Launch your controller node:
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace my_robot_controller_testing.py 
	```

	NOTE: Each student will create their own controller script.  Replace `testing` with your UBusername to run your control algorithm.  This doesn't have to match the name of the race (you will be using your controller to race on tracks created by other users).

4. **Terminal 4** -- Launch your keyboard controller (only for testing purposes):
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace key_publisher.py the_Integer_RobotID_given_to_you_by_the_tower
	```
	
	- For example, if you were assigned RobotID 1, that last command would be `rosrun turtlebotrace key_publisher.py 1`.
	
5. Go back to **Terminal 2** and hit `Enter` to release the game.  

	- Your robot won't move until the tower has given the go-ahead signal.

6. Return to **Terminal 4** and use the `w` (forward), `a` (spin left), `d` (spin right), `x` (backwards), and `s` (stop) keys.

	- When you're done with the keyboard controller, hit `Ctrl-Z`.  
	- If you use Ctrl-C, the terminal window will probably freeze.  In that case, you'll have to close/re-open Terminal 4.
	
---

## Networked Version 
These instructions assume that 
- There is a computer named `darkstar` that will be the "master".  Replace `darkstar` everywhere below if another computer is used as the master.
- The race is `murray`.  Replace `murray` with the appropriate username to run a different race.
		
	 
### On the Tower computer (server):
1. **Terminal 1** -- Set master and launch gazebo:
	```
	export ROS_MASTER_URI=http://darkstar:11311
	cd ~/catkin_ws/src/turtlebotrace/scripts
	roslaunch turtlebotrace race_murray.launch
	```
	
	- Replace `darkstar` with the name of the computer that will run the tower. 
	- Replace `murray` with a different UBusername to run a different race.
	
3. **Terminal 2** -- Set master and run tower:
	```
	export ROS_MASTER_URI=http://darkstar:11311
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace tower.py murray
	```

	- NOTE 1: This script requires one input argument, which is the name of the race.  Replace `murray` with the UBusername matching what you used in Step 1.
	
	- NOTE 2: The robots won't be able to move until you "release the game" by hitting `Enter` in this terminal window.


### On the Client Computer:
1. **Terminal 1** -- Set master and run robot controller:
	```
	export ROS_MASTER_URI=http://darkstar:11311
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace my_robot_controller_testing.py 
	```

	NOTE: Each student will create their own controller script.  Replace `testing` with your UBusername to run your control algorithm.  This doesn't have to match the name of the race (you will be using your controller to race on tracks created by other users).

2. **Terminal 2** (FOR TESTING ONLY) -- Run the manual keyboard controller:
	```
	export ROS_MASTER_URI=http://darkstar:11311
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace key_publisher.py the_Integer_RobotID_given_to_you_by_the_tower
	```
	
	- For example, if you were assigned RobotID 3, that last command would be `rosrun turtlebotrace key_publisher.py 3`.

3. Tell the person running the Tower to "release the game" by hitting `Enter` in their Terminal 2.

