# OptiTrack
ROS interface to OptiTrack Motive Application for Rigid Body position and orientation. The Motive Application is a visual interface for users to see the scene as OptiTrack cameras see them, declare rigid bodies, and configure the Camera Frame Rate and Data Streaming Settings of OptiTrack cameras. Initializes a ROS node called ('optitrack\_publisher') which receives, parses, and publishes the position and orientation of a wand to a topic named ('optitrack\_data'). Additionally, you can record positions and orientations of the three omnids and human in their corresponding CSV files by executing ('talker\_with\_CSVs') instead of ('talker'). Another node named ('Optitrack\_Subscriber\_With\_Statistics') is subscribed to ('optitrack\_data') and prints out the published optitrack data to confirm its receival. Finally, ('Optitrack\_Subscriber\_With\_Statistics') also publishes metadata of the optitrack data to a topic named ('/statistics'). More in-depth descriptions will be given on these nodes and topics.

## Quick Setup & Use
For minimal latency, zero packet loss, and consistent time intervals between message publications, you should wire the computer that Motive is running on to your router via Ethernet and then from your router to the computer running this ROS package also via Ethernet.

### Setup
1. Download the repo on your computer
2. On the computer you plan to run the optitrack ROS node, connect to the same Wifi network as the computer running Motive (default is NUMSR).
	- The server (Motive's) address and local (your computer's) address is hard coded in the package.
	  If your selected common network is not NUMSR, you must change the code in ('optitrack\_data\_publisher.cpp'),
	  specifically in initialize\_optitrack(), to set g\_connectParams.serverAddress to the Motive computer's IP 
	  address and g\_connectParams.localAddress to your computer's IP address. If you're computer is Ubuntu, you
	  can get the IP address by entering ('ip addr show') in Terminal.  In the Motive computer's Terminal, 
	  enter ('ipconfig') and look for the address listed under IPv4.
3. Boot up Motive and set the Camera Frame Rate in the top right corner to your desired rate.
4. Click the Data Streaming symbol in the bottom right corner to open the Data Streaming Pane. Make sure the following Data Streaming Pane settings match:
	- Enable: ON
	- Local Interface: the IP address matches the one specified in ('optitrack\_data\_publisher.cpp') initialize\_optitrack() g_connectParams.serverAddress. Set to loopback if motive computer = ROS computer
	- Transmission Type: UniCast
	- Rigid Bodies: On
	- All other marker types: Off
	- Up Axis: Z-Axis

### How to Run Code
1. In terminal, enter the repo's folder optitrack\_ROS\_ws and run ('colcon build')
2. Open three new terminal windows and in each of those terminals, enter the repo's folder optitrack\_ROS\_ws and run ('source install/setup.bash')
3. Run ('ros2 run optitrack talker'), ('ros2 run optitrack listener\_with\_stats'), and ('ros2 topic echo /statistics') in its own terminal window in that order. Make sure no error statements are printed when running talker.
4. Enter ('q') in the talker window to stop receiving OptiTrack data. Press ('Ctrl+C') to stop listener and statistics.

### Node Overview
1. ('talker') ('talker\_with\_CSVs') -> ('optitrack\_publisher\')
	- receives data from a [NatNet Client](https://docs.optitrack.com/v3.0/developer-tools/natnet-sdk/natnet-creating-a-native-c++-client-application), parses it, and publishes a message to a topic named ('optitrack\_data')
	- ('talker\_with\_CSVs') will save received data into corresponding CSV files for analysis, whereas ('talker') will not
	- Topic ('optitrack\_data') receives information in the following listed order:
		- name of the Rigid Body
		- mean error of its markers location to its expected position (in meters)
		- whether the Rigid Body is in frame (1) or occluded (0)
		- frame ID
		- OptiTrack system's timestamp of the frame, not to be confused with your computer's timestamp (in seconds)
		- xyz positions of the geometric center - the mean position of indivudal markers belonging to that Rigid Body (in meters)
		- quaternion representation of Rigid Body orientation
		- (if available) [system, transmit, and client lateny](https://docs.optitrack.com/v3.0/developer-tools/natnet-sdk/latency-measurements)
		- your computer's timestamp the instant before the message is published so ('OptiTrack\_Subscriber\_Node\_With\_Statistics') can calculate the ROS publishing and receiving latency. (in mileseconds)
3. ('listener\_with\_stats') -> ('OptiTrack\_Subscriber\_Node\_With\_Statistics')
	- receives the message sent by ('optitrack\_publisher\'), prints it out, and publishes ROS message latency (time between publishing and receival)
	  and period (time between consecutive published messages) to a topic named ('/statistics')
	- Topic ('/statistics') receives information in the structure of a statistics_msg/msg/MetricsMessage:
		- statistics:
			- data_type 1: average (milleseconds)
			- data_type 2: minimum (milleseconds)
			- data_type 3: maximum (milleseconds)
			- data_type 4: standard deviation (milleseconds)
			- data_type 5: sample count

### Message
1. ('optitrack::msg::String')	- Provides an interface between ROS and OptiTrack data receiver. Contains variable of type std\_msgs::msg::Header which holds the timestamp of the instant before the node 
			  	  publishes the message for ROS to automatically calculate the message latency between publisher and subscriber node. Also contains information from OptiTrack data in
			  	  string form. You can modify the names and variable types to hold different parts of OptiTrack data. If you do though, be sure to edit ('optitrack\_data\_publisher.cpp')'s 
			  	  DataHandler() function for setting message field values and ('optitrack\_subscriber\_with\_statistics.cpp') for RCLCPP log to print the data.
