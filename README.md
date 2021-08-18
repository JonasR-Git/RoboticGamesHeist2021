# RoboticGamesHeist2021

###Run Instructions

  How to start the Simulation:
  - `$ roslaunch heist map_2.launch`

  For other Maps:
  - `Change the map file in guard_navigation to the right .yaml file`
  - `Add in the Launch file:`
    - `<include file="$(find heist_guard)/launch/map_2_guard.launch"/>`

### Miscellaneous
- roslaunch [package] [launch file]
    - Starts Gazebo with Turtlebot3 and the map environment
    - *Example: roslaunch heist map_1.launch*

- rostopic pub [topic name] [message name]
    - publishes a message via Terminal
    - *Example: rostopic pub -r 5 /guard/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x:0.0, y: 0.0, z: 0.0}}'* 
    	- means that the robot should move 5 times, means 5 seconds with the linear speed x = 0.1

- rostopic list
    - Displays all the currently published topics
    
- rostopic info [topic name]
    - Shows the current list of subscribers and publishers to the given topic
    - *Example: rostopic info /guard/scan*
    
- rosmsg show [message name]
    - Shows the content of a message and how it is defined
    - *Example: rosmsg show geometry_msgs/Pose*
    
- rostopic echo [topic name]
    - Shows live the change of the topic 
    - *Example: rostopic echo /guard/scan*
  
- rosnode list
    - Display the list of nodes that are currently running in the System
    
- rosnode info [node name]
    - Shows the current Subscriptions and Publications of a spezific node
    - *Example: rosnode info /guard*
       
- rqt_graph
  - Displays all the available running nodes

- chmod +x [Python file]
    - Grants executable permissions to the specified file
    - *Example: chmod +x map_builder.py*

- ls -la 
    - Shows the permissions for the files in the current directory
    - Every Python script requires the executable permission
    - *If x is included, the file has executable permissions*
  
- Every Python script requires a specific first line
    - *#!/usr/bin/env python3.8*






### Guidelines

Write a Publisher: 
1. Determine a name for the topic to publish
2. Determine the type of the messages that the topic will publish
3. Determine the frequency of topic publication. How many Messages per second?
4. Create a publisher object with parameters chosen 
5. Keep publishing the topic message at the selected frequency

Write a Subscriber:
1. Identify the name for the topic to listen to
2. Identify the type of the messages to be received
3. Define a callback function that will be automatically executed when a new message is received on the topic
4. Start listening for the topic messages


### Pulisher and Receiver and what is needed for them

- Message Structure:
   - package_name/message_type
       - type1 filed1
       - type2 filed2
   	
   - *Example: geometry_msgs/Twist*
        - *linear*
            - *x float64*
            - *y float64*
            - *z float64*
        - *angular*
            - *x float64*
            - *y float64*
            - *z float64*
        	
- Create a new Message Type:
  - create a folder named msg
  - create a message, for example: *IotSensor.msg*
  - define the msg, for example: int32 id, string name, float32 temperature, floar32 humidity
  - add message_generation to find_package in CMakeLists.txt
  - add the msg file to the add_message_files in CMakeLists
  - add message_runtime to the Catkin_depends in catkin_package in CMakeLIsts.txt
  - add <build_depend>message_generation</build_depend> and <exec_depend>message_runtime</exec_depend> in the package.xml

- Create a Ros Node:
   - rospy.init_node('[Node Name]', anonymous=True)
       - anonymous=True makes sure that every Node is unique

- Create a Publisher Object:
   - pub = rospy.Publisher('[Topic Name]', [Topic Type], queue_size=[Buffer size])
       - queue_size= 10 for example saves up to 10 messages, before deleting the oldest one for saving a new one
    
- Spezify the publish rate:
   - rate = rospy.Rate([int])

- Example: 

    def talker():

        pub = rospy.Publisher('sender', String, queue_size=5)
		
        rospy.init_node('talker', anonymous=True)
	
        rate = rospy.Rate(1) 
	
        while not rospy.is_shutdown():
	
            hello_str = "hello World"
		
            rospy.publish(hello_str)
			
            rate.sleep()
			
- Create a Subscriber Object:
   - rospy.Subscriber("[Topic Name]", [Topic Type], [callback_function])
   
- Create a Callback Function: 
   - def listener_callback(message): print("I heard %s", message.data)
   
- Start the Listening:
   - rospy.spin()
   
- Example:

    def listener_callback(message):
	
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", message.data)
		
    def listener():
	
        rospy.init_node('listener', anonymous=True)
		
        rospy.Subscriber("sender", String, listener_callback)
		
        rospy.spin()

### How to create a Map (.yaml und .pgm)

- Launch the right map
  - *Example: roslaunch heist map_2_test.launch*
- Start gmapping for example
  - *Example: roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping*
- Navigate the Robot manuell or automatic
  - *Example: roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch* (keyboard)
  - *Example: roslaunch turtlebot3_gazebo turtlebot3_simulation.launch* (automatic)
