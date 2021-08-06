# RoboticGamesHeist2021

### Miscellaneous
- roslaunch [package] [launch file]
    - Starts Gazebo with Turtlebot3 and the map environment
    - *Example: roslaunch heist map_1.launch*

- rostopic list
    - Displays all the currently published topics

- rostopic info [topic name]
    - Shows the current list of subscribers and publishers to the given topic
    - *Example: rostopic info /guard/scan*
  
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

-Idea for Navigation:
    -1: https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html
    -2: https://github.com/ros-planning/navigation
    -3: http://wiki.ros.org/Robots/evarobot/noetic/Autonomous%20Navigation%20of%20a%20Known%20Map%20with%20Evarobot
