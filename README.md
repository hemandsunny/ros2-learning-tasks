# ros2-learning-tasks
The goal of this project is to build a solid understanding of core ROS 2 principles (nodes, topics, launch files, custom packages) and their application in a realistic, simulated environment.

Task 1 - Simulate tortoisebot.urdf in Gazebo and run it using Teleop

Go to task1_ws directory -``cd task1_ws``

Build the workspace - ``colcon build``

RViz only launch - ``ros2 launch tortoisebot_description rviz_launch.py ``

Gazebo+Rviz launch - ``ros2 launch tortoisebot_gazebo gazebo_launch.py``


Task 2 - Get nearest object distance using LiDAR and laser filters

Go to task2_ws directory - ``cd task2_ws``

Build workspace - ``colcon build``

Closest Distance using custom filter - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py ``

Closest Distance using laser_filters package - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py  use_laser_filters:=true ``
