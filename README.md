# ros2-learning-tasks
The goal of this project is to build a solid understanding of core ROS 2 principles (nodes, topics, launch files, custom packages) and their application in a realistic, simulated environment.

**Common Setup**

Go to task<number>_ws directory -``cd task<number>_ws``

Build the workspace - ``colcon build``

Source the workspace - ``source install/setup.bash``


**Task 1 - Simulate tortoisebot.urdf in Gazebo and run it using Teleop**

RViz only launch - ``ros2 launch tortoisebot_description rviz_launch.py ``

Gazebo+Rviz launch - ``ros2 launch tortoisebot_gazebo gazebo_launch.py``


**Task 2 - Get nearest object distance using LiDAR and laser filters**

Closest Distance using custom filter - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py ``

Closest Distance using laser_filters package - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py  use_laser_filters:=true ``

**Task 3 - Follow a Ball Shaped Object**

Follow the closest object - ``ros2 launch tortoisebot_actions ball_follower_launch.py ``



