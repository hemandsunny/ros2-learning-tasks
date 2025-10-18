from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os



def generate_launch_description():

    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot_Task2.urdf']))
    

    world_file=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'worlds', 'square_world.sdf'])

    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'config', 'rviz_task2.rviz']))

    gazebo_rviz_launch_file=IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']),launch_arguments={'urdf_path':urdf_path,'world_path':world_file,'rviz_file':rviz_config}.items())
    
    closestDistanceFinderNode=Node(
        package='tortoisebot_filters',
        name='lidar_closest_distance',
        executable='find_closest_object_distance',
        output='screen'
    )
    closestDistanceFinderNode=TimerAction(
        period=3.0,
        actions=[

            Node(
        package='tortoisebot_filters',
        name='lidar_closest_distance',
        executable='find_closest_object_distance',
        output='screen'
    )

        ]
    )
    return LaunchDescription([
        
        gazebo_rviz_launch_file,
        closestDistanceFinderNode

    ])