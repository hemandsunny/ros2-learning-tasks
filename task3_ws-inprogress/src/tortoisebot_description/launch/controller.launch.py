from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    simple_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_velocity_controller', '--controller-manager', '/controller_manager']
    )

    ld = LaunchDescription()
    # ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(simple_velocity_controller_spawner)
    return ld