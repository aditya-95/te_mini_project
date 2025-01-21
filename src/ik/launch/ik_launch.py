import os
from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
    urdf_file_path= '../urdf/manipulator2.urdf'
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='ik',
            executable='basic_control',
            name='control'
            ),
        Node(
            package='ik',
            executable='ik_joint_state_publisher',
            name='ik_publisher'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description':robot_desc}]
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', '../rviz2/rviz_config.rviz']
        )
        ])
