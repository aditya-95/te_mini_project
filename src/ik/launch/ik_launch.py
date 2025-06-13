import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
    this_pkg_path = os.path.join(get_package_share_directory('ik'))

    # change .urdf to .xacro if needed
    urdf_file_path= os.path.join(this_pkg_path, 'urdf', 'manipulator2.urdf') 

    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_file_path}")
    
    # determine whether to process xacro or urdf
    if urdf_file_path.endswith('.xacro'):
        doc = xacro.process_file(urdf_file_path)
        robot_desc = doc.toprettyxml(indent=' ')
    else:
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
            name='ik_publisher',
            parameters=[{'robot_description':robot_desc}]
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
            arguments=['-d', os.path.join(this_pkg_path, 'rviz2', 'rviz_config.rviz')]
        )
        ])
