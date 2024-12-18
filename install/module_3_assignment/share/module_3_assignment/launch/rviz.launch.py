from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('module_3_assignment')
    urdfFile = os.path.join(pkg_path, 'urdf', 'task1.urdf')

    return LaunchDescription([
        # Node to publish the robot's URDF to the robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdfFile]),
     
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
    ])