from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Read the SDF file content
    sdf_path = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'
    
    with open(sdf_path, 'r') as f:
        sdf_content = f.read()

    # Spawn robot 1
    spawn_robot1 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/spawn_entity', 
            'gazebo_msgs/srv/SpawnEntity', 
            f"{{name: 'turtlebot3_1', xml: '{sdf_content}', robot_namespace: 'robot1', initial_pose: {{position: {{x: 1.0, y: 1.0, z: 0.01}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, reference_frame: 'world'}}"
        ],
        name='spawn_robot1',
        shell=True
    )

    # Spawn robot 2
    spawn_robot2 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/spawn_entity', 
            'gazebo_msgs/srv/SpawnEntity', 
            f"{{name: 'turtlebot3_2', xml: '{sdf_content}', robot_namespace: 'robot2', initial_pose: {{position: {{x: 2.0, y: 2.0, z: 0.01}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, reference_frame: 'world'}}"
        ],
        name='spawn_robot2',
        shell=True
    )

    return LaunchDescription([
        spawn_robot1,
        spawn_robot2
    ])
