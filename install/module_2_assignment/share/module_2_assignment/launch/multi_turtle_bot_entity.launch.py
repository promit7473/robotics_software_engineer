from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the turtlebot3 model
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Path to the turtlebot3 model (adjust this to the correct path in your system)
    model_path = os.path.join(
        pkg_turtlebot3_gazebo, 
        'models', 
        'turtlebot3_waffle', 
        'model.sdf'
    )

    # Spawn first robot
    spawn_robot1 = ExecuteProcess( 
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'turtlebot3_1',  
             '-file', model_path,
             '-x', '1.0',
             '-y', '1.0',
             '-z', '0.01',
             '-robot_namespace', 'robot1'],
        name='spawn_robot_1',
        output='screen'
    )


    # Spawn second robot
    spawn_robot2 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'turtlebot3_2',
             '-file', model_path,
             '-x', '2.0',
             '-y', '2.0',
             '-z', '0.01',
             '-robot_namespace', 'robot2'],
        name='spawn_robot_2',
        output='screen'
    )

    # Spawn third robot
    spawn_robot3 = ExecuteProcess(  # Changed variable name from spawn_robot2 to spawn_robot3
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'turtlebot3_3',  # Changed entity name
             '-file', model_path,
             '-x', '3.0',
             '-y', '3.0',
             '-z', '0.01',
             '-robot_namespace', 'robot3'],
        name='spawn_robot_3',
        output='screen'
    )


    # Spawn fourth robot
    spawn_robot4 = ExecuteProcess(  
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'turtlebot3_4',  
             '-file', model_path,
             '-x', '4.0',
             '-y', '4.0',
             '-z', '0.01',
             '-robot_namespace', 'robot4'],
        name='spawn_robot_4',
        output='screen'
    )

    

    # Spawn fifth robot
    spawn_robot5 = ExecuteProcess(  
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'turtlebot3_5',
             '-file', model_path,
             '-x', '5.0',
             '-y', '5.0',
             '-z', '0.01',
             '-robot_namespace', 'robot5'],
        name='spawn_robot_5',
        output='screen'
    )

    # Robot controller nodes
    robot2_controller = Node(
        package='module_2_assignment',
        executable='multi_turtlebot3_controller',
        name='robot2_controller',
        namespace='robot2',
        parameters=[
            {'cmd_vel_topic': '/robot2/cmd_vel'}
        ]
    )

    robot3_controller = Node(
        package='module_2_assignment',
        executable='multi_turtlebot3_controller',
        name='robot3_controller',
        namespace='robot3',
        parameters=[
            {'cmd_vel_topic': '/robot3/cmd_vel'}
        ]
    )

    robot4_controller = Node(
        package='module_2_assignment',
        executable='multi_turtlebot3_controller',
        name='robot4_controller',
        namespace='robot4',
        parameters=[
            {'cmd_vel_topic': '/robot4/cmd_vel'}
        ]
    )

    return LaunchDescription([
        spawn_robot1,
        spawn_robot2,
        spawn_robot3,
        spawn_robot4,
        spawn_robot5,  
        robot2_controller,
        robot3_controller,
        robot4_controller
        
    ])