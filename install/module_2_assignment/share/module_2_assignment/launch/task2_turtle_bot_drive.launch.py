from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declaring launch arguments
    declare_b = DeclareLaunchArgument(
        'b',
        default_value='1.0',
        description='Parameter b for the robot driver'
    )
    declare_motion_type = DeclareLaunchArgument(
        'motion_type',
        default_value='circular',
        description='Motion type for the robot driver'
    )
    declare_linear_velocity = DeclareLaunchArgument(
        'linear_velocity',
        default_value='0.2',
        description='Linear velocity for the robot driver'
    )

    # Define the node with parameters
    turtlebot3_driver = Node(
        package='module_2_assignment',
        executable='task1_drive_turtle',
        name='task1_drive_turtle',
        output='screen',
        parameters=[
            {'b': LaunchConfiguration('b')},
            {'motion_type': LaunchConfiguration('motion_type')},
            {'linear_velocity': LaunchConfiguration('linear_velocity')},
            {'cmd_vel_topic': '/cmd_vel'},
        ]
    )

    return LaunchDescription([
        declare_b,
        declare_motion_type,
        declare_linear_velocity,
        turtlebot3_driver,
    ])






# ros2 launch module_2_assignment task2_turtle_bot_drive.launch.py b:=0.5 motion_type:=logarithmic linear_velocity:=0.9