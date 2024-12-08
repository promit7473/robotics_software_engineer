from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Clear the turtlesim simulation
    clear_simulation = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/clear', 'std_srvs/srv/Empty', '{}'],
        name='clear_simulation',
        shell=True
    )

    # Main turtlesim node
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    # Spawn 5 turtles diagonally
    spawn_turtle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 1.5, y: 1.5, theta: 0.0, name: 'turtle2'}\""],
        name='spawn_turtle2',
        shell=True
    )

    spawn_turtle3 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 3.5, y: 3.5, theta: 0.0, name: 'turtle3'}\""],
        name='spawn_turtle3',
        shell=True
    )


    spawn_turtle5 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 7.5, y: 7.5, theta: 0.0, name: 'turtle5'}\""],
        name='spawn_turtle5',
        shell=True
    )

    spawn_turtle6 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 9.5, y: 9.5, theta: 0.0, name: 'turtle6'}\""],
        name='spawn_turtle6',
        shell=True
    )

    # Turtle drivers for middle 3 turtles

    turtle_driver1 = Node(
        package='module_2_assignment',
        executable='task3_driver',
        name='turtle_driver1',
        parameters=[
            {'cmd_vel_topic': '/turtle1/cmd_vel'}
        ]
    )

    turtle_driver3 = Node(
        package='module_2_assignment',
        executable='task3_driver',
        name='turtle_driver3',
        parameters=[
            {'cmd_vel_topic': '/turtle3/cmd_vel'}
        ]
    )


    turtle_driver5 = Node(
        package='module_2_assignment',
        executable='task3_driver',
        name='turtle_driver5',
        parameters=[
            {'cmd_vel_topic': '/turtle5/cmd_vel'}
        ]
    )

    return LaunchDescription([
        clear_simulation,
        turtlesim,
        spawn_turtle2,
        spawn_turtle3,
        spawn_turtle5,
        spawn_turtle6,
        turtle_driver1,
        turtle_driver3,
        turtle_driver5
    ])