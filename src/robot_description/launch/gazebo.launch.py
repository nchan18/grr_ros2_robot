from gettext import install
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Process the URDF file
    install_dir = get_package_prefix('robot_description')
    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share'
    
    print("GAZEBO MODELS PATH==" + str(os.environ['GAZEBO_MODEL_PATH']))

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    position = [1.0, 1.0, 2.0]
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'examplo',
                                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2])],
                        output='screen')
    

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'diff_drive_base_controller'],
        output='screen'
    )


    # Start Rviz2 with basic view
    rviz2_config_path = os.path.join(get_package_share_directory('robot_description'), 'config/view.rviz')
    run_rviz2 = ExecuteProcess(
        cmd=['rviz2', '-d', rviz2_config_path],
        output='screen'
    )

    # Joystick Teleop
    joy = Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])

    config_filepath = os.path.join(get_package_share_directory('teleop_twist_joy'), 'config/xbox.config.yaml')
    teleop = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node', 
        parameters=[config_filepath],
        remappings={('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped')}
        )


    # Launch!
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joy,
        teleop,
        run_rviz2
    ])