"""One physical Waffle Pi + OpenMANIPULATOR-X + LDS02 + wrist RGB-D model."""
import os
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_model(context):
    share = Path(get_package_share_directory('namo_manipulation'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    model_roots = [tb3/'models', share.parent, Path(__file__).resolve().parents[2],
                   Path(get_package_share_directory('turtlebot3_manipulation_description')).parent,
                   Path(get_package_share_directory('realsense2_description')).parent]
    get = lambda name: LaunchConfiguration(name).perform(context)
    description = (share/'models/robot.urdf').read_text().replace(
        '__NAMO_CONTROLLERS__', str(share/'config/controllers.yaml'))
    ballast_mass = float(get('rear_ballast_mass'))
    if not math.isfinite(ballast_mass) or ballast_mass < 0:
        raise ValueError('rear_ballast_mass must be finite and nonnegative')
    robot = ET.fromstring(description)
    wheel_acceleration = float(get('wheel_acceleration'))
    if not math.isfinite(wheel_acceleration) or wheel_acceleration < 0:
        raise ValueError('wheel_acceleration must be finite and nonnegative')
    robot.find("gazebo/plugin[@name='diff_drive']/max_wheel_acceleration").text = str(wheel_acceleration)
    ballast = robot.find("link[@name='rear_ballast_link']")
    if ballast_mass == 0:
        for element in list(robot):
            if element.get('name') in ('rear_ballast_link', 'rear_ballast_joint') or element.get('reference') == 'rear_ballast_link':
                robot.remove(element)
    elif ballast is not None:
        ballast.find('inertial/mass').set('value', str(ballast_mass))
        inertia = ballast.find('inertial/inertia')
        for key, value in {'ixx': ballast_mass*(.14**2+.04**2)/12,
                           'iyy': ballast_mass*(.06**2+.04**2)/12,
                           'izz': ballast_mass*(.06**2+.14**2)/12}.items():
            inertia.set(key, str(value))
    else:
        raise RuntimeError('Regenerate the robot with a nonzero ballast before overriding its mass')
    description = ET.tostring(robot, encoding='unicode')
    sim = {'use_sim_time': True}
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'namo_robot', '-topic', '/robot_description',
                            '-x', get('robot_x'), '-y', get('robot_y'), '-z', '0.01',
                            '-Y', get('robot_yaw'), '-timeout', '90'],
                 parameters=[sim], output='screen')
    controller_spawner = Node(package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller', 'right_gripper_controller',
                   '--controller-manager', '/controller_manager', '--controller-manager-timeout', '90'],
        parameters=[sim], output='screen')
    actions = [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join(map(str, model_roots))+':'+os.environ.get('GAZEBO_MODEL_PATH','')),
        ExecuteProcess(cmd=['gzserver', '--verbose', get('world_path'),
                            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': description}], output='screen'),
        # Throttle measured state delivery, preserving the 500 Hz force loop
        # and each message's original simulation stamp and joint values.
        Node(package='topic_tools', executable='throttle', name='arm_joint_state_throttle',
             arguments=['messages', '/joint_state_broadcaster/joint_states', '50.0', '/joint_states'],
             parameters=[sim, {'use_wall_clock': True}], output='screen'),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controller_spawner])),
    ]
    can = share/'models/can.sdf'
    if get('spawn_can').lower() == 'true':
        if not can.exists():
            raise RuntimeError('Generate models/can.sdf before spawn_can:=true')
        actions.append(Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'namo_can', '-file', str(can),
                                       '-x', get('can_x'), '-y', get('can_y'), '-z', '0.0676', '-timeout','90'],
                            parameters=[sim], output='screen'))
    if get('gui').lower() == 'true':
        view_plugin = Path(get_package_prefix('namo_rviz'))/'lib/libnamo_gazebo_view.so'
        actions.append(ExecuteProcess(cmd=['gzclient', '--gui-client-plugin', str(view_plugin)], output='screen'))
    if get('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[sim],
                            arguments=['-d', get('rviz_config')], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('namo_manipulation'))
    return LaunchDescription([
        DeclareLaunchArgument('world_path', default_value=str(share/'worlds/model_check.world')),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_can', default_value='false'),
        DeclareLaunchArgument('can_x', default_value='0.30'),
        DeclareLaunchArgument('can_y', default_value='0.0'),
        DeclareLaunchArgument('rear_ballast_mass', default_value='0.0',
                              description='Visible rear ballast kg: explicit simulation assumption; 0 removes it'),
        DeclareLaunchArgument('wheel_acceleration', default_value='0.0',
                              description='Gazebo wheel-speed limiter; 0 retains finite-force motor constraint'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=str(share/'rviz/manipulation.rviz')),
        OpaqueFunction(function=launch_model),
    ])
