"""Gazebo Classic + Waffle Pi + static map/AMCL + Nav2, without hardware nodes."""
import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    share = Path(get_package_share_directory('namo_sim'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    scenario = LaunchConfiguration('scenario').perform(context).upper()
    if scenario not in ('A', 'B'):
        raise ValueError('scenario must be A or B')
    truth = lambda arg: LaunchConfiguration(arg).perform(context).lower() == 'true'
    params = str(share / 'config' / 'nav2.yaml')
    sim = {'use_sim_time': True}
    actions = [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', str(tb3 / 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        ExecuteProcess(cmd=['gzserver', '--verbose', str(share / 'worlds' / f'{scenario}.world'),
                            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                       output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': (tb3 / 'urdf' / 'turtlebot3_waffle_pi.urdf').read_text()}],
             output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'turtlebot3', '-file', str(share / 'models' / 'waffle_pi.sdf'),
                        '-x', '0', '-y', '-2', '-z', '0.01', '-Y', '1.5707963267948966', '-timeout', '90'],
             parameters=[sim], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': str(share / 'maps' / f'{scenario}.yaml')}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[params], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], remappings=[('cmd_vel', '/cmd_vel_nav')], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, {'default_nav_to_pose_bt_xml': str(share / 'config' / 'navigate.xml'),
                                  'default_nav_through_poses_bt_xml': str(share / 'config' / 'navigate_through.xml')}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_namo',
             parameters=[sim, {'autostart': True, 'node_names': ['map_server', 'amcl', 'controller_server',
                                                               'planner_server', 'bt_navigator'],
                               'bond_timeout': 10.0}], output='screen'),
        Node(package='namo_sim', executable='velocity_gate', name='velocity_gate', parameters=[sim], output='screen'),
        Node(package='namo_sim', executable='scan_clearing', name='scan_clearing', parameters=[sim], output='screen'),
    ]
    if truth('obstacle'):
        actions.append(Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'movable_obstacle', '-file', str(share / 'models' / 'obstacle.sdf'),
                                       '-x', LaunchConfiguration('obstacle_x').perform(context),
                                       '-y', LaunchConfiguration('obstacle_y').perform(context), '-z', '0.3', '-timeout', '90'],
                            parameters=[sim], output='screen'))
    if truth('gui'):
        actions.append(ExecuteProcess(cmd=['gzclient'], output='screen'))
    if truth('rviz'):
        rviz_config = share / 'rviz' / 'namo.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[sim],
                            arguments=['-d', str(rviz_config)] if rviz_config.exists() else [], output='screen'))
    if truth('mission'):
        mission_params = share / 'config' / 'mission.yaml'
        configured=yaml.safe_load(mission_params.read_text())
        mission_config=dict(configured['mission']['ros__parameters'],use_sim_time=True,scenario=scenario)
        remover_config=dict(configured['mock_remover']['ros__parameters'],use_sim_time=True)
        actions.append(Node(package='namo_sim', executable='mission', name='mission',
                            parameters=[mission_config], output='screen'))
        actions.append(Node(package='namo_sim', executable='mock_remover', name='mock_remover',
                            parameters=[remover_config], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='A'),
        DeclareLaunchArgument('obstacle', default_value='false'),
        DeclareLaunchArgument('obstacle_x', default_value='2.4'),
        DeclareLaunchArgument('obstacle_y', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mission', default_value='false'),
        OpaqueFunction(function=_launch),
    ])
