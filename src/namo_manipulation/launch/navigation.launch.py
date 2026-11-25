"""Nav2 for the arm-capable robot; never starts hardware drivers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share=Path(get_package_share_directory('namo_manipulation'))
    old=Path(get_package_share_directory('namo_sim'))
    scenario=LaunchConfiguration('scenario').perform(context)
    params=str(share/'config/nav2.yaml')
    sim={'use_sim_time':True}
    return [
        Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[params,{'yaml_filename':str(share/'maps'/f'{scenario}.yaml')}],output='screen'),
        Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[params],output='screen'),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',parameters=[params],remappings=[('cmd_vel','/cmd_vel_nav')],output='screen'),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params],output='screen'),
        Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',parameters=[params,{'default_nav_to_pose_bt_xml':str(share/'config/navigate.xml'),'default_nav_through_poses_bt_xml':str(old/'config/navigate_through.xml')}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_namo',parameters=[sim,{'autostart':True,'node_names':['map_server','amcl','controller_server','planner_server','bt_navigator'],'bond_timeout':10.}],output='screen'),
        Node(package='namo_manipulation',executable='drive_gate',parameters=[sim],output='screen'),
        Node(package='namo_sim',executable='scan_clearing',parameters=[sim],output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scenario',default_value='A'),OpaqueFunction(function=nodes)])
