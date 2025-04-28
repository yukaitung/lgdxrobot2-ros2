"""\
This script initalises LGDXRobot2 Webots simulation and RViz visualision.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_bringup sim_display.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher
import os

launch_args = [
  DeclareLaunchArgument(
    name='world',
    default_value='apartment.wbt',
    description='World file in `lgdxrobot2_webots` package.'
  ),
  DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use the simulation time from Webots.'
  ),
  DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Launch RViz2.'
  ),
  DeclareLaunchArgument(
    name='rviz_config', 
    default_value='',
    description='The absolute path for the RViz config file.'
  )
]

def launch_setup(context):
  webots_package_dir = get_package_share_directory('lgdxrobot2_webots')
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  robot_description_path = os.path.join(webots_package_dir, 'resource', 'lgdxrobot2.urdf')
  world = LaunchConfiguration('world')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config')
  
  webots = WebotsLauncher(
    world=PathJoinSubstitution([webots_package_dir, 'worlds', world]),
    ros2_supervisor=True
  )

  lgdxrobot2_driver = WebotsController(
    robot_name='LGDXRobot2',
    parameters=[
      {
        'robot_description': robot_description_path,
        'use_sim_time': use_sim_time
      },
    ],
    respawn=True
  )
  
  description_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display.launch.py')
    ),
    launch_arguments={
      'use_sim_time': use_sim_time,
      'model': 'lgdxrobot2_sim_description.urdf',
      'use_rviz': use_rviz,
      'rviz_config': rviz_config,
    }.items(),
  )
  
  waiting_nodes = WaitForControllerConnection(
    target_driver=lgdxrobot2_driver,
    nodes_to_start=[description_node]
  )
  
  return [webots, webots._supervisor, lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld