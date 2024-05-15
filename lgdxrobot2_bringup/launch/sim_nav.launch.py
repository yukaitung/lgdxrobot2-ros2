"""\
This script initalises LGDXRobot2 Webots simulation, RViz visualision and ROS2 Nav2 stack.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_bringup sim_nav.launch.py
"""

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
import os

launch_args = [
  DeclareLaunchArgument(
    name='profile',
    default_value='webots',
    description='Parameters profile.'
  ),
  DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Namespace for the robot.'
  ),
  DeclareLaunchArgument(
    name='world',
    default_value='apartment.wbt',
    description='World file in `lgdxrobot2_webots` package.'
  ),
  DeclareLaunchArgument(
    name='slam',
    default_value='True',
    description='Whether run a SLAM.'
  ),
  DeclareLaunchArgument(
    name='map',
    default_value='',
    description='The absolute path for map yaml.'
  ),
  DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use the simulation time from Webots.'
  ),
  DeclareLaunchArgument(
    name='autostart',
    default_value='True',
    description='Automatically startup the nav2 stack',
  ),
  DeclareLaunchArgument(
    name='use_composition',
    default_value='True',
    description='Whether to use composed bringup',
  ),
  DeclareLaunchArgument(
    name='use_respawn',
    default_value='False',
    description='Whether to respawn if a node crashes.'
  ),
  DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Launch RViz2.'
  ),
  DeclareLaunchArgument(
    name='rviz_config', 
    default_value='nav.rviz',
    description='The absolute path for the RViz config file.'
  ),
  DeclareLaunchArgument(
    name='use_explore_lite', 
    default_value='False',
    description='Launch explore_lite to explore the map automatically.'
  )
]

def generate_param_path_with_profile(file_name, profile):
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  path = os.path.join(package_dir, "param", profile, file_name)
  if os.path.exists(path):
    return path
  else:
    return os.path.join(package_dir, "param", file_name)
      

def launch_setup(context):
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  webots_package_dir = get_package_share_directory('lgdxrobot2_webots')
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
  robot_description_path = os.path.join(webots_package_dir, 'resource', 'lgdxrobot2.urdf')
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace')
  namespace_str = LaunchConfiguration('namespace').perform(context)
  world = LaunchConfiguration('world')
  slam = LaunchConfiguration('slam')
  map = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config')
  use_explore_lite = LaunchConfiguration('use_explore_lite')
  
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
    remappings=[
      ('/cmd_vel', namespace_str + '/cmd_vel'), 
      ('/odom', namespace_str + '/odom'), 
      ('/tf', namespace_str + '/tf'), 
      ('/tf_static', namespace_str + '/tf_static'),
      ('/camera/color/camera_info', namespace_str + '/camera/color/camera_info'),
      ('/camera/color/image_color', namespace_str + '/camera/color/image_color'),
      ('/camera/depth/camera_info', namespace_str + '/camera/depth/camera_info'),
      ('/camera/depth/image', namespace_str + '/camera/depth/image'),
      ('/camera/depth/point_cloud', namespace_str + '/camera/depth/point_cloud'),
      ('/scan', namespace_str + '/scan'),
      ('/scan/point_cloud', namespace_str + '/scan/point_cloud'),
    ],
    respawn=True
  )

  description_nodes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_sim_time': use_sim_time,
      'model': 'lgdxrobot2_simulation.urdf',
      'use_rviz': use_rviz,
      'rviz_config': PathJoinSubstitution([package_dir, 'rviz', rviz_config]),
    }.items(),
  )

  nav2_nodes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(nav2_package_dir, 'launch', 'nav2_base.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'slam': slam,
      'map': map,
      'ekf_params_file': generate_param_path_with_profile('ekf.yaml', profile_str),
      'nav2_params_file': generate_param_path_with_profile('nav2.yaml', profile_str),
      'lattice_file': generate_param_path_with_profile('lattice.json', profile_str),
      'use_sim_time': use_sim_time,
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn
    }.items(),
  )
   
  explore_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(package_dir, 'launch', 'explore.launch.py')
    ),
    condition=IfCondition(use_explore_lite),
    launch_arguments={
      'config': generate_param_path_with_profile('explore_node.yaml', profile_str),
      'namespace': namespace,
      'use_sim_time': use_sim_time
    }.items()
  )

  waiting_nodes = WaitForControllerConnection(
    target_driver=lgdxrobot2_driver,
    nodes_to_start=[description_nodes] + [nav2_nodes] + [explore_node]
  )

  return [webots, webots._supervisor, lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld
