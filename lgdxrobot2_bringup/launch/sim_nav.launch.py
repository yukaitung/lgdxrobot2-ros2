"""\
This script initalises LGDXRobot2 Webots simulation, RViz visualision and ROS2 Nav2 stack.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_bringup sim_nav.launch.py slam:=True
ros2 launch lgdxrobot2_bringup sim_nav.launch.py slam:=True use_explore_lite:=True
ros2 launch lgdxrobot2_bringup sim_nav.launch.py profile:='sim-loc'

ros2 launch lgdxrobot2_bringup sim_nav.launch.py slam:=True profile:='sim-slam-mynamespace' namespace:='mynamespace' use_rviz:='False'
"""

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
import os

launch_args = [
  DeclareLaunchArgument(
    name='profile',
    default_value='sim-slam',
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
    default_value='False',
    description='Whether run a SLAM.'
  ),
  DeclareLaunchArgument(
    name='use_localization', 
    default_value='True',
    description='Whether to enable localization or not'
  ),
  DeclareLaunchArgument(
    name='map',
    default_value='apartment.yaml',
    description='Map yaml file in `lgdxrobot2_webots` package.'
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
    default_value='',
    description='The absolute path for the RViz config file.'
  ),
  DeclareLaunchArgument(
    name='use_explore_lite', 
    default_value='False',
    description='Launch explore_lite to explore the map automatically.'
  ),
  DeclareLaunchArgument(
    name='log_level', 
    default_value='info',
    description='Log levely.'
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
  nav2_package_dir = get_package_share_directory('nav2_bringup')
  robot_description_path = os.path.join(webots_package_dir, 'resource', 'lgdxrobot2.urdf')
  
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'
  world = LaunchConfiguration('world')
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config:
    rviz_config = os.path.join(package_dir, 'rviz', profile_str) + '.rviz'
  use_explore_lite = LaunchConfiguration('use_explore_lite')
  
  # Webots Simulator
  webots = WebotsLauncher(
    world=PathJoinSubstitution([webots_package_dir, 'worlds', world]),
    ros2_supervisor=True
  )
  lgdxrobot2_driver = WebotsController(
    robot_name='LGDXRobot2',
    namespace=namespace,
    parameters=[
      {
        'robot_description': robot_description_path,
        'use_sim_time': use_sim_time
      },
    ],
    remappings=[
      ('/cmd_vel', 'cmd_vel'), 
      ('/odom', 'odom'), 
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/camera/color/camera_info', 'camera/color/camera_info'),
      ('/camera/color/image_color', 'camera/color/image_color'),
      ('/camera/depth/camera_info', 'camera/depth/camera_info'),
      ('/camera/depth/image', 'camera/depth/image'),
      ('/camera/depth/point_cloud', 'camera/depth/point_cloud'),
      ('/scan', 'scan'),
      ('/scan/point_cloud', 'scan/point_cloud'),
      ('/imu/data', 'imu/data'),
      ('/daemon/crtitcal_status', 'daemon/crtitcal_status'),
      ('/remove_urdf_robot', 'remove_urdf_robot')
    ],
    respawn=True
  )
  
  # Robot Description
  description_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_sim_time': use_sim_time,
      'model': 'lgdxrobot2_sim_description.urdf',
      'use_rviz': use_rviz,
      'rviz_config': rviz_config,
    }.items(),
  )
  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    namespace=namespace,
    output='screen',
    parameters=[
      generate_param_path_with_profile('ekf.yaml', profile_str),
      {'use_sim_time': use_sim_time }
    ],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static')
    ]
  )

  # Nav2
  ros2_nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(nav2_package_dir, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_namespace': use_namespace,
      'slam': slam,
      'use_localization': use_localization,
      'map': PathJoinSubstitution([webots_package_dir, 'maps', map]),
      'use_sim_time': use_sim_time,
      'params_file': generate_param_path_with_profile('nav2.yaml', profile_str),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )

  # Explore for Nav2
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
    target_driver = lgdxrobot2_driver,
    nodes_to_start = [description_node] + [robot_localization_node]  + [explore_node] + [ros2_nav]
  )

  return [webots, webots._supervisor, lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld