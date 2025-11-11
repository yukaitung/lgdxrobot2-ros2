"""\
This script initalises LGDXRobot2 Webots simulation, RViz visualision and ROS2 Nav2 stack.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash

# NAV2 SLAM
ros2 launch lgdxrobot2_bringup simulation_nav.launch.py slam:=True profile:='sim-slam'
ros2 launch lgdxrobot2_bringup simulation_nav.launch.py slam:=True use_explore_lite:=True profile:='sim-slam'

# NAV2 SLAM With cloud
ros2 launch lgdxrobot2_bringup simulation_nav.launch.py slam:=True use_cloud:=True profile:='sim-slam' cloud_address:='192.168.1.10:5162'

# NAV2 Localisation
ros2 launch lgdxrobot2_bringup simulation_nav.launch.py

# NAV2 Localisation With cloud
ros2 launch lgdxrobot2_bringup simulation_nav.launch.py use_cloud:=True cloud_address:='192.168.1.10:5162'
"""

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
from lgdxrobot2_bringup.utils import get_param_path
import os

launch_args = [
  # Common
  DeclareLaunchArgument(
    name='profile',
    default_value='sim-loc',
    description='Parameters profile.'
  ),
  DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Namespace for the robot.'
  ),
  
  # Webots
  DeclareLaunchArgument(
    name='world',
    default_value='default.wbt',
    description='World file in `lgdxrobot2_webots` package.'
  ),
  
  # NAV2
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
    default_value='default.yaml',
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
    name='use_explore_lite', 
    default_value='False',
    description='Launch explore_lite to explore the map automatically.'
  ),
  
  # Display
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
  
  # Cloud
  DeclareLaunchArgument(
    name='use_cloud',
    default_value='False',
    description='Whether to enable cloud.'
  ),
  DeclareLaunchArgument(
    name='cloud_address',
    default_value='host.docker.internal:5162',
    description='Address of LGDXRobot Cloud.'
  )
]
      
def launch_setup(context):
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  nav2_package_dir = get_package_share_directory('nav2_bringup')
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  webots_package_dir = get_package_share_directory('lgdxrobot2_webots')
  robot_description_path = os.path.join(webots_package_dir, 'resource', 'lgdxrobot2.urdf')
  
  # Common
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'
  
  # Webots
  world = LaunchConfiguration('world')
  
  # NAV2
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  use_explore_lite = LaunchConfiguration('use_explore_lite')
  
  # Display
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config:
    rviz_config = os.path.join(package_dir, 'rviz', profile_str) + '.rviz'
  
  # Cloud
  use_cloud = LaunchConfiguration('use_cloud')
  cloud_address = LaunchConfiguration('cloud_address').perform(context)
  
  #
  # Webots Simulator
  #
  webots = WebotsLauncher(
    world=PathJoinSubstitution([webots_package_dir, 'worlds', world]),
    ros2_supervisor=True
  )
  lgdxrobot2_driver = WebotsController(
    robot_name='robot1',
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
      ('/agent/robot_data', 'agent/robot_data'),
      ('/remove_urdf_robot', 'remove_urdf_robot')
    ],
    respawn=True
  )
  
  #
  # Base
  #
  description_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_sim_time': use_sim_time,
      'use_joint_state_publisher': 'False',
      'use_rviz': use_rviz,
      'rviz_config': rviz_config,
    }.items(),
  )
  lgdxrobot2_agent_node = Node(
    package='lgdxrobot2_agent',
    executable='lgdxrobot2_agent_node',
    namespace=namespace,
    output='screen',
    condition=IfCondition(use_cloud),
    parameters=[{
      'cloud_enable': True,
      'cloud_address': cloud_address,
      'cloud_root_cert': '/config/keys/root.crt',
      'cloud_client_key': '/config/keys/Robot1.key',
      'cloud_client_cert': '/config/keys/Robot1.crt',
      'cloud_slam_enable': slam,
      'sim_enable': True,
    }],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/agent/auto_task', 'agent/auto_task'),
      ('/agent/robot_data', 'agent/robot_data'),
    ],
  )
  lgdxrobot2_agent = TimerAction(period=15.0, actions=[lgdxrobot2_agent_node])
  
  #
  # NAV2
  #
  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    namespace=namespace,
    output='screen',
    parameters=[
      get_param_path('ekf.yaml', profile_str, namespace),
      {'use_sim_time': use_sim_time }
    ],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static')
    ]
  )
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
      'params_file': get_param_path('nav2.yaml', profile_str, namespace),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )
  explore_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(package_dir, 'launch', 'explore.launch.py')
    ),
    condition=IfCondition(use_explore_lite),
    launch_arguments={
      'config': get_param_path('explore_node.yaml', profile_str, namespace),
      'namespace': namespace,
      'use_sim_time': use_sim_time
    }.items()
  )
  
  waiting_nodes = WaitForControllerConnection(
    target_driver = lgdxrobot2_driver,
    nodes_to_start = [description_node, robot_localization_node, explore_node, ros2_nav, lgdxrobot2_agent]
  )

  return [webots, webots._supervisor, lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld