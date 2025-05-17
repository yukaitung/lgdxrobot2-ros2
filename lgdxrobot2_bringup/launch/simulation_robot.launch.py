"""\
"""

from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
from launch.substitutions import Command
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
]

def generate_param_path_with_profile(file_name, profile):
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  path = os.path.join(package_dir, "param", profile, file_name)
  if os.path.exists(path):
    return path
  else:
    return os.path.join(package_dir, "param", file_name)
      
def launch_setup(context):
  webots_package_dir = get_package_share_directory('lgdxrobot2_webots')
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  nav2_package_dir = get_package_share_directory('nav2_bringup')
  robot_description_path = os.path.join(webots_package_dir, 'resource', 'lgdxrobot2.urdf')
  
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  
  lgdxrobot2_driver = WebotsController(
    robot_name=namespace,
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
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=namespace,
    parameters=[
      {'robot_description': Command(['xacro ', os.path.join(description_package_dir, 'description', 'lgdxrobot2_sim_description.urdf')])}
    ],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static')
    ]
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

  waiting_nodes = WaitForControllerConnection(
    target_driver = lgdxrobot2_driver,
    nodes_to_start = [robot_localization_node] + [robot_state_publisher_node] + [ros2_nav]
  )

  return [lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld