"""\
This script initalises LGDXRobot2 Webots simulation, RViz visualision and ROS2 Nav2 stack.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash

# NAV2 SLAM
ros2 launch lgdxrobot2_bringup nav.launch.py slam:=True profile:='slam'
ros2 launch lgdxrobot2_bringup nav.launch.py slam:=True use_explore_lite:=True profile:='slam'

# NAV2 Localisation
ros2 launch lgdxrobot2_bringup nav.launch.py
"""

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from lgdxrobot2_bringup.utils import get_param_path
import os
import yaml

launch_args = [
  # Common
  DeclareLaunchArgument(
    name='profile',
    default_value='loc',
    description='Parameters profile.'
  ),
  DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Namespace for the robot.'
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
    description='Absolute path to the map yaml file.'
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
  
  # Sensor
  DeclareLaunchArgument(
    name='use_lidar', 
    default_value='True', 
    description='Whether to enable the LiDAR.'
  ),
  DeclareLaunchArgument(
    name='lidar_model', 
    default_value='c1', 
    description='RPLIDAR model name.'
  ),
  DeclareLaunchArgument(
    name='use_camera', 
    default_value='True', 
    description='Whether to enable the camera.'
  ),
  DeclareLaunchArgument(
    name='use_joy', 
    default_value='False', 
    description='Whether to enable the joy.'
  ),
]
      
def launch_setup(context):
  # Common
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'

  # NAV2
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  use_explore_lite = LaunchConfiguration('use_explore_lite')

  # Display
  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config:
    rviz_config = os.path.join(package_dir, 'rviz', profile_str) + '.rviz'

  # Sensors
  use_lidar = LaunchConfiguration('use_lidar')
  lidar_model = LaunchConfiguration('lidar_model').perform(context)
  use_rviz = LaunchConfiguration('use_rviz')
  use_camera = LaunchConfiguration('use_camera')
  use_camera_bool = LaunchConfiguration('use_camera').perform(context).lower() == 'true'
  use_joy = LaunchConfiguration('use_joy')
  
  camera_pkg_share = ''
  if use_camera_bool:
    camera_pkg_share = get_package_share_directory('realsense2_camera')
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  lidar_pkg_share = get_package_share_directory('sllidar_ros2')
  nav2_package_dir = get_package_share_directory('nav2_bringup')
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  
  #
  # Base
  #
  description_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_joint_state_publisher': 'False',
      'use_rviz': use_rviz,
      'rviz_config': rviz_config,
    }.items(),
  )
  lgdxrobot2_agent_node = Node(
    package='lgdxrobot2_agent',
    executable='lgdxrobot2_agent_node',
    output='screen',
    parameters=[get_param_path("lgdxrobot2_agent_node.yaml", profile_str, namespace)],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/agent/auto_task', 'agent/auto_task'),
      ('/agent/robot_data', 'agent/robot_data'),
      ('/agent/odom', 'agent/odom'),
      ('/joint_states', 'joint_states'),
    ],
  )

  #
  # Sensors
  #
  lidar_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(lidar_pkg_share, 'launch', 'sllidar_' + lidar_model + '_launch.py')
    ),
    condition=IfCondition(use_lidar),
    launch_arguments={
      'frame_id': 'lidar_link'
    }.items(),
  )
  camera_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(camera_pkg_share, 'launch', 'rs_launch.py')
    ),
    condition=IfCondition(use_camera),
    launch_arguments=yaml.load(open(get_param_path("realsense2_camera.yaml", profile_str, namespace)), Loader=yaml.FullLoader).items(),
  )
  imu_filter_madgwick_node = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    output='screen',
    remappings=[(namespace + '/imu/data_raw', namespace + '/camera/imu')],
    parameters=[get_param_path("imu_filter_madgwick.yaml", profile_str, namespace)],
    condition=IfCondition(use_camera)
  )
  joy_node = Node(
    package='joy',
    executable='joy_node',
    output='screen',
    condition=IfCondition(use_joy),
  )

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
      get_param_path('ekf.yaml', profile_str, namespace)
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
      'map': map,
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
    }.items()
  )

  return [description_node, lgdxrobot2_agent_node, lidar_node, camera_node, imu_filter_madgwick_node, joy_node, robot_localization_node, ros2_nav, explore_node]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld