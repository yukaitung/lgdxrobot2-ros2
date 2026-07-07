from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from lgdxrobot2_bringup.utils import ParamManager
import os

launch_args = [
  # Common
  DeclareLaunchArgument(
    name='profiles_path',
    default_value='',
    description='Absolute path to the profiles directory, or leave empty to use the default.'
  ),
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

  # Display
  DeclareLaunchArgument(
    name='use_rviz',
    default_value='False',
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
    name='use_joy', 
    default_value='False', 
    description='Whether to enable the joy.'
  ),
  DeclareLaunchArgument(
      name='use_keyboard', 
      default_value='True', 
      description='Control the robot using `teleop_twist_keyboard`. Start the node in another terminal to control the robot.'
  ),
]
      
def launch_setup(context):
  # Common
  profiles_path = LaunchConfiguration('profiles_path').perform(context)
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'
  p = ParamManager(profiles_path, profile_str, namespace)

  # NAV2
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')

  # Sensors
  use_lidar = LaunchConfiguration('use_lidar')
  lidar_model = LaunchConfiguration('lidar_model').perform(context)
  use_joy = LaunchConfiguration('use_joy')
  use_keyboard = LaunchConfiguration('use_keyboard')
  
  # Pcakges
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  lidar_pkg_share = get_package_share_directory('sllidar_ros2')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
  
  # Display
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config:
    rviz_config = p.get_rviz_config()

  #
  # Base
  #
  description_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display_launch.py')
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
    parameters=[{
      'reset_transform': True,
      'use_joy': use_joy,
      'use_keyboard': use_keyboard,
    }],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/agent/system', 'agent/system'),
      ('/agent/odom', 'agent/odom'),
      ('/agent/imu', 'agent/imu'),
      ('/agent/mag', 'agent/mag'),
      ('/agent/mag', 'agent/software_emergency_stop'),
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
    }.items()
  )
  imu_filter_madgwick_node = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    output='screen',
    remappings=[
      (namespace + '/imu/data_raw', namespace + '/agent/imu'),
      (namespace + '/imu/mag', namespace + '/agent/mag'),
    ],
    parameters=[p.get_param_path("imu_filter_madgwick.yaml")]
  )
  joy_node = Node(
    package='joy',
    executable='joy_node',
    output='screen',
    condition=IfCondition(use_joy),
    remappings=[
      ('/joy', 'joy')
    ]
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
      p.get_param_path('ekf.yaml')
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
      'params_file': p.get_param_path('nav2.yaml'),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )

  return [description_node, lgdxrobot2_agent_node, lidar_node, imu_filter_madgwick_node, joy_node, robot_localization_node, ros2_nav]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld