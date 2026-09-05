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
    default_value='warehouse.yaml',
    description='Map yaml file in `lgdxrobot2sim_webots` package.'
  ),
  DeclareLaunchArgument(
    name='keepout_mask',
    default_value='',
    description='Full path to keepout mask yaml file to load.'
  ),
  DeclareLaunchArgument(
    name='speed_mask',
    default_value='',
    description='Full path to speed mask yaml file to load.'
  ),
  DeclareLaunchArgument(
    name='graph',
    default_value='',
    description='Path to the graph file to load.'
  ),
  DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
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
    name='use_intra_process_comms',
    default_value='False',
    description='Whether to use intra process communications',
  ),
  DeclareLaunchArgument(
    name='container_name',
    default_value='nav2_container',
    description='the name of container that nodes will load in if use composition',
  ),
  DeclareLaunchArgument(
    name='use_respawn',
    default_value='False',
    description='Whether to respawn if a node crashes. Applied when composition is disabled.'
  ),
  DeclareLaunchArgument(
    name='use_keepout_zones', 
    default_value='False',
    description='Whether to enable keepout zones or not'
  ),
  DeclareLaunchArgument(
    name='use_speed_zones', 
    default_value='False',
    description='Whether to enable speed zones or not'
  ),
  DeclareLaunchArgument(
    name='log_level', 
    default_value='info',
    description='log level'
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
    name='use_joy', 
    default_value='False', 
    description='Whether to enable joy pacakge.'
  ),
  DeclareLaunchArgument(
    name='use_keyboard', 
    default_value='False', 
    description='Whether to enable teleop_twist_keyboard package.'
  ),
]
      
def launch_setup(context):
  # Common
  profiles_path = LaunchConfiguration('profiles_path').perform(context)
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  p = ParamManager(profiles_path, profile_str, namespace)

  # NAV2
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map').perform(context)
  keepout_mask = LaunchConfiguration('keepout_mask')
  speed_mask = LaunchConfiguration('speed_mask')
  graph = LaunchConfiguration('graph')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')
  container_name = LaunchConfiguration('container_name')
  use_respawn = LaunchConfiguration('use_respawn')
  use_keepout_zones = LaunchConfiguration('use_keepout_zones').perform(context)
  use_speed_zones = LaunchConfiguration('use_speed_zones').perform(context)
  log_level = LaunchConfiguration('log_level')

  # Sensors
  use_joy = LaunchConfiguration('use_joy')
  use_keyboard = LaunchConfiguration('use_keyboard')
  
  # Pcakges
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
  
  # Display
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config:
    rviz_config = p.get_rviz_config()
    
  # Rewrite Nav2 params
  yaml_substitutions = {
    'KEEPOUT_ZONE_ENABLED': use_keepout_zones,
    'SPEED_ZONE_ENABLED': use_speed_zones,
    'ROS_NAMESPACE': namespace,
    'INITAL_POSE_X': '0.0',
    'INITAL_POSE_Y': '0.0',
    'INITAL_POSE_Z': '0.0',
    'INITAL_POSE_R': '0.0',
  }

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
      ('/agent/software_emergency_stop', 'agent/software_emergency_stop'),
      ('/joint_states', 'joint_states'),
    ],
  )

  #
  # Sensors
  #
  lidar_node = Node(
    package='lgdx_rplidar_c1',
    executable='rplidar_c1_node',
    output='screen',
    parameters=[{
        'frame_id': 'lidar_link'
    }]
  )
  imu_filter_madgwick_node = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    output='screen',
    remappings=[
      (namespace + '/imu/data_raw', namespace + '/agent/imu'),
      (namespace + '/imu/mag', namespace + '/agent/mag'),
    ],
    parameters=[p.get_processed_param_path("imu_filter_madgwick.yaml", yaml_substitutions)]
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
      p.get_processed_param_path('ekf.yaml', yaml_substitutions)
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
      'slam': slam,
      'use_localization': use_localization,
      'map': map,
      'keepout_mask': keepout_mask,
      'speed_mask': speed_mask,
      'graph': graph,
      'use_sim_time': use_sim_time,
      'params_file': p.get_processed_param_path('nav2.yaml', yaml_substitutions),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_intra_process_comms': use_intra_process_comms,
      'container_name': container_name,
      'use_respawn': use_respawn,
      'log_level': log_level,
      'use_keepout_zones': use_keepout_zones,
      'use_speed_zones': use_speed_zones,
    }.items(),
  )


  return [description_node, lgdxrobot2_agent_node, lidar_node, imu_filter_madgwick_node, joy_node, robot_localization_node, ros2_nav]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld