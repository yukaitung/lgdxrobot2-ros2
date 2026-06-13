from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from lgdxrobot2_bringup.utils import ParamManager

launch_args = [
  # Common
  DeclareLaunchArgument(
    name='profiles_path',
    default_value='',
    description='Absolute path to the profiles directory, or leave empty to use the default.'
  ),
  DeclareLaunchArgument(
    name='profile',
    default_value='loc-wb',
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
  
  # Initial Pose
  DeclareLaunchArgument(
    name='initial_pose_x',
    default_value='0.0',
    description='Initial pose x'
  ),
  DeclareLaunchArgument(
    name='initial_pose_y',
    default_value='0.0',
    description='Initial pose y'
  ),
  DeclareLaunchArgument(
    name='initial_pose_z',
    default_value='0.0',
    description='Initial pose z'
  ),
  DeclareLaunchArgument(
    name='initial_pose_yaw',
    default_value='0.0',
    description='Initial pose yaw'
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
  ),
  # Certificates
  # Root: root.crt
  # Client: <namespace>.key, <namespace>.crt
  DeclareLaunchArgument(
    name='cloud_cert_folder',
    default_value='/config/keys/',
    description='Path to the server’s root certificate'
  ),
]
    
def launch_setup(context):
  webots_package_dir = get_package_share_directory('lgdxrobot2sim_webots')
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
  robot_description_path = os.path.join(webots_package_dir, 'resource', 'lgdxrobot2.urdf')
  
  # Common
  profiles_path = LaunchConfiguration('profiles_path').perform(context)
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  p = ParamManager(profiles_path, profile_str, namespace)
  
  # NAV2
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map').perform(context)
  map_path = PathJoinSubstitution([webots_package_dir, 'maps', map])
  keepout_mask = LaunchConfiguration('keepout_mask')
  speed_mask = LaunchConfiguration('speed_mask')
  graph = LaunchConfiguration('graph')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  use_keepout_zones = LaunchConfiguration('use_keepout_zones').perform(context)
  use_speed_zones = LaunchConfiguration('use_speed_zones').perform(context)
  log_level = LaunchConfiguration('log_level')
  
  # Cloud
  use_cloud = LaunchConfiguration('use_cloud')
  use_cloud_str = LaunchConfiguration('use_cloud').perform(context)
  cloud_address = LaunchConfiguration('cloud_address').perform(context)
  cloud_cert_folder = LaunchConfiguration('cloud_cert_folder').perform(context)
  cloud_client_key = os.path.join(cloud_cert_folder, namespace + '.key')
  cloud_client_cert = os.path.join(cloud_cert_folder, namespace + '.crt')
  cloud_root_cert = os.path.join(cloud_cert_folder, 'rootCA.crt')
  
  # Manage map
  nav2_delay_enable = False
  if use_cloud_str.lower() == 'true':
    nav2_delay_enable = True
    graph = os.path.join(os.getcwd(), 'route.geojson')
    map_path = os.path.join(os.getcwd(), 'map.yaml')
    keepout_mask = os.path.join(os.getcwd(), 'keepout_mask.yaml')
    speed_mask = os.path.join(os.getcwd(), 'speed_mask.yaml')
    use_keepout_zones = 'True'
    use_speed_zones = 'True'
  
  # Initial Pose
  initial_pose_x = LaunchConfiguration('initial_pose_x').perform(context)
  initial_pose_y = LaunchConfiguration('initial_pose_y').perform(context)
  initial_pose_z = LaunchConfiguration('initial_pose_z').perform(context)
  initial_pose_yaw = LaunchConfiguration('initial_pose_yaw').perform(context)
  
  # Rewrite Nav2 params
  yaml_substitutions = {
    'KEEPOUT_ZONE_ENABLED': use_keepout_zones,
    'SPEED_ZONE_ENABLED': use_speed_zones,
    'ROS_NAMESPACE': namespace,
    'INITAL_POSE_X': initial_pose_x,
    'INITAL_POSE_Y': initial_pose_y,
    'INITAL_POSE_Z': initial_pose_z,
    'INITAL_POSE_R': initial_pose_yaw,
  }
  
  #
  # Webots Simulator
  #
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
      ('/agent/odom', 'agent/odom'), 
      ('/agent/imu', 'agent/imu'),
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/camera/color/camera_info', 'camera/color/camera_info'),
      ('/camera/color/image_color', 'camera/color/image_raw'),
      ('/camera/depth/camera_info', 'camera/depth/camera_info'),
      ('/camera/depth/image', 'camera/depth/image_rect_raw'),
      ('/camera/depth/point_cloud', 'camera/depth/color/points'),
      ('/scan', 'scan'),
      ('/scan/point_cloud', 'scan/point_cloud'),
      ('/remove_urdf_robot', 'remove_urdf_robot'),
      ('/agent/software_emergency_stop', 'cloud/software_emergency_stop'),
    ],
    respawn=True
  )
  
  #
  # Base
  #
  lgdxrobot_cloud_node = Node(
    package='lgdxrobot_cloud_adapter',
    executable='lgdxrobot_cloud_adapter_node',
    condition=IfCondition(use_cloud),
    namespace=namespace,
    output='screen',
    parameters=[{
      'slam_enable': slam,
      'address': cloud_address,
      'client_key': cloud_client_key,
      'client_cert': cloud_client_cert,
      'root_cert': cloud_root_cert,
    }],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static')
    ]
  )
  nav2_delay_node = Node(
    package='lgdxrobot_cloud_adapter',
    executable='nav2_delay_node',
    namespace=namespace,
    output='screen',
    parameters=[{
      'nav2_delay_enable': nav2_delay_enable,
    }],
  )
  
  #
  # Robot State Publisher
  #
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=namespace,
    parameters=[
      {'robot_description': Command(['xacro ', os.path.join(description_package_dir, 'description', 'lgdxrobot2.urdf')])},
      {'use_sim_time': use_sim_time}
    ],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static')
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
      p.get_processed_param_path('ekf.yaml', yaml_substitutions),
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
      'slam': slam,
      'use_localization': use_localization,
      'map': map_path,
      'keepout_mask': keepout_mask,
      'speed_mask': speed_mask,
      'graph': graph,
      'use_sim_time': use_sim_time,
      'params_file': p.get_processed_param_path('nav2.yaml', yaml_substitutions),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
      'log_level': log_level,
      'use_keepout_zones': use_keepout_zones,
      'use_speed_zones': use_speed_zones,
    }.items(),
  )
  
  # Nav2 delay Event
  nav2_delay_event = RegisterEventHandler(
     OnProcessExit(
      target_action=nav2_delay_node,
      on_exit=[ros2_nav, robot_localization_node, robot_state_publisher_node]
    )
  )

  waiting_nodes = WaitForControllerConnection(
    target_driver = lgdxrobot2_driver,
    nodes_to_start = [lgdxrobot_cloud_node, nav2_delay_node, nav2_delay_event]
  )

  return [lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld