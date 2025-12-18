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
    default_value='loc-sim',
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
  DeclareLaunchArgument(
    name='cloud_root_cert',
    default_value='/config/keys/root.crt',
    description='Path to the server’s root certificate'
  ),
  DeclareLaunchArgument(
    name='cloud_client_key',
    default_value='/config/keys/Robot1.key',
    description='Path to the client’s key file'
  ),
  DeclareLaunchArgument(
    name='cloud_client_cert',
    default_value='/config/keys/Robot1.crt',
    description='Path to the client’s crt file'
  )
]
      
def launch_setup(context):
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
  webots_package_dir = get_package_share_directory('lgdxrobot2_webots')
  robot_description_path = os.path.join(webots_package_dir, 'resource', 'lgdxrobot2.urdf')
  
  # Common
  profiles_path = LaunchConfiguration('profiles_path').perform(context)
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'
  p = ParamManager(profiles_path, profile_str, namespace)
  
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
  
  # Display
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config:
    rviz_config = p.get_rviz_config()
  
  # Cloud
  use_cloud = LaunchConfiguration('use_cloud')
  cloud_address = LaunchConfiguration('cloud_address').perform(context)
  cloud_client_key = LaunchConfiguration('cloud_client_key').perform(context)
  cloud_client_cert = LaunchConfiguration('cloud_client_cert').perform(context)
  cloud_root_cert = LaunchConfiguration('cloud_root_cert').perform(context)
  
  #
  # Webots Simulator
  #
  webots = WebotsLauncher(
    world=PathJoinSubstitution([webots_package_dir, 'worlds', world]),
    ros2_supervisor=True
  )
  lgdxrobot2_driver = WebotsController(
    robot_name='Robot1',
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
      'cloud_root_cert': cloud_root_cert,
      'cloud_client_key': cloud_client_key,
      'cloud_client_cert': cloud_client_cert,
      'sim_enable': True,
      'cloud_slam_enable': slam,
    }],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/agent/auto_task', 'agent/auto_task'),
      ('/agent/robot_data', 'agent/robot_data'),
    ],
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
      p.get_param_path('ekf.yaml'),
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
      'params_file': p.get_param_path('nav2.yaml'),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )
  
  waiting_nodes = WaitForControllerConnection(
    target_driver = lgdxrobot2_driver,
    nodes_to_start = [description_node, robot_localization_node, ros2_nav, lgdxrobot2_agent_node]
  )

  return [webots, webots._supervisor, lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld