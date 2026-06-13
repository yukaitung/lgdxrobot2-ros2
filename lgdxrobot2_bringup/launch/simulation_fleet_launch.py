from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from lgdxrobot2_bringup.utils import ParamManager
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node
import os

launch_args = [
  # Webots
  DeclareLaunchArgument(
    name='world',
    default_value='warehouse.wbt',
    description='World file in `lgdxrobot2sim_webots` package.'
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
  
  # Simulation Settings
  DeclareLaunchArgument(
    name='robot_count',
    default_value='2',
    description='Number of robots to be spawned, maximum is 5.'
  ),
]

def launch_setup(context):
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  webots_package_dir = get_package_share_directory('lgdxrobot2sim_webots')
  p = ParamManager("", "", "")
  
  # Webots
  world = LaunchConfiguration('world').perform(context)
  
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
  cloud_address = LaunchConfiguration('cloud_address').perform(context)
  
  # Simulation Settings
  robot_count = int(LaunchConfiguration('robot_count').perform(context))
  
  #
  # Webots Simulator
  #
  webots = WebotsLauncher(
    world=p.get_processed_webots_world_path(world, robot_count),
    ros2_supervisor=True
  )
  
  #
  # NAV2
  #
  robots = []
  for i in range(robot_count):
    r = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(package_dir, 'launch', 'simulation_fleet_base_launch.py')
      ),
      launch_arguments={
        'namespace': 'Robot' + str(i + 1),
        'slam': slam,
        'use_localization': use_localization,
        'map': map_path,
        'keepout_mask': keepout_mask,
        'speed_mask': speed_mask,
        'graph': graph,
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'use_composition': use_composition,
        'use_respawn': use_respawn,
        'log_level': log_level,
        'use_keepout_zones': use_keepout_zones,
        'use_speed_zones': use_speed_zones,
        'use_cloud': use_cloud,
        'cloud_address': cloud_address,
        'initial_pose_y': str(i * 1.5),
      }.items(),
    )
    robots.append(r)

  return [webots, webots._supervisor] + robots

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld