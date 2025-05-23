"""\
For LGDXRobot2 Development
Make sure all certificates are stored in correct place
  
Usage: 
cd lgdx_ws 
. install/setup.bash
ros2 launch lgdxrobot2_bringup simulation_two_robot.launch.py
"""

from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node
import os

launch_args = [
  DeclareLaunchArgument(
    name='world',
    default_value='two_robots.wbt',
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
    default_value='two_robots.yaml',
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
  
  world = LaunchConfiguration('world')
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  
  webots = WebotsLauncher(
    world=PathJoinSubstitution([webots_package_dir, 'worlds', world]),
    ros2_supervisor=True
  )
  
  lgdxrobot2_mcu_node1 = Node(
    package='lgdxrobot2_daemon',
    executable='lgdxrobot2_daemon_node',
    namespace='robot1',
    output='screen',
    parameters=[{
      'cloud_enable': True,
      'cloud_address': '192.168.1.10:5162',
      'cloud_root_cert': '/home/user/key/rootCA.crt',
      'cloud_client_key': '/home/user/key/Robot1.key',
      'cloud_client_cert': '/home/user/key/Robot1.crt',
    }],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/daemon/auto_task', 'daemon/auto_task'), 
      ('/daemon/crtitcal_status', 'daemon/crtitcal_status'), 
    ],
  )
  
  lgdxrobot2_mcu_node2 = Node(
    package='lgdxrobot2_daemon',
    executable='lgdxrobot2_daemon_node',
    namespace='robot2',
    output='screen',
    parameters=[{
      'cloud_enable': True,
      'cloud_address': '192.168.1.10:5162',
      'cloud_root_cert': '/home/user/key/rootCA.crt',
      'cloud_client_key': '/home/user/key/Robot2.key',
      'cloud_client_cert': '/home/user/key/Robot2.crt',
    }],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static'),
      ('/daemon/auto_task', 'daemon/auto_task'), 
      ('/daemon/crtitcal_status', 'daemon/crtitcal_status'), 
    ],
  )
  
  robot1 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(package_dir, 'launch', 'simulation_robot.launch.py')
    ),
    launch_arguments={
      'profile': 'robot1',
      'namespace': 'robot1',
      'slam': slam,
      'use_localization': use_localization,
      'map': map,
      'use_sim_time': use_sim_time,
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )
  
  robot2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(package_dir, 'launch', 'simulation_robot.launch.py')
    ),
    launch_arguments={
      'profile': 'robot2',
      'namespace': 'robot2',
      'slam': slam,
      'use_localization': use_localization,
      'map': map,
      'use_sim_time': use_sim_time,
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )

  return [webots, webots._supervisor, robot1, robot2, lgdxrobot2_mcu_node1, lgdxrobot2_mcu_node2]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld