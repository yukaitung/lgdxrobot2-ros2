from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
import os

launch_args = [
  DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Namespace for the robot.'
  ),
  DeclareLaunchArgument(
    name='slam',
    default_value='True',
    description='Whether run a SLAM.'
  ),
  DeclareLaunchArgument(
    name='map',
    default_value='',
    description='The absolute path for map yaml file.'
  ),
  DeclareLaunchArgument(
    name='ekf_params_file',
    default_value='',
    description='The absolute path for robot_localization_node parameters.'
  ),
  DeclareLaunchArgument(
    name='nav2_params_file',
    default_value='',
    description='The absolute path for nav2 parameters.'
  ),
  DeclareLaunchArgument(
    name='lattice_file',
    default_value='',
    description='The absolute path for lattice file.'
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

def launch_setup(context):
  namespace = LaunchConfiguration('namespace')
  use_namespace = 'True' if LaunchConfiguration('namespace').perform(context) != '' else 'False'
  slam = LaunchConfiguration('slam')
  map = LaunchConfiguration('map').perform(context)
  ekf_params_file = LaunchConfiguration('ekf_params_file')
  nav2_params_file = LaunchConfiguration('nav2_params_file')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  bringup_dir = get_package_share_directory('lgdxrobot2_navigation')
  launch_dir = os.path.join(bringup_dir, 'launch')

  lattice_file = LaunchConfiguration('lattice_file')
  nav2_params_file = ReplaceString(
    source_file=nav2_params_file,
    replacements={'<lattice_filepath>': ('/', lattice_file)},
  )

  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
      ekf_params_file,
      {'use_sim_time': use_sim_time }
    ]
  )

  ros2_nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
    launch_arguments={
      'namespace': namespace,
      'use_namespace': use_namespace,
      'slam': slam,
      'map': map,
      'use_sim_time': use_sim_time,
      'params_file': nav2_params_file,
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )

  return [robot_localization_node, ros2_nav]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld