from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os

launch_args = [
  DeclareLaunchArgument(
    name='profile',
    default_value='webots',
    description='Parameters profile.'
  ),
  DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Robot name.'
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
    'use_composition',
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
  package_dir = get_package_share_directory('lgdxrobot2_navigation')
  path = os.path.join(package_dir, "param", profile, file_name)
  if os.path.exists(path):
    return path
  else: # Rollback to default parameter
    return os.path.join(package_dir, "param", file_name)
      
def launch_setup(context):
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace')
  use_namespace = 'True' if LaunchConfiguration('namespace').perform(context) != '' else 'False'
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  bringup_dir = get_package_share_directory('lgdxrobot2_navigation')
  launch_dir = os.path.join(bringup_dir, 'launch')

  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
      generate_param_path_with_profile("ekf.yaml", profile_str),
      {'use_sim_time': use_sim_time }
    ]
  )

  ros2_nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
    launch_arguments={
      'namespace': namespace,
      'use_namespace': use_namespace,
      'slam': 'True',
      'map': '',
      'use_sim_time': use_sim_time,
      'params_file': generate_param_path_with_profile("nav2.yaml", profile_str),
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