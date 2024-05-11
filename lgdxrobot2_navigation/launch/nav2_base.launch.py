from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_param_path_with_profile(file_name, profile):
  package_dir = get_package_share_directory('lgdxrobot2_navigation')
  path = os.path.join(package_dir, "param", profile, file_name)
  if os.path.exists(path):
    return path
  else: # Rollback to default parameter
    return os.path.join(package_dir, "param", file_name)
      
def generate_launch_description():
  profile = 'webots'
  use_sim_time = LaunchConfiguration('use_sim_time')

  nav2_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [FindPackageShare("nav2_bringup"), '/launch', '/bringup_launch.py']
    ),
    launch_arguments={
      'namespace': 'LGDXRobot2',
      'use_namespace': 'True',
      'slam': 'True',
      'map': '',
      'use_sim_time': use_sim_time,
      'params_file': generate_param_path_with_profile("nav2.yaml", profile)
    }.items()
  )
  
  return LaunchDescription([
    DeclareLaunchArgument(
      name='use_sim_time',
      default_value='True',
      description='Use the simulation time from Webots.'
    ),
    
    nav2_node
  ])