"""\
This script publishes the model for the LGDXRobot2 and RViz visualision.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_description display.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
import os

launch_args = [
  DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Namespace for the robot.'
  ),
  DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use the simulation time from Webots.'
  ),
  DeclareLaunchArgument(
    name='model', 
    default_value='lgdxrobot2_sim_description.urdf',
    description='Model file in `lgdxrobot2_description` package.'
  ),
  DeclareLaunchArgument(
    name='rviz_config',
    default_value='', 
    description='Absolute path to rviz config file'
  ),
  DeclareLaunchArgument(
    name='use_rviz', 
    default_value='True', 
    description='Visualise the odometry with Rviz'
  ),
]

def launch_setup(context):
  description_pkg_share = get_package_share_directory('lgdxrobot2_description')
  namespace_str = LaunchConfiguration('namespace').perform(context)
  use_sim_time = LaunchConfiguration('use_sim_time')
  model_path = os.path.join(description_pkg_share, 'description', LaunchConfiguration('model').perform(context))
  rviz_config_path = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config_path:
    rviz_config_path = os.path.join(description_pkg_share, 'rviz', 'display.rviz')
  use_rviz = LaunchConfiguration('use_rviz')

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', model_path])}],
    remappings=[
      ('/tf', namespace_str + '/tf'), 
      ('/tf_static', namespace_str + '/tf_static'),
      ('/joint_states', namespace_str + '/joint_states'), 
      ('/robot_description', namespace_str + '/robot_description')
    ]
  )
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(use_rviz),
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_path],
    parameters=[{'use_sim_time': use_sim_time}],
  )

  return [robot_state_publisher_node, rviz_node]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld