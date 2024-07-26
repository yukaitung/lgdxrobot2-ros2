"""\
This script publishes the model for the LGDXRobot2 and RViz visualision.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_description display.launch.py
"""

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

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
    default_value='lgdxrobot2_description.urdf',
    description='Model file in `lgdxrobot2_description` package.'
  ),
  DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Launch RViz2.'
  ),
  DeclareLaunchArgument(
    name='rviz_config', 
    default_value='',
    description='The absolute path for the RViz config file.'
  ),
]

def launch_setup(context):
  package_dir = get_package_share_directory('lgdxrobot2_description')
  namespace_str = LaunchConfiguration('namespace').perform(context)
  use_sim_time = LaunchConfiguration('use_sim_time')
  model = LaunchConfiguration('model')
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config')

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
      'robot_description': Command(['xacro ', PathJoinSubstitution([package_dir, 'description', model])])
    }],
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
    name='rviz2',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-d', rviz_config],
    condition=launch.conditions.IfCondition(use_rviz)
  )

  return [robot_state_publisher_node, rviz_node]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld