import launch
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_dir = get_package_share_directory('lgdxrobot2_description')
  model = LaunchConfiguration('model')
  rviz_config = LaunchConfiguration('rviz_config')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_rviz = LaunchConfiguration('use_rviz')

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
      'robot_description': Command(['xacro ', PathJoinSubstitution([package_dir, 'src', 'description', model])])
    }]
  )
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
  )
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-d', PathJoinSubstitution([package_dir, 'rviz', rviz_config])],
    condition=launch.conditions.IfCondition(use_rviz)
  )

  return launch.LaunchDescription([
    DeclareLaunchArgument(
      name='model', 
      default_value='lgdxrobot2_description.urdf',
      description='Model file in lgdxrobot2_description package.'
    ),
    DeclareLaunchArgument(
      name='rviz_config', 
      default_value='urdf_config.rviz',
      description='Rviz config file in lgdxrobot2_description package.'
    ),
    DeclareLaunchArgument(
      name='use_sim_time',
      default_value='True',
      description='Use the simulation time from Webots.'
    ),
    DeclareLaunchArgument(
      name='use_rviz',
      default_value='True',
      description='Launch RVIZ2.'
    ),
    joint_state_publisher_node,
    robot_state_publisher_node,
    rviz_node
  ])