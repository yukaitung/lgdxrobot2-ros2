import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

launch_args = [
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
    default_value='display.rviz',
    description='RViz config file in `lgdxrobot2_description` package.'
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
      'robot_description': Command(['xacro ', PathJoinSubstitution([package_dir, 'src', 'description', model])])
    }],
    remappings=[
      ('/tf', namespace_str + '/tf'), 
      ('/tf_static', namespace_str + '/tf_static'),
      ('/joint_states', namespace_str + '/joint_states'), 
      ('/robot_description', namespace_str + '/robot_description')
    ]
  )
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    remappings=[
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
    arguments=['-d', PathJoinSubstitution([package_dir, 'rviz', rviz_config])],
    condition=launch.conditions.IfCondition(use_rviz)
  )

  return [robot_state_publisher_node, joint_state_publisher_node, rviz_node]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld