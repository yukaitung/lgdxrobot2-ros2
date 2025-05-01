"""\
This script initalises complete ROS2 NAV stack using Rtabmap to generate map.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_bringup nav_rtabmap.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
import os
import yaml

launch_args = [
  DeclareLaunchArgument(
    name='model', 
    default_value='lgdxrobot2_description.urdf',
    description='Model file in `lgdxrobot2_description` package.'
  ),
  DeclareLaunchArgument(
    name='profile',
    default_value='sim-slam',
    description='Parameters profile.'
  ),
  DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Namespace for the robot.'
  ),
  DeclareLaunchArgument(
    name='world',
    default_value='apartment.wbt',
    description='World file in `lgdxrobot2_webots` package.'
  ),
  DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether run a SLAM.'
  ),
  DeclareLaunchArgument(
    name='use_localization', 
    default_value='False',
    description='Whether to enable localization or not'
  ),
  DeclareLaunchArgument(
    name='map',
    default_value='apartment.yaml',
    description='Map yaml file in `lgdxrobot2_webots` package.'
  ),
  DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
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
  DeclareLaunchArgument(
    name='use_explore_lite', 
    default_value='False',
    description='Launch explore_lite to explore the map automatically.'
  ),
  DeclareLaunchArgument(
    name='log_level', 
    default_value='info',
    description='Log levely.'
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
  profile = "nav-rtabmap"
  description_pkg_share = get_package_share_directory('lgdxrobot2_description')
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  nav2_package_dir = get_package_share_directory('nav2_bringup')
  model_path = os.path.join(description_pkg_share, 'description', LaunchConfiguration('model').perform(context))
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if LaunchConfiguration('namespace').perform(context) != '' else 'False'
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config_path = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config_path:
    rviz_config_path = os.path.join(package_dir, 'rviz', 'sim-loc') + '.rviz'
  log_level = LaunchConfiguration('log_level')

  # Robot visualisation
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', model_path])}],
  )
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
  )
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(use_rviz),
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_path],
  )
  
  # LGDX Core
  lgdxrobot2_mcu_node = Node(
    package='lgdxrobot2_daemon',
    executable='lgdxrobot2_daemon_node',
    output='screen',
    parameters=[generate_param_path_with_profile("lgdxrobot2_daemon_node.yaml", profile)],
    remappings=[('/daemon/ext_imu', '/imu/data')]
  )
  
  # Camera, IMU filter
  realsense2_camera_node = ComposableNodeContainer(
    name='camera_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='realsense2_camera',
        namespace='',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera',
        parameters=[generate_param_path_with_profile("realsense2_camera.yaml", profile)],
        extra_arguments=[{'use_intra_process_comms': True}])
      ],
    output='screen',
    emulate_tty=True, # needed for display of logs
    arguments=['--ros-args', '--log-level', log_level],
  )
  imu_transformer = Node(
    package='imu_transformer',
    executable='imu_transformer_node',
    output='screen',
    remappings=[('/imu_in', '/camera/imu')]
  )
  imu_filter_madgwick_node = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    output='screen',
    remappings=[('/imu/data_raw', '/imu_out')],
    parameters=[generate_param_path_with_profile("imu_filter_madgwick.yaml", profile)]
  )
  
  # Rtabmap
  rtabmap_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("rtabmap_launch"), 'launch', 'rtabmap.launch.py')),
    launch_arguments=yaml.load(open(generate_param_path_with_profile("rtabmap.yaml", profile)), Loader=yaml.FullLoader).items()
  )
  
  # State Estimation Nodes
  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
      generate_param_path_with_profile("ekf.yaml", profile),
      {'use_sim_time': use_sim_time }
    ]
  )
  
  # Nav2
  ros2_nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(nav2_package_dir, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_namespace': use_namespace,
      'slam': slam,
      'use_localization': use_localization,
      'use_sim_time': use_sim_time,
      'params_file': generate_param_path_with_profile('nav2.yaml', profile),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )


  return [robot_state_publisher_node, joint_state_publisher_node, rviz_node, lgdxrobot2_mcu_node, realsense2_camera_node, imu_transformer, imu_filter_madgwick_node, rtabmap_node, robot_localization_node, ros2_nav]
    
def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld