
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from lgdxrobot2_bringup.utils import ParamManager
from pathlib import Path
import os

launch_args = [
    # Common
  DeclareLaunchArgument(
    name='profiles_path',
    default_value='',
    description='Absolute path to the profiles directory, or leave empty to use the default.'
  ),
  DeclareLaunchArgument(
    name='profile',
    default_value='loc-sim',
    description='Parameters profile.'
  ),
  DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Namespace for the robot.'
  ),
  
  # Gazebo
  DeclareLaunchArgument(
    name='world',
    default_value='maze.sdf',
    description='World file in `lgdxrobot2sim_gz` package.'
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
    default_value='maze.yaml',
    description='Map yaml file in `lgdxrobot2sim_gz` package.'
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
  ),
  
  # Display
  DeclareLaunchArgument(
    name='use_rviz',
    default_value='False',
    description='Launch RViz2.'
  ),
  DeclareLaunchArgument(
    name='rviz_config', 
    default_value='',
    description='The absolute path for the RViz config file.'
  ),
]

def launch_setup(context):
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  gz_package_dir = get_package_share_directory('lgdxrobot2sim_gz')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
  ros_gz_sim_package_dir = get_package_share_directory('ros_gz_sim')
  
  # Common
  profiles_path = LaunchConfiguration('profiles_path').perform(context)
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'
  p = ParamManager(profiles_path, profile_str, namespace)
  
  # Webots
  world = LaunchConfiguration('world').perform(context)
  
  # NAV2
  slam = LaunchConfiguration('slam')
  use_localization = LaunchConfiguration('use_localization')
  map = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  
  # Display
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  if not rviz_config:
    rviz_config = p.get_rviz_config()
  
  # Set Gazebo resource path
  gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=':'.join([
      str(Path(description_package_dir).parent.resolve()),
    ])
  )

  #
  # Gazebo
  #
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(ros_gz_sim_package_dir, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-v4 -r ' + os.path.join(gz_package_dir, 'worlds', world)}.items(),
  )
  gazebo_spawn = Node(package='ros_gz_sim', executable='create',
    parameters=[{
      'name': 'robot',
      'x': 0.0,
      'z': 0.0,
      'Y': 0.0,
      'topic': 'robot_description'}],
    output='screen'
  )
  gazebo_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
      '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
      '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
      '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
      '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
      '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ],
    remappings=[
      ('/cmd_vel', 'cmd_vel'),
      ('/imu', 'imu/data'),
      ('/joint_states', 'joint_states'),
      ('/odom', 'agent/odom'),
      ('/scan', 'scan'),
    ],
    output='screen'
  )
  
  #
  # Base
  #
  description_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_sim_time': use_sim_time,
      'use_joint_state_publisher': 'False',
      'use_sim_description': 'True',
      'use_rviz': use_rviz,
      'rviz_config': rviz_config,
    }.items(),
  )
  
  #
  # NAV2
  #
  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    namespace=namespace,
    output='screen',
    parameters=[
      p.get_param_path('ekf.yaml'),
      {'use_sim_time': use_sim_time }
    ],
    remappings=[
      ('/tf', 'tf'), 
      ('/tf_static', 'tf_static')
    ]
  )
  ros2_nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(nav2_package_dir, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_namespace': use_namespace,
      'slam': slam,
      'use_localization': use_localization,
      'map': PathJoinSubstitution([gz_package_dir, 'maps', map]),
      'use_sim_time': use_sim_time,
      'params_file': p.get_param_path('nav2.yaml'),
      'autostart': autostart,
      'use_composition': use_composition,
      'use_respawn': use_respawn,
    }.items(),
  )
  
  return [gz_resource_path, gazebo, gazebo_spawn, gazebo_bridge, description_node, robot_localization_node, ros2_nav]
  
def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld