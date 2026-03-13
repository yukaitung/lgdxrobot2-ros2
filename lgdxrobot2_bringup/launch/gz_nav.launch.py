
from pathlib import Path
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from lgdxrobot2_bringup.utils import ParamManager
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
  
  # Webots
  DeclareLaunchArgument(
    name='world',
    default_value='default.wbt',
    description='World file in `lgdxrobot2sim_webots` package.'
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
    default_value='default.yaml',
    description='Map yaml file in `lgdxrobot2sim_webots` package.'
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
  
  # Cloud
  DeclareLaunchArgument(
    name='use_cloud',
    default_value='False',
    description='Whether to enable cloud.'
  ),
  DeclareLaunchArgument(
    name='cloud_address',
    default_value='host.docker.internal:5162',
    description='Address of LGDXRobot Cloud.'
  ),
  DeclareLaunchArgument(
    name='cloud_root_cert',
    default_value='/config/keys/root.crt',
    description='Path to the server’s root certificate'
  ),
  DeclareLaunchArgument(
    name='cloud_client_key',
    default_value='/config/keys/Robot1.key',
    description='Path to the client’s key file'
  ),
  DeclareLaunchArgument(
    name='cloud_client_cert',
    default_value='/config/keys/Robot1.crt',
    description='Path to the client’s crt file'
  )
]

def launch_setup(context):
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  ros_gz_sim_package = get_package_share_directory('ros_gz_sim')
  
  # Common
  profiles_path = LaunchConfiguration('profiles_path').perform(context)
  profile_str = LaunchConfiguration('profile').perform(context)
  namespace = LaunchConfiguration('namespace').perform(context)
  use_namespace = 'True' if namespace != '' else 'False'
  p = ParamManager(profiles_path, profile_str, namespace)
  
  # Webots
  world = LaunchConfiguration('world')
  
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
  
  # Cloud
  use_cloud = LaunchConfiguration('use_cloud')
  cloud_address = LaunchConfiguration('cloud_address').perform(context)
  cloud_client_key = LaunchConfiguration('cloud_client_key').perform(context)
  cloud_client_cert = LaunchConfiguration('cloud_client_cert').perform(context)
  cloud_root_cert = LaunchConfiguration('cloud_root_cert').perform(context)
  
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
      os.path.join(ros_gz_sim_package, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-v4 -r empty.sdf'}.items(),
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
      # Clock (IGN -> ROS2)
      '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
      # Joint states (IGN -> ROS2)
      '/world/empty/model/robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    ],
    remappings=[
      ('/world/empty/model/robot/joint_state', '/joint_states'),
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
      'use_rviz': use_rviz,
      'use_sim_description': 'True',
      'use_joint_state_publisher': 'False',
    }.items(),
  )
  
  return [gz_resource_path, gazebo, gazebo_spawn, gazebo_bridge, description_node]
  
def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld