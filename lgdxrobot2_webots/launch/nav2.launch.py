import launch
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
import os

launch_args = [
  DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Robot name.'
  ),
  DeclareLaunchArgument(
    'world',
    default_value='world.wbt',
    description='World file in `lgdxrobot2_webots` package.'
  ),
  DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use the simulation time from Webots.'
  ),
  DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Launch RViz2.'
  )
]

def launch_setup(context):
  package_dir = get_package_share_directory('lgdxrobot2_webots')
  robot_description_path = os.path.join(package_dir, 'resource', 'lgdxrobot2.urdf')
  description_package_dir = get_package_share_directory('lgdxrobot2_description')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
  namespace = LaunchConfiguration('namespace')
  namespace_str = LaunchConfiguration('namespace').perform(context)
  world = LaunchConfiguration('world')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  webots = WebotsLauncher(
    world=PathJoinSubstitution([package_dir, 'worlds', world]),
    ros2_supervisor=True
  )

  lgdxrobot2_driver = WebotsController(
    robot_name='LGDXRobot2',
    parameters=[
      {
        'robot_description': robot_description_path,
        'use_sim_time': use_sim_time
      },
    ],
    remappings=[
      ('/cmd_vel', namespace_str + '/cmd_vel'), 
      ('/odom', namespace_str + '/odom'), 
      ('/tf', namespace_str + '/tf'), 
      ('/tf_static', namespace_str + '/tf_static'),
      ('/camera/color/camera_info', namespace_str + '/camera/color/camera_info'),
      ('/camera/color/image_color', namespace_str + '/camera/color/image_color'),
      ('/camera/depth/camera_info', namespace_str + '/camera/depth/camera_info'),
      ('/camera/depth/image', namespace_str + '/camera/depth/image'),
      ('/camera/depth/point_cloud', namespace_str + '/camera/depth/point_cloud'),
      ('/scan', namespace_str + '/scan'),
      ('/scan/point_cloud', namespace_str + '/scan/point_cloud'),
    ],
    respawn=True
  )

  description_nodes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_package_dir, 'launch', 'display.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_sim_time': use_sim_time,
      'model': 'lgdxrobot2_simulation.urdf',
      'use_rviz': 'False',
    }.items(),
  )
  
  nav2_nodes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(nav2_package_dir, 'launch', 'nav2_base.launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'use_sim_time': use_sim_time,
      'profile': 'webots'
    }.items(),
  )
  
  waiting_nodes = WaitForControllerConnection(
    target_driver=lgdxrobot2_driver,
    nodes_to_start=[description_nodes] + [nav2_nodes]
  )

  return [webots, webots._supervisor, lgdxrobot2_driver, waiting_nodes]

def generate_launch_description():
  opfunc = OpaqueFunction(function = launch_setup)
  ld = LaunchDescription(launch_args)
  ld.add_action(opfunc)
  return ld
