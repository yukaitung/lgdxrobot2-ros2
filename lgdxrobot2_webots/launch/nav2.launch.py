import launch
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
import os

def generate_launch_description():
  package_dir = get_package_share_directory('lgdxrobot2_webots')
  robot_description_path = os.path.join(package_dir, 'resource', 'lgdxrobot2.urdf')
  nav2_package_dir = get_package_share_directory('lgdxrobot2_navigation')
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
      ('/cmd_vel', '/LGDXRobot2/cmd_vel'), 
      ('/odom', '/LGDXRobot2/odom'), 
      ('/tf', '/LGDXRobot2/tf'), 
      ('/tf_static', '/LGDXRobot2/tf_static'),
    ],
    respawn=True
  )
  
  nav2_nodes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(nav2_package_dir, 'launch', 'nav2_base.launch.py')
    ),
    launch_arguments={
      'use_sim_time': use_sim_time,
      'profile': 'webots'
    }.items(),
  )
  
  waiting_nodes = WaitForControllerConnection(
    target_driver=lgdxrobot2_driver,
    nodes_to_start=[nav2_nodes]
  )
  
  return LaunchDescription([
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
    ),
    
    webots,
    webots._supervisor,

    lgdxrobot2_driver,
    waiting_nodes,

    # This action will kill all nodes once the Webots simulation has exited
    launch.actions.RegisterEventHandler(
      event_handler=launch.event_handlers.OnProcessExit(
        target_action=webots,
        on_exit=[
          launch.actions.EmitEvent(event=launch.events.Shutdown())
        ],
      )
    )
  ])
