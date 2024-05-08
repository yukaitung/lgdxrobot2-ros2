import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
import os

def generate_launch_description():
  package_dir = get_package_share_directory('lgdxrobot2_webots')
  world = 'world.wbt'
  use_sim_time = LaunchConfiguration('use_sim_time')
  robot_description_path = os.path.join(package_dir, 'resource', 'lgdxrobot2.urdf')
  
  webots = WebotsLauncher(
    world=PathJoinSubstitution([package_dir, 'worlds', world]),
    ros2_supervisor=True
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{
        'robot_description': '<robot name=""><link name=""/></robot>'
    }],
  )

  lgdxrobot2_driver = WebotsController(
    robot_name='lgdxrobot2',
    parameters=[
      {
        'robot_description': robot_description_path,
        'use_sim_time': use_sim_time
      },
    ],
    respawn=True
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      name='use_sim_time',
      default_value='True',
      description='Use the /clock topic to synchronize the ROS controller with the simulation.'
    ),
    webots,
    webots._supervisor,

    robot_state_publisher,

    lgdxrobot2_driver,

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
