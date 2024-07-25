"""\
This script initalises LGDXRobot2 MCU node, JOY node and visualise the odometry with Rviz

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_mcu joy.launch.py
ros2 launch lgdxrobot2_mcu joy.launch.py rviz:=false # Without Rviz
"""

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
import launch_ros
import os

def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_description').find('lgdxrobot2_description')
    default_model_path = os.path.join(description_pkg_share, 'src/description/lgdxrobot2_description.urdf')
    default_rviz_config_path = os.path.join(description_pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        condition=LaunchConfigurationEquals('rviz', 'true'),
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    lgdxrobot2_mcu_node = launch_ros.actions.Node(
        package='lgdxrobot2_mcu',
        executable='lgdxrobot2_mcu_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('serial_port'),
                     'publish_odom': True,
                     'publish_tf': True}]
    )
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='serial_port', default_value='', description='Absolute path serial port device.'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='rviz', default_value='true', description='Visualise the odometry with Rviz'),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=lgdxrobot2_mcu_node,
                on_start=[robot_state_publisher_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=lgdxrobot2_mcu_node,
                on_start=[joint_state_publisher_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=lgdxrobot2_mcu_node,
                on_start=[rviz_node],
            )
        ),
        lgdxrobot2_mcu_node,
        joy_node
    ])