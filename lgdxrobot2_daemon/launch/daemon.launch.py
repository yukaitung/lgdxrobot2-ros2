"""\
This script initalises LGDXRobot2 Deamon node.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_daemon daemon.launch.py
"""

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    lgdxrobot2_daemon_node = launch_ros.actions.Node(
        package='lgdxrobot2_daemon',
        executable='lgdxrobot2_daemon_node',
        output='screen',
        parameters=[{'cloud_enable': LaunchConfiguration('cloud_enable')},
                    {'cloud_address': LaunchConfiguration('cloud_address')}, 
                    {'cloud_root_cert': LaunchConfiguration('cloud_client_key')}, 
                    {'cloud_client_key': LaunchConfiguration('cloud_client_key')},
                    {'cloud_client_cert': LaunchConfiguration('cloud_client_cert')},
                    {'serial_port_enable': LaunchConfiguration('serial_port_enable')},
                    {'serial_port_name': LaunchConfiguration('serial_port_name')},
                    {'serial_port_reset_transform': LaunchConfiguration('serial_port_reset_transform')},
                    {'serial_port_control_mode': LaunchConfiguration('serial_port_control_mode')},
                    {'serial_port_publish_odom': LaunchConfiguration('serial_port_publish_odom')},
                    {'serial_port_publish_tf': LaunchConfiguration('serial_port_publish_tf')},
                    {'serial_port_base_link_name': LaunchConfiguration('serial_port_base_link_name')},
                    {'serial_port_use_external_imu': LaunchConfiguration('serial_port_use_external_imu')},
                    {'sim_enable': LaunchConfiguration('sim_enable')}]
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='cloud_enable', default_value='false', description='Enable LGDXRobot2 Cloud.'),
        launch.actions.DeclareLaunchArgument(name='cloud_address', default_value='', description='Address of LGDXRobot2 Cloud.'),
        launch.actions.DeclareLaunchArgument(name='cloud_root_cert', default_value='', description='Path to server root certificate, required in LGDXRobot2 Cloud.'),
        launch.actions.DeclareLaunchArgument(name='cloud_client_key', default_value='', description='Path to client\'s private key, required in LGDXRobot2 Cloud.'),
        launch.actions.DeclareLaunchArgument(name='cloud_client_key', default_value='', description='Path to client\'s certificate chain, required in LGDXRobot2 Cloud.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_enable', default_value='false', description='Enable serial port communication for LGDXRobot2 MCU.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_name', default_value='', description='Default serial port name or (Linux only) perform automated search if the this is unspecified.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_reset_transform', default_value='false', description='Reset robot transform on start up.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_control_mode', default_value='cmd_vel', description='Robot control mode, using `joy` for joystick or `cmd_vel` for ROS nav stack.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_publish_odom', default_value='false', description='Publishing odometry information from the robot.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_publish_tf', default_value='false', description='Publishing tf information from the robot.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_base_link_name', default_value='base_link', description='Custom `base_link` name.'),
        launch.actions.DeclareLaunchArgument(name='serial_port_use_external_imu', default_value='false', description='Using external IMU for odometry calcuation.'),
        launch.actions.DeclareLaunchArgument(name='sim_enable', default_value='false', description='Enable simulation for LGDXRobot2 hardware, serial port must be disable for this feature.'),
        lgdxrobot2_daemon_node,
    ])