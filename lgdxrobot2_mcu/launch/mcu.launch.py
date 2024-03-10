"""\
This script initalises LGDXRobot2 MCU node only.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_mcu mcu.launch.py
"""

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    lgdxrobot2_mcu_node = launch_ros.actions.Node(
        package='lgdxrobot2_mcu',
        executable='lgdxrobot2_mcu_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('serial_port')},
                    {'control_mode': LaunchConfiguration('control_mode')}, 
                    {'publish_odom': LaunchConfiguration('publish_odom')}, 
                    {'publish_tf': LaunchConfiguration('publish_tf')},
                    {'base_link_frame': LaunchConfiguration('base_link_frame')},
                    {'use_external_imu': LaunchConfiguration('use_external_imu')},
                    {'reset_transform': LaunchConfiguration('reset_transform')}]
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='serial_port', default_value='', description='Default serial port name or (Linux only) perform automated search if the port name is unspecified.'),
        launch.actions.DeclareLaunchArgument(name='control_mode', default_value='joy', description='Robot control mode, using `joy` for joystick or `cmd_vel` for ROS nav stack.'),
        launch.actions.DeclareLaunchArgument(name='publish_odom', default_value='false', description='Publishing odometry information from the chassis.'),
        launch.actions.DeclareLaunchArgument(name='publish_tf', default_value='false', description='Publishing tf information from the chassis.'),
        launch.actions.DeclareLaunchArgument(name='base_link_frame', default_value='base_link', description='Custom name for base_link frame.'),
        launch.actions.DeclareLaunchArgument(name='use_external_imu', default_value='false', description='Using external IMU for odometry calcuation.'),
        launch.actions.DeclareLaunchArgument(name='reset_transform', default_value='false', description='Reset robot transform on start up.'),
        lgdxrobot2_mcu_node,
    ])