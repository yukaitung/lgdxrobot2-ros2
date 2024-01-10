import launch
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    mcu_node = launch_ros.actions.Node(
        package='lgdxrobot2_mcu',
        executable='mcu_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('serial_port')}, {'control_mode': LaunchConfiguration('control_mode')}]
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='serial_port', default_value='', description='Default serial port name or automated search if unspecified.'),
        launch.actions.DeclareLaunchArgument(name='control_mode', default_value='', description='Robot control mode, using `joy` / unspecified for joystick or `cmd_vel` for ROS nav stack.'),
        mcu_node,
    ])