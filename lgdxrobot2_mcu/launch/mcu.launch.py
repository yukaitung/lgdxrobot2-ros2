import launch
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    mcu_node = launch_ros.actions.Node(
        package='lgdxrobot2_mcu',
        executable='lgdxrobot2_mcu_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('serial_port')},
                    {'control_mode': LaunchConfiguration('control_mode')}, 
                    {'publish_odom': LaunchConfiguration('publish_odom')}, 
                    {'publish_tf': LaunchConfiguration('publish_tf')},
                    {'base_link_frame': LaunchConfiguration('base_link_frame')}]
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='serial_port', default_value='', description='Default serial port name or (Linux only) perform automated search if the port name is unspecified.'),
        launch.actions.DeclareLaunchArgument(name='control_mode', default_value='joy', description='Robot control mode, using `joy` for joystick or `cmd_vel` for ROS nav stack.'),
        launch.actions.DeclareLaunchArgument(name='publish_odom', default_value='true', description='Publishing odometry information from the chassis.'),
        launch.actions.DeclareLaunchArgument(name='publish_tf', default_value='true', description='Publishing tf information from the chassis.'),
        launch.actions.DeclareLaunchArgument(name='base_link_frame', default_value='base_link', description='Custom name for base_link frame.'),
        mcu_node,
    ])