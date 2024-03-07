import launch
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    mcu_node = launch_ros.actions.Node(
        package='lgdxrobot2_mcu',
        executable='lgdxrobot2_mcu_node',
        output='screen',
        parameters=[{
                        'serial_port': LaunchConfiguration('serial_port'),
                        'publish_odom': True,
                        'publish_tf': True
                    }]
    )
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='serial_port', default_value='', description='Absolute path serial port device.'),
        mcu_node,
        joy_node
    ])