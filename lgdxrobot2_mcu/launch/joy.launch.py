import launch
from launch.substitutions import Command, LaunchConfiguration
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
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
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
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,

        mcu_node,
        joy_node
    ])