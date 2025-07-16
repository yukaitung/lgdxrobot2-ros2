"""\
This script initalises LGDXRobot2 MCU node, JOY node and visualise the odometry with Rviz

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_agent joy.launch.py
ros2 launch lgdxrobot2_agent joy.launch.py rviz:=false # Without Rviz
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
import os

launch_args = [
    DeclareLaunchArgument(
        name='serial_port_name', 
        default_value='', 
        description='Absolute path serial port device.'
    ),
    DeclareLaunchArgument(
        name='model', 
        default_value='',
        description='Absolute path to robot urdf file'
    ),
    DeclareLaunchArgument(
        name='rviz_config',
        default_value='', 
        description='Absolute path to rviz config file'
    ),
    DeclareLaunchArgument(
        name='use_rviz', 
        default_value='True', 
        description='Visualise the odometry with Rviz'
    ),
]

def launch_setup(context):
    description_pkg_share = get_package_share_directory('lgdxrobot2_description')
    serial_port_name = LaunchConfiguration('serial_port_name')
    model_path = LaunchConfiguration('model').perform(context)
    if not model_path:
        model_path = os.path.join(description_pkg_share, 'description', 'lgdxrobot2_description.urdf')
    rviz_config_path = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config_path:
        rviz_config_path = os.path.join(description_pkg_share, 'rviz', 'display.rviz')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model_path]),
        }]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(use_rviz),
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    lgdxrobot2_agent_node = Node(
        package='lgdxrobot2_agent',
        executable='lgdxrobot2_agent_node',
        output='screen',
        parameters=[{
            'mcu_enable': True,
            'mcu_name': serial_port_name,
            'mcu_control_mode': 'joy',
            'mcu_publish_odom': True,
            'mcu_publish_tf': True,
            'mcu_reset_transform': True
        }]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )
    return [robot_state_publisher_node, joint_state_publisher_node, rviz_node, lgdxrobot2_agent_node, joy_node]
    
def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld