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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

launch_args = [
    DeclareLaunchArgument(
        name='serial_port_name', 
        default_value='', 
        description='Absolute path serial port device.'
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
    use_rviz = LaunchConfiguration('use_rviz')

    description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(description_pkg_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={
        'use_sim_time': 'False',
        'use_joint_state_publisher': 'True',
        'use_rviz': use_rviz,
        }.items(),
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
    return [description_node, lgdxrobot2_agent_node, joy_node]
    
def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld