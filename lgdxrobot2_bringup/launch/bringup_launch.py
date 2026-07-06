from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

launch_args = [
    DeclareLaunchArgument(
        name='serial_port_name', 
        default_value='/dev/lgdxrobot2', 
        description='Serial port name for the LGDXRobot2 or default to /dev/lgdxrobot2.'
    ),
    DeclareLaunchArgument(
        name='use_joy', 
        default_value='True', 
        description='Control robot using `joy_node`.'
    ),
    DeclareLaunchArgument(
        name='use_keyboard', 
        default_value='True', 
        description='Control the robot using `teleop_twist_keyboard`. Start the node in another terminal to control the robot.'
    ),
    DeclareLaunchArgument(
        name='use_lidar', 
        default_value='True', 
        description='Whether to enable the LiDAR.'
    ),
    DeclareLaunchArgument(
        name='use_rviz', 
        default_value='False', 
        description='Visualize in RViz.'
    ),
]

def launch_setup(context):
    use_joy = LaunchConfiguration('use_joy')
    use_keyboard = LaunchConfiguration('use_keyboard')
    use_lidar = LaunchConfiguration('use_lidar')
    use_rviz = LaunchConfiguration('use_rviz')
    
    description_pkg_share = get_package_share_directory('lgdxrobot2_description')
    lidar_pkg_share = get_package_share_directory('lgdx_rplidar_c1')
    serial_port_name = LaunchConfiguration('serial_port_name')
    
    description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg_share, 'launch', 'display_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'use_joint_state_publisher': 'False',
            'use_rviz': use_rviz,
        }.items(),
    )
    lgdxrobot2_agent_node = Node(
        package='lgdxrobot2_agent',
        executable='lgdxrobot2_agent_node',
        output='screen',
        parameters=[{
            'serial_port_name': serial_port_name,
            'reset_transform': True,
            'use_joy': use_joy,
            'use_keyboard': use_keyboard,
            'publish_tf': True,
        }]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        condition=IfCondition(use_joy),
    )
    lidar_node = Node(
        package='lgdx_rplidar_c1',
        executable='rplidar_c1_node',
        output='screen',
        parameters=[{
            'frame_id': 'lidar_link'
        }]
    )
    return [description_node, lgdxrobot2_agent_node, joy_node, lidar_node]
    
def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld