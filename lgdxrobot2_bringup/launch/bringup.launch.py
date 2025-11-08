"""\
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

launch_args = [
    DeclareLaunchArgument(
        name='serial_port_name', 
        default_value='', 
        description='Absolute path to the serial port device.'
    ),
    DeclareLaunchArgument(
        name='use_joy', 
        default_value='True', 
        description='Whether to enable the joy.'
    ),
    DeclareLaunchArgument(
        name='use_lidar', 
        default_value='True', 
        description='Whether to enable the LiDAR.'
    ),
    DeclareLaunchArgument(
        name='lidar_model', 
        default_value='c1', 
        description='RPLIDAR model name.'
    ),
    DeclareLaunchArgument(
        name='use_camera', 
        default_value='True', 
        description='Whether to enable the camera.'
    ),
    DeclareLaunchArgument(
        name='use_rviz', 
        default_value='True', 
        description='Visualize in RViz.'
    ),
]

def launch_setup(context):
    description_pkg_share = get_package_share_directory('lgdxrobot2_description')
    lidar_pkg_share = get_package_share_directory('sllidar_ros2')
    camera_pkg_share = get_package_share_directory('realsense2_camera')
    serial_port_name = LaunchConfiguration('serial_port_name')
    use_joy = LaunchConfiguration('use_joy')
    use_lidar = LaunchConfiguration('use_lidar')
    lidar_model = LaunchConfiguration('lidar_model').perform(context)
    use_camera = LaunchConfiguration('use_camera')
    use_rviz = LaunchConfiguration('use_rviz')

    description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg_share, 'launch', 'display.launch.py')
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
            'mcu_enable': True,
            'mcu_name': serial_port_name,
            'mcu_control_mode': 'joy',
            'mcu_publish_odom': True,
            'mcu_publish_tf': True,
            'mcu_reset_transform': True,
            'mcu_publish_joint_state': True
        }]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        condition=IfCondition(use_joy),
    )
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_share, 'launch', 'sllidar_' + lidar_model + '_launch.py')
        ),
        condition=IfCondition(use_lidar),
        launch_arguments={
            'frame_id': 'lidar_link'
        }.items(),
    )
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg_share, 'launch', 'rs_launch.py')
        ),
        condition=IfCondition(use_camera),
        launch_arguments={
            'camera_namespace': '',
            'depth_module.depth_profile': '1280x720x30',
            'pointcloud.enable': 'True',
            'enable_accel': 'True',
            'enable_gyro': 'True',
            'unite_imu_method': '2'
        }.items(),
    )
    return [description_node, lgdxrobot2_agent_node, joy_node, lidar_node, camera_node]
    
def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld