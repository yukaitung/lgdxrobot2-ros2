"""\
This script initalises complete ROS2 NAV stack using lgdxodom as odometry source.

Usage: 
cd lgdx_ws # The location of the source code
. install/setup.bash
ros2 launch lgdxrobot2_navigation nav2_rtabmap_lgdxodom.launch.py
"""

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.substitutions import FindPackageShare
import os
import yaml

def generate_param_path_with_profile(file_name, profile):
    navigation_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_navigation').find('lgdxrobot2_navigation')
    path = os.path.join(navigation_pkg_share, "param", profile, file_name)
    if os.path.exists(path):
        return path
    else: # Rollback to default parameter
        return os.path.join(navigation_pkg_share, "param", file_name)

def generate_launch_description():
    profile = "rtabmap_lgdxodom"
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_description').find('lgdxrobot2_description')
    navigation_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_navigation').find('lgdxrobot2_navigation')
    default_model_path = os.path.join(description_pkg_share, 'src/description/lgdxrobot2_description.urdf')
    default_rviz_config_path = os.path.join(navigation_pkg_share, 'rviz/default.rviz')

    # Robot visualisation
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
    # LGDX Core
    lgdxrobot2_mcu_node = launch_ros.actions.Node(
        package='lgdxrobot2_mcu',
        executable='lgdxrobot2_mcu_node',
        output='screen',
        parameters=[generate_param_path_with_profile("lgdxrobot2_mcu_node.yaml", profile)]
    )
    # Camera, IMU filter
    realsense2_camera_node = launch_ros.actions.Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        namespace='camera',
        parameters=[generate_param_path_with_profile("realsense2_camera.yaml", profile)]
    )
    imu_transformer = launch_ros.actions.Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        output='screen',
        remappings=[('/imu_in', '/camera/imu')]
    )
    imu_filter_madgwick_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        remappings=[('/imu/data_raw', '/imu_out')],
        parameters=[generate_param_path_with_profile("imu_filter_madgwick.yaml", profile)]
    )
    # Rtabmap
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rtabmap_launch"), '/launch', '/rtabmap.launch.py']
        ),
        launch_arguments=yaml.load(open(generate_param_path_with_profile("rtabmap.yaml", profile)), Loader=yaml.FullLoader).items()
    )
    # State Estimation Nodes
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[generate_param_path_with_profile("ekf.yaml", profile)]
    )
    # NAV2
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py']
        ),
        launch_arguments={'params_file': generate_param_path_with_profile("nav2.yaml", profile)}.items()
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        
        lgdxrobot2_mcu_node,
        realsense2_camera_node,
        imu_transformer,
        imu_filter_madgwick_node,

        rtabmap_node,
        robot_localization_node,
        nav2_node
    ])