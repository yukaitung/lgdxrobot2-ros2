import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_description').find('lgdxrobot2_description')
    navigation_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_navigation').find('lgdxrobot2_navigation')
    slam_toolbox_pkg_share = launch_ros.substitutions.FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    default_model_path = os.path.join(description_pkg_share, 'src/description/lgdxrobot2_description.urdf')
    default_rviz_config_path = os.path.join(navigation_pkg_share, 'rviz/default.rviz')

    # Camera, IMU, Scan
    realsense2_camera_node = launch_ros.actions.Node(
        package='realsense2_camera',
        namespace='camera',
        name='camera',
        executable='realsense2_camera_node',
        output='screen',
        parameters=[os.path.join(navigation_pkg_share, 'param/realsense2_camera.yaml')]
    )
    imu_filter_madgwick_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        remappings=[('/imu/data_raw', '/camera/imu')],
        parameters=[os.path.join(navigation_pkg_share, 'param/imu_filter_madgwick.yaml')]
    )
    depthimage_to_laserscan_node = launch_ros.actions.Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        remappings=[('depth', '/camera/depth/image_rect_raw'),
                    ('depth_camera_info', '/camera/depth/camera_info')],
        parameters=[os.path.join(navigation_pkg_share, 'param/depthimage_to_laserscan.yaml')]
    )
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
    # Robot navigation
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(navigation_pkg_share, 'param/ekf.yaml')]
    )
    async_slam_toolbox_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(slam_toolbox_pkg_share, 'config/apper_params_online_async.yaml')]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        realsense2_camera_node,
        imu_filter_madgwick_node,
        depthimage_to_laserscan_node,

        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        
        robot_localization_node,
        async_slam_toolbox_node
    ])