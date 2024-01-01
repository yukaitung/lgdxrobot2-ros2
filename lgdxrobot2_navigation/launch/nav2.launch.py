import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_description').find('lgdxrobot2_description')
    navigation_pkg_share = launch_ros.substitutions.FindPackageShare(package='lgdxrobot2_navigation').find('lgdxrobot2_navigation')
    default_model_path = os.path.join(description_pkg_share, 'src/description/lgdxrobot2_description.urdf')
    default_rviz_config_path = os.path.join(description_pkg_share, 'rviz/urdf_config.rviz')

    # Camera
    realsense2_camera_node = launch_ros.actions.Node(
        package='realsense2_camera',
        namespace='camera',
        name='camera',
        executable='realsense2_camera_node',
        parameters=[os.path.join(navigation_pkg_share, 'param/realsense2_camera.yaml')]
    )
    # Robot Visualisation
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
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(navigation_pkg_share, 'param/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Flag to enable use_sim_time'),
        realsense2_camera_node,
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        
        robot_localization_node
    ])