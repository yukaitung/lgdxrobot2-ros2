from ament_index_python.packages import get_package_share_directory
import tempfile
import os

def process_yaml(input_path: str, namespace: str, inital_pose_x: str, inital_pose_y: str, inital_pose_z: str, inital_pose_yaw: str) -> str:
    if namespace and not namespace.startswith('/'):
      namespace = '/' + namespace
    with open(input_path, 'r', encoding='utf-8') as file:
      content = file.read()
    replaced_content = content.replace('<ROS_NAMESPACE>', namespace)
    replaced_content = replaced_content.replace('<INITAL_POSE_X>', inital_pose_x)
    replaced_content = replaced_content.replace('<INITAL_POSE_Y>', inital_pose_y)
    replaced_content = replaced_content.replace('<INITAL_POSE_Z>', inital_pose_z)
    replaced_content = replaced_content.replace('<INITAL_POSE_YAW>', inital_pose_yaw)
    temp_file = tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.yaml', encoding='utf-8')
    temp_file.write(replaced_content)
    temp_file_path = temp_file.name
    temp_file.close()
    return temp_file_path
  
def get_rviz_config_path_with_profile(profile: str) -> str:
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  path = os.path.join(package_dir, 'rviz', profile) + '.rviz'
  if os.path.exists(path):
    return path
  else:
    # Check if the profile starts with slam-xxx, loc-xxx
    words = profile.split('-', 1)
    if len(words) > 1:
      return os.path.join(package_dir, 'rviz', words[0]) + '.rviz'
    else:
      return os.path.join(package_dir, 'rviz', 'default.rviz')

def get_path_with_profile(file_name: str, profile: str) -> str:
  package_dir = get_package_share_directory('lgdxrobot2_bringup')
  path = os.path.join(package_dir, "param", profile, file_name)
  if os.path.exists(path):
    return path
  else:
    return os.path.join(package_dir, "param", file_name)

def get_param_path(file_name: str, profile: str, namespace: str, inital_pose_x: str = '0.0', inital_pose_y: str = '0.0', inital_pose_z: str = '0.0', inital_pose_yaw: str = '0.0') -> str:
  param_path = get_path_with_profile(file_name, profile)
  return process_yaml(param_path, namespace, inital_pose_x, inital_pose_y, inital_pose_z, inital_pose_yaw)
      