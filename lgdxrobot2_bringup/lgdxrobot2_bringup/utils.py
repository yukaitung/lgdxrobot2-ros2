from ament_index_python.packages import get_package_share_directory
import tempfile
import os

class ParamManager:
  ___profiles_path: str
  __profile: str
  __namespace: str
  
  def __init__(self, profiles_path: str, profile: str, namespace: str):
    if profiles_path == "":
      profiles_path = get_package_share_directory('lgdxrobot2_bringup')
    self.___profiles_path = profiles_path
    self.__profile = profile
    self.__namespace = namespace

  def __process_yaml(self, param_path: str, inital_pose_x: str, inital_pose_y: str, inital_pose_z: str, inital_pose_yaw: str) -> str:
    namespace = self.__namespace
    if namespace and not namespace.startswith('/'):
      namespace = '/' + namespace
    with open(param_path, 'r', encoding='utf-8') as file:
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
  
  def get_rviz_config(self) -> str:
    self.___profiles_path = get_package_share_directory('lgdxrobot2_bringup')
    path = os.path.join(self.___profiles_path, 'rviz', self.__profile) + '.rviz'
    if os.path.exists(path):
      return path
    else:
      # Check if the profile starts with slam-xxx, loc-xxx
      words = self.__profile.split('-', 1)
      if len(words) > 1:
        return os.path.join(self.___profiles_path, 'rviz', words[0]) + '.rviz'
      else:
        return os.path.join(self.___profiles_path, 'rviz', 'default.rviz')
      
  def __get_path_with__profile(self, file_name: str) -> str:
    path = os.path.join(self.___profiles_path, "param", self.__profile, file_name)
    if os.path.exists(path):
      return path
    else:
      return os.path.join(self.___profiles_path, "param", file_name)
    
  def get_param_path(self, file_name: str, inital_pose_x: str = '0.0', inital_pose_y: str = '0.0', inital_pose_z: str = '0.0', inital_pose_yaw: str = '0.0') -> str:
    param_path = self.__get_path_with__profile(file_name)
    return self.__process_yaml(param_path, inital_pose_x, inital_pose_y, inital_pose_z, inital_pose_yaw)
        