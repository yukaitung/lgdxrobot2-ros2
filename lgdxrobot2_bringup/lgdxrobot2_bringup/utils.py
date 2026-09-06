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

  def __process_yaml(self, param_path: str, replace: dict = {}) -> str:
    namespace = self.__namespace
    if namespace and not namespace.startswith('/'):
      namespace = '/' + namespace
    with open(param_path, 'r', encoding='utf-8') as file:
      content = file.read()
    for key, value in replace.items():
      content = content.replace(key, value)
    temp_file = tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.yaml', encoding='utf-8')
    temp_file.write(content)
    temp_file_path = temp_file.name
    temp_file.close()
    return temp_file_path
  
  def get_rviz_config(self) -> str:
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
      
  def get_param_path(self, file_name: str) -> str:
    path = os.path.join(self.___profiles_path, "param", self.__profile, file_name)
    if os.path.exists(path):
      return path
    else:
      return os.path.join(self.___profiles_path, "param", file_name)
    
  def get_processed_param_path(self, file_name: str, replace: dict = {}) -> str:
    param_path = self.get_param_path(file_name)
    return self.__process_yaml(param_path, replace)
        
  def get_processed_webots_world_path(self, world: str, robot_count: int) -> str:
    webots_package_dir = get_package_share_directory('lgdxrobot2sim_webots')
    world_path = os.path.join(webots_package_dir, 'worlds', world)
    with open(world_path, 'r', encoding='utf-8') as file:
      content = file.read()
    content = content.replace('StartingPoint {', 'LGDXRobot2 {', robot_count)
    temp_file = tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.wbt', encoding='utf-8')
    temp_file.write(content)
    temp_file_path = temp_file.name
    temp_file.close()
    return temp_file_path
    