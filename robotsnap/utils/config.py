import json
import os, yaml, glob
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose

class Config:
    def __init__(self, d):
        for k, v in d.items():
            if isinstance(v, dict):
                v = Config(v)
            setattr(self, k, v)

    def to_dict(self):
        result = {}
        for k, v in self.__dict__.items():
            if isinstance(v, Config):
                result[k] = v.to_dict()
            else:
                result[k] = v
        return result


def get_latest_log_dir(base_dir, name_log):
        pattern = os.path.join(base_dir, f"{name_log}*")
        dirs = [d for d in glob.glob(pattern) if os.path.isdir(d)]
        if not dirs:
            return os.path.join(base_dir, name_log)
        latest_dir = max(dirs, key=os.path.getmtime)
        return latest_dir


def load_ros2_package_config(package_name, relative_path):
    pkg_path = get_package_share_directory(package_name)
    config_path = os.path.join(pkg_path, relative_path)
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)
    return Config(data)

def get_package_src_directory(ros_package_name: str, src_folder_name: str = None):
    """
    Retourne le chemin vers le dossier src du package ROS2.
    - ros_package_name : nom déclaré dans package.xml (ex: 'robotsnap')
    - src_folder_name : nom du dossier dans ros2_ws/src (ex: 'robotsnap_pkg')
    """
    share_dir = Path(get_package_share_directory(ros_package_name))
    # Exemple : /home/user/ros2_ws/install/robotsnap/share/robotsnap
    workspace_root = share_dir.parents[3]  # ros2_ws/
    src_root = workspace_root / 'src'

    # Si on précise le nom du dossier source, on l’utilise directement
    if src_folder_name:
        candidate = src_root / src_folder_name
    else:
        # sinon, on suppose que le dossier source porte le même nom que le package
        candidate = src_root / ros_package_name

    if not candidate.exists():
        raise FileNotFoundError(f"Impossible de trouver le dossier source : {candidate}")
    return candidate

class RosJsonEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Pose):
            return {
                "position": {
                    "x": obj.position.x,
                    "y": obj.position.y,
                    "z": obj.position.z,
                },
                "orientation": {
                    "x": obj.orientation.x,
                    "y": obj.orientation.y,
                    "z": obj.orientation.z,
                    "w": obj.orientation.w,
                }
            }
        if hasattr(obj, "tolist"):
            return obj.tolist()
        return super().default(obj)