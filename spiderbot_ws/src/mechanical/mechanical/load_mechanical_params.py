import os
from ament_index_python.packages import get_package_share_directory
import yaml


def load_mechanical_params() -> dict:
    package_name = 'mechanical'
    package_path = get_package_share_directory(package_name)

    yaml_file_path = os.path.join(package_path, 'config', 'robot_params.yaml')

    with open(yaml_file_path, 'r') as yaml_file:
        params = yaml.safe_load(yaml_file)

    return params
