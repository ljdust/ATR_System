import yaml
from pathlib import Path
# Load the main config file for all settings
def read_yaml(config_filename, key=None):
    config_filename = Path(config_filename)
    if config_filename.is_file():
        with open(config_filename, "r") as f:
            return yaml.safe_load(f)
    else:
        print(f'\033[2;31;43m ERROR: {config_filename} does not exist \033[0;0m')