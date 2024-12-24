import json
import os
from datetime import datetime

class ConfigHandler:
    def __init__(self, config_dir="configs"):
        self.config_dir = config_dir
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)

    def save_config(self, values, name=None):
        """Save the current leg values to a config file"""
        if name is None:
            # Generate a timestamp-based name if none provided
            name = f"config_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Ensure .json extension
        if not name.endswith('.json'):
            name += '.json'
            
        filepath = os.path.join(self.config_dir, name)
        
        with open(filepath, 'w') as f:
            json.dump(values, f, indent=4)
        
        return filepath

    def load_config(self, filename):
        """Load leg values from a config file"""
        filepath = os.path.join(self.config_dir, filename)
        
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Config file {filepath} not found")
            
        with open(filepath, 'r') as f:
            return json.load(f)

    def list_configs(self):
        """List all available config files"""
        if not os.path.exists(self.config_dir):
            return []
        return [f for f in os.listdir(self.config_dir) if f.endswith('.json')]
