import json
import os
from datetime import datetime

class ConfigHandler:
    def __init__(self, config_dir="configs"):
        self.config_dir = config_dir
        self.default_config = "default_motor_config.json"
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
        
        # Create default config if it doesn't exist
        self.default_config_path = os.path.join(self.config_dir, self.default_config)
        if not os.path.exists(self.default_config_path):
            self.save_default_config()

    def save_config(self, values, name=None):
        """Save the current motor values to a config file"""
        if name is None:
            name = self.default_config
        
        # Ensure .json extension
        if not name.endswith('.json'):
            name += '.json'
            
        filepath = os.path.join(self.config_dir, name)
        
        with open(filepath, 'w') as f:
            json.dump(values, f, indent=4)
        
        return filepath

    def load_config(self, filename=None):
        """Load motor values from a config file"""
        if filename is None:
            filename = self.default_config
            
        filepath = os.path.join(self.config_dir, filename)
        
        if not os.path.exists(filepath):
            if filename == self.default_config:
                return self.save_default_config()
            raise FileNotFoundError(f"Config file {filepath} not found")
            
        with open(filepath, 'r') as f:
            return json.load(f)

    def save_default_config(self):
        """Create default configuration with centered values for all motors"""
        default_values = {
            'dc_motors': {
                'LDC': 0,
                'RDC': 0
            },
            'servo_motors': {}
        }
        
        # Add default servo values (90 degrees for center position)
        motor_groups = {
            'left_front': ["L1", "L2", "L3"],
            'left_center': ["L5", "L6", "L7", "L8"],
            'left_back': ["L9", "L10", "L12"],
            'right_front': ["R14", "R15", "R16"],
            'right_center': ["R6", "R8", "R10", "R12"],
            'right_back': ["R1", "R2", "R3"]
        }
        
        for group in motor_groups.values():
            for motor_id in group:
                default_values['servo_motors'][motor_id] = 90
        
        return self.save_config(default_values, self.default_config)

    def update_motor_value(self, motor_id, value):
        """Update a specific motor's value in the default config"""
        config = self.load_config()
        
        # Determine if it's a DC or servo motor
        if motor_id in ['LDC', 'RDC']:
            config['dc_motors'][motor_id] = value
        else:
            config['servo_motors'][motor_id] = value
        
        self.save_config(config)
        return config

    def list_configs(self):
        """List all available config files"""
        if not os.path.exists(self.config_dir):
            return []
        return [f for f in os.listdir(self.config_dir) if f.endswith('.json')]
