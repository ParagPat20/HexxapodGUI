import json
import os

class ConfigHandler:
    def __init__(self, config_file='motor_config.json'):
        self.config_file = config_file
    
    def load_config(self):
        """Load motor configuration from file"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                # Ensure all required sections exist
                if 'servo_motors' not in config:
                    config['servo_motors'] = {}
                if 'dc_motors' not in config:
                    config['dc_motors'] = {}
                if 'offsets' not in config:
                    config['offsets'] = {}
                return config
            except Exception as e:
                print(f"Error loading config: {e}")
        
        # Return default config if file doesn't exist or error occurs
        return {
            'servo_motors': {},
            'dc_motors': {},
            'offsets': {}
        }
    
    def save_config(self, config):
        """Save motor configuration to file"""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving config: {e}")
            return False
