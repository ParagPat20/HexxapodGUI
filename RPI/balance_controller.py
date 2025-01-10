import numpy as np
import time
import json
from realsense_handler import RealSenseHandler

class BalanceController:
    def __init__(self, serial_comm):
        self.serial_comm = serial_comm
        self.realsense = RealSenseHandler()
        self.calibrated = False
        self.motor_directions = {'LDC': 1, 'RDC': 1}  # 1 or -1 for direction
        
        # Load config from file or use defaults
        self.config = self.load_config()
        
        # Initialize state variables
        self.prev_error = 0
        self.integral = 0
        self.prev_time = None
        self.filtered_angle = 0
        self.prev_gyro = 0
        
    def load_config(self):
        """Load configuration from file"""
        try:
            with open('RPI/balance_config.json', 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            # Default configuration
            default_config = {
                'pid': {
                    'Kp': 15.0,  # Proportional gain
                    'Ki': 0.1,   # Integral gain
                    'Kd': 0.5,   # Derivative gain
                    'max_integral': 50.0  # Anti-windup limit
                },
                'balance': {
                    'max_speed': 255,
                    'min_speed': -255,
                    'calibration_speed': 50,
                    'calibration_time': 0.5,
                    'target_angle': 0.0,  # Desired angle in degrees
                    'dead_zone': 1.0      # Minimum angle error to respond to
                },
                'imu': {
                    'complementary_alpha': 0.96,  # Complementary filter coefficient
                    'gyro_scale': 0.01,          # Gyro scaling factor
                    'pitch_offset': 0.0          # Calibrated zero angle
                }
            }
            self.save_config(default_config)
            return default_config
            
    def save_config(self, config=None):
        """Save current configuration to file"""
        if config is None:
            config = self.config
        with open('RPI/balance_config.json', 'w') as f:
            json.dump(config, f, indent=4)
            
    def update_config(self, new_config):
        """Update configuration with new values"""
        def update_dict(d, u):
            for k, v in u.items():
                if isinstance(v, dict):
                    d[k] = update_dict(d.get(k, {}), v)
                else:
                    d[k] = v
            return d
        
        self.config = update_dict(self.config, new_config)
        self.save_config()
        return {'status': 'success', 'message': 'Configuration updated'}
        
    def start(self):
        """Start the RealSense camera and IMU"""
        self.realsense.start()
        time.sleep(2)  # Wait for sensors to stabilize
        
    def calibrate_motors(self):
        """Calibrate motor directions using IMU feedback"""
        print("Starting motor calibration...")
        
        cal_speed = self.config['balance']['calibration_speed']
        cal_time = self.config['balance']['calibration_time']
        
        # Test each motor individually
        for motor in ['LDC', 'RDC']:
            print(f"Calibrating {motor}...")
            
            # Get initial orientation
            initial_data = self.realsense.get_imu_data()
            initial_gyro = initial_data['gyro']
            
            # Apply small forward motion
            self.serial_comm.send_command(motor, cal_speed)
            time.sleep(cal_time)
            
            # Get new orientation
            current_data = self.realsense.get_imu_data()
            current_gyro = current_data['gyro']
            
            # Stop motor
            self.serial_comm.send_command(motor, 0)
            time.sleep(cal_time)
            
            # Calculate direction based on gyro change
            # We're mainly interested in rotation around Y-axis (vertical)
            gyro_change = current_gyro[1] - initial_gyro[1]
            
            # Set direction multiplier based on expected vs actual rotation
            self.motor_directions[motor] = 1 if gyro_change > 0 else -1
            
        self.calibrated = True
        print("Motor calibration completed!")
        print(f"Motor directions: {self.motor_directions}")
        
    def get_tilt_angle(self, accel, gyro):
        """Calculate tilt angle using complementary filter"""
        # Get config values
        alpha = self.config['imu']['complementary_alpha']
        gyro_scale = self.config['imu']['gyro_scale']
        
        # Calculate accelerometer angle (pitch)
        accel_angle = np.degrees(np.arctan2(accel[1], accel[2]))
        
        # Get current time
        current_time = time.time()
        if self.prev_time is None:
            self.prev_time = current_time
            self.filtered_angle = accel_angle
            self.prev_gyro = gyro[0]
            return accel_angle
            
        # Calculate dt
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # Apply complementary filter
        gyro_angle = self.filtered_angle + (gyro[0] + self.prev_gyro) * 0.5 * dt * gyro_scale
        self.prev_gyro = gyro[0]
        
        # Combine accelerometer and gyro angles
        self.filtered_angle = alpha * gyro_angle + (1 - alpha) * accel_angle
        
        return self.filtered_angle
        
    def balance_update(self):
        """Calculate and apply balance corrections"""
        if not self.calibrated:
            print("Please calibrate motors first!")
            return
            
        # Get current IMU data
        imu_data = self.realsense.get_imu_data()
        accel = imu_data['accel']
        gyro = imu_data['gyro']
        
        # Calculate current angle
        current_angle = self.get_tilt_angle(accel, gyro)
        
        # Apply pitch offset
        current_angle -= self.config['imu']['pitch_offset']
        
        # Calculate error
        target_angle = self.config['balance']['target_angle']
        error = current_angle - target_angle
        
        # Check if error is within dead zone
        if abs(error) < self.config['balance']['dead_zone']:
            self.serial_comm.send_command('LDC', 0)
            self.serial_comm.send_command('RDC', 0)
            return
        
        # Get PID constants
        Kp = self.config['pid']['Kp']
        Ki = self.config['pid']['Ki']
        Kd = self.config['pid']['Kd']
        max_integral = self.config['pid']['max_integral']
        
        # Update integral term with anti-windup
        self.integral = np.clip(self.integral + error, -max_integral, max_integral)
        
        # Calculate derivative
        derivative = error - self.prev_error
        self.prev_error = error
        
        # Calculate PID output
        output = Kp * error + Ki * self.integral + Kd * derivative
        
        # Limit speed
        max_speed = self.config['balance']['max_speed']
        min_speed = self.config['balance']['min_speed']
        base_speed = np.clip(output, min_speed, max_speed)
        
        # Apply motor directions and send commands
        left_speed = int(base_speed * self.motor_directions['LDC'])
        right_speed = int(base_speed * self.motor_directions['RDC'])
        
        self.serial_comm.send_command('LDC', left_speed)
        self.serial_comm.send_command('RDC', right_speed)
        
        # Return debug info
        return {
            'angle': current_angle,
            'error': error,
            'output': output,
            'left_speed': left_speed,
            'right_speed': right_speed
        }
        
    def stop(self):
        """Stop all motors and sensors"""
        self.serial_comm.send_command('LDC', 0)
        self.serial_comm.send_command('RDC', 0)
        self.realsense.stop() 