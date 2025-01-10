try:
    import pyrealsense2 as rs
except ImportError:
    import subprocess
    subprocess.check_call(["python", '-m', 'pip', 'install', 'pyrealsense2'])
    import pyrealsense2 as rs
import numpy as np
import time
import threading

class RealSenseHandler:
    def __init__(self):
        # Initialize RealSense context and get device
        self.ctx = rs.context()
        if len(self.ctx.query_devices()) == 0:
            raise RuntimeError("No RealSense devices found!")
            
        self.device = self.ctx.query_devices()[0]
        print(f"Using device: {self.device.get_info(rs.camera_info.name)}")
        
        # Check for IMU
        self.has_imu = False
        for sensor in self.device.query_sensors():
            if sensor.is_motion_sensor():
                self.has_imu = True
                print("Found Motion Sensor")
                break
                
        if not self.has_imu:
            raise RuntimeError("Device does not have IMU!")
            
        # Initialize pipeline and config
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Store latest IMU and depth data
        self.imu_data = {'accel': None, 'gyro': None}
        self.depth_data = None
        self.running = False
        
    def start(self):
        """Start the RealSense pipeline with all necessary streams"""
        print("Enabling streams...")
        
        # Enable IMU streams
        try:
            self.config.enable_stream(rs.stream.accel)
            print("Enabled accelerometer stream")
        except Exception as e:
            print(f"Failed to enable accelerometer: {e}")
            
        try:
            self.config.enable_stream(rs.stream.gyro)
            print("Enabled gyro stream")
        except Exception as e:
            print(f"Failed to enable gyro: {e}")
            
        # Enable depth stream
        try:
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            print("Enabled depth stream")
        except Exception as e:
            print(f"Failed to enable depth: {e}")
            
        print("\nStarting pipeline...")
        self.profile = self.pipeline.start(self.config)
        
        # Get depth scale
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print(f"Depth Scale is: {self.depth_scale}")
        
        self.running = True
        self.imu_thread = threading.Thread(target=self._imu_reader)
        self.imu_thread.daemon = True  # Thread will close when main program exits
        self.imu_thread.start()
        
        print("Pipeline started successfully")
        time.sleep(2)  # Wait for sensors to stabilize
        
    def _imu_reader(self):
        """Background thread to continuously read IMU data"""
        while self.running:
            try:
                # Wait for frames
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                
                # Process IMU data
                accel = frames.first_or_default(rs.stream.accel)
                if accel:
                    accel_data = accel.as_motion_frame().get_motion_data()
                    self.imu_data['accel'] = np.array([accel_data.x, accel_data.y, accel_data.z])
                    
                gyro = frames.first_or_default(rs.stream.gyro)
                if gyro:
                    gyro_data = gyro.as_motion_frame().get_motion_data()
                    self.imu_data['gyro'] = np.array([gyro_data.x, gyro_data.y, gyro_data.z])
                
                # Get depth frame
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    self.depth_data = np.asanyarray(depth_frame.get_data())
                    
            except Exception as e:
                print(f"Error in IMU reader: {str(e)}")
                time.sleep(0.1)  # Prevent tight loop on error
                
    def get_imu_data(self):
        """Get the latest IMU readings"""
        if not self.running:
            raise RuntimeError("Pipeline not started!")
        return self.imu_data
        
    def get_depth_data(self):
        """Get the latest depth frame"""
        if not self.running:
            raise RuntimeError("Pipeline not started!")
        return self.depth_data
        
    def get_depth_scale(self):
        """Get the depth scale"""
        if not self.running:
            raise RuntimeError("Pipeline not started!")
        return self.depth_scale
        
    def stop(self):
        """Stop the RealSense pipeline"""
        self.running = False
        if hasattr(self, 'imu_thread') and self.imu_thread.is_alive():
            self.imu_thread.join(timeout=1.0)  # Wait up to 1 second for thread to finish
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
            
    def __del__(self):
        """Destructor to ensure cleanup"""
        self.stop()
        
        