## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time

try:
    # Configure and start the pipeline
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("No RealSense devices found!")
        exit(0)
        
    print(f"Found {len(devices)} device(s):")
    for dev in devices:
        print(f"    {dev.get_info(rs.camera_info.name)}")
        
    # Get the first device
    device = devices[0]
    print(f"\nUsing device: {device.get_info(rs.camera_info.name)}")
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Get device product line for setting a supporting resolution
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    
    print(f"Product line: {device_product_line}")
    print("Available sensors:")
    for sensor in device.query_sensors():
        print(f"    {sensor.get_info(rs.camera_info.name)}")
        if sensor.is_motion_sensor():
            print("        - This is a motion sensor")
            for profile in sensor.get_stream_profiles():
                print(f"        - {profile.stream_name()}: {profile.fps()} fps")
    
    print("\nEnabling streams...")
    
    # Enable streams one by one with error checking
    try:
        config.enable_stream(rs.stream.accel)
        print("Enabled accelerometer stream")
    except Exception as e:
        print(f"Failed to enable accelerometer: {e}")
        
    try:
        config.enable_stream(rs.stream.gyro)
        print("Enabled gyro stream")
    except Exception as e:
        print(f"Failed to enable gyro: {e}")
    
    try:
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        print("Enabled depth stream")
    except Exception as e:
        print(f"Failed to enable depth: {e}")
        
    try:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        print("Enabled color stream")
    except Exception as e:
        print(f"Failed to enable color: {e}")
    
    print("\nStarting pipeline...")
    
    # Create a config object
    cfg = pipeline.start(config)
    
    # Getting the depth sensor's depth scale
    depth_sensor = cfg.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth Scale is: {depth_scale}")
    
    # Create opencv window
    cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
    
    print("\nPipeline started successfully")
    print("Press 'q' to exit")
    
    try:
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            
            # Process IMU data
            try:
                accel = frames.first_or_default(rs.stream.accel)
                if accel:
                    accel_data = accel.as_motion_frame().get_motion_data()
                    print(f"\rAccel: X:{accel_data.x:>6.2f} Y:{accel_data.y:>6.2f} Z:{accel_data.z:>6.2f}", end='')
                    
                gyro = frames.first_or_default(rs.stream.gyro)
                if gyro:
                    gyro_data = gyro.as_motion_frame().get_motion_data()
                    print(f" | Gyro: X:{gyro_data.x:>6.2f} Y:{gyro_data.y:>6.2f} Z:{gyro_data.z:>6.2f}", end='')
            except Exception as e:
                print(f"\rIMU Error: {e}", end='')
            
            # Process depth and color frames
            try:
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if depth_frame and color_frame:
                    # Convert images to numpy arrays
                    depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())
                    
                    # Apply colormap on depth image
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    
                    # Stack both images horizontally
                    images = np.hstack((color_image, depth_colormap))
                    
                    # Show images
                    cv2.imshow('RealSense', images)
            except Exception as e:
                print(f"\rFrame Error: {e}", end='')
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        print("\nStopping pipeline...")
        pipeline.stop()
        cv2.destroyAllWindows()

except Exception as e:
    print(f"\nError occurred: {str(e)}")
    print(f"Error type: {type(e).__name__}")
    import traceback
    traceback.print_exc()
    if 'pipeline' in locals():
        try:
            pipeline.stop()
        except:
            pass