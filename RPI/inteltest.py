import subprocess

try:
    # Attempt to install pyrealsense2 using pip
    subprocess.check_call(["pip", "install", "pyrealsense2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("Successfully installed pyrealsense2 using pip.")
except subprocess.CalledProcessError as e:
    try:
        # Attempt to install pyrealsense2 using apt-get
        subprocess.check_call(["apt-get", "install", "librealsense2-dev"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("Successfully installed pyrealsense2 using apt-get.")
    except subprocess.CalledProcessError as e:
        try:
            # Attempt to install pyrealsense2 using pip3
            subprocess.check_call(["pip3", "install", "pyrealsense2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("Successfully installed pyrealsense2 using pip3.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to install pyrealsense2: {e}")