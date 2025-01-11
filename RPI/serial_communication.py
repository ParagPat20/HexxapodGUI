import serial
import time

class SerialCommunicator:
    def __init__(self, port, baudrate=115200, timeout=1):
        """Initialize serial communication"""
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Serial connection established on {port}")
            time.sleep(2)  # Wait for Arduino to reset
        except Exception as e:
            print(f"Error initializing serial connection: {e}")
            self.serial = None
    
    def send_command(self, motor_id, value):
        """Send command to Arduino"""
        if not self.serial:
            print("Serial connection not available")
            return {'status': 'error', 'message': 'Serial connection not available'}
        
        try:
            # Format command
            command = f"{motor_id}:{value}\n"
            print(f"Sending command: {command.strip()}")
            
            # Send command
            self.serial.write(command.encode())
            self.serial.flush()
            
            # Wait for and read response
            response = self.serial.readline().decode().strip()
            print(f"Received response: {response}")
            
            if response.startswith("OK"):
                return {'status': 'success', 'message': response}
            else:
                return {'status': 'error', 'message': response}
                
        except Exception as e:
            print(f"Error sending command: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def close_serial(self):
        """Close serial connection"""
        if self.serial:
            try:
                self.serial.close()
                print("Serial connection closed")
            except Exception as e:
                print(f"Error closing serial connection: {e}")

