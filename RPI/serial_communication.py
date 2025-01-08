import serial
import time
import json

class SerialCommunicator:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reset
        print(f"Serial connection established on {port}")

    def send_command(self, command_type, value):
        """Send command to ESP32"""
        try:
            if command_type == 'CMD':
                # Direct command string
                command = f"{value}\n"
            elif command_type == 'PARAM':
                # Parameter update command
                command = f"{value}\n"
            else:
                # Motor control command
                command = f"{command_type}:{value}\n"
            
            self.serial.write(command.encode())
            time.sleep(0.01)  # Small delay to ensure command is processed
            
            # Read response if available
            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                return {'status': 'success', 'message': response}
            
            return {'status': 'success', 'message': 'Command sent'}
            
        except Exception as e:
            return {
                'status': 'error',
                'message': f'Serial communication error: {str(e)}'
            }

    def close_serial(self):
        """Close serial connection"""
        if self.serial.is_open:
            self.serial.close()
            print("Serial connection closed")

