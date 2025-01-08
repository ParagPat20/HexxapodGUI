import serial
import time

class SerialCommunicator:
    def __init__(self, port='/dev/ttyS0', baud_rate=115200):
        """Initialize serial communication."""
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.connect()
    
    def connect(self):
        """Establish serial connection."""
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"Connected to {self.port} at {self.baud_rate} baud rate.")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to serial port: {e}")
            return False
    
    def reconnect(self, max_attempts=5):
        """Attempt to reconnect to the serial port."""
        attempt = 0
        while attempt < max_attempts:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                
                if self.connect():
                    print("Successfully reconnected to serial port.")
                    return True
                    
            except serial.SerialException as e:
                attempt += 1
                print(f"Reconnection attempt {attempt}/{max_attempts} failed: {e}")
                
                if attempt < max_attempts:
                    print(f"Retrying in 2 seconds...")
                    time.sleep(2)
                else:
                    print("Maximum reconnection attempts reached.")
                    return False
    
    def send_command(self, motor_id, value):
        """Send a command to the ESP32.
        
        Args:
            motor_id (str): Motor ID (e.g., 'L1', 'R1', 'LDC', 'RDC', 'CMD', 'PARAM')
            value: Command value:
                - For servos: 0-180 degrees
                - For DC motors: -255 to 255 speed
                - For CMD: Command string (e.g., "BALANCE:ON")
                - For PARAM: Parameter string (e.g., "Kp:30.0")
        """
        try:
            if not self.ser or not self.ser.is_open:
                if not self.reconnect():
                    return {"status": "error", "message": "Failed to connect"}
            
            # Format command based on type
            if motor_id == 'CMD':
                command = f"{value}\n"  # Direct command
            elif motor_id == 'PARAM':
                command = f"PARAM:{value}\n"  # Parameter update
            elif motor_id in ['LDC', 'RDC']:
                command = f"{motor_id}:{value}\n"  # DC motor command
            else:
                value = max(0, min(180, int(value)))  # Servo command
                command = f"{motor_id}:{value}\n"
            
            # Send command
            self.ser.write(command.encode('ascii'))
            
            # Wait for and return response for balance commands
            if motor_id in ['CMD', 'PARAM']:
                time.sleep(0.1)  # Give ESP32 time to respond
                response = self.read_response()
                if response:
                    return {"status": "success", "message": response}
            
            return {"status": "success", "message": "OK"}
            
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def read_response(self):
        """Read response from ESP32."""
        try:
            if not self.ser or not self.ser.is_open:
                if not self.reconnect():
                    return None
            
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('ascii').strip()
                print(f"Received: {response}")
                return response
                
        except Exception as e:
            print(f"Error reading response: {e}")
        return None
    
    def close_serial(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")

