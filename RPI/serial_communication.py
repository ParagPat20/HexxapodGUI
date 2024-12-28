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
        """Send a command to the ESP32."""
        try:
            if not self.ser or not self.ser.is_open:
                if not self.reconnect():
                    return False
            
            command = f"{motor_id}:{value}\n"
            self.ser.write(command.encode('ascii'))
            print(f"Sent command: {command.strip()}")
            return True
            
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
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

