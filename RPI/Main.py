import zmq
import json
import threading
import queue
import time
from serial_communication import SerialCommunicator
import platform

class HexapodServer:
    def __init__(self, zmq_port=5555):
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{zmq_port}")
        
        # Motor grouping
        self.motor_groups = {
            'left_front': {
                "L1": "Front Leg",
                "L2": "Front Lower",
                "L3": "Front Middle"
            },
            'left_center': {
                "L5": "Center Upper",
                "L6": "Center Lower 2",
                "L7": "Center Lower",
                "L8": "Center Leg"
            },
            'left_back': {
                "L9": "Back Mid",
                "L10": "Back Lower",
                "L12": "Back Leg"
            },
            'right_front': {
                "R14": "Front Lower",
                "R15": "Front Mid",
                "R16": "Front Leg"
            },
            'right_center': {
                "R6": "Center Upper",
                "R8": "Center Leg",
                "R10": "Center Lower 2",
                "R12": "Center Lower"
            },
            'right_back': {
                "R1": "Back Lower",
                "R2": "Back Mid",
                "R3": "Back Leg"
            }
        }
        
        # Determine the correct serial port based on the platform
        if platform.system() == 'Windows':
            serial_port = 'COM7'  # Update this to match your Windows COM port
        else:
            serial_port = '/dev/ttyUSB0'  # Default for Raspberry Pi
            
        self.serial_comm = SerialCommunicator(port=serial_port)
        
        # Message queues for communication
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        
        # Control flags
        self.running = True
        
        # Start communication threads
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.response_thread = threading.Thread(target=self._response_worker, daemon=True)
        self.serial_thread.start()
        self.response_thread.start()
        
        print(f"Hexapod server started on port {zmq_port}")
    
    def _serial_worker(self):
        """Worker thread to handle serial command sending"""
        while self.running:
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get()
                    if self.serial_comm.send_command(command['motor_id'], command['value']):
                        # Wait a bit for the command to be processed
                        time.sleep(0.01)
                else:
                    time.sleep(0.01)  # Prevent busy waiting
            except Exception as e:
                print(f"Serial communication error: {e}")
    
    def _response_worker(self):
        """Worker thread to handle serial response reading"""
        while self.running:
            try:
                response = self.serial_comm.read_response()
                if response:
                    self.response_queue.put(response)
                time.sleep(0.01)  # Prevent busy waiting
            except Exception as e:
                print(f"Response reading error: {e}")
    
    def handle_command(self, message):
        """Handle incoming ZMQ commands"""
        try:
            command_type = message.get('type')
            data = message.get('data')
            print(f"Received command: {command_type}")
            
            if command_type == 'update_motor':
                motor_id = data.get('motor_id')
                value = data.get('value')
                if motor_id and value is not None:
                    # Handle DC motor commands (LDC and RDC)
                    if motor_id in ['LDC', 'RDC']:
                        # Ensure value is within -255 to 255 range
                        value = max(-255, min(255, int(value)))
                        if self.serial_comm.send_command(motor_id, value):
                            print(f"DC Motor {motor_id} updated to speed {value}")
                            return {"status": "success", "message": f"DC Motor {motor_id} updated to speed {value}"}
                    else:
                        # Handle servo motor commands
                        value = max(0, min(180, float(value)))
                        if self.serial_comm.send_command(motor_id, int(value)):
                            print(f"Servo Motor {motor_id} updated to position {value}")
                            return {"status": "success", "message": f"Servo Motor {motor_id} updated to position {value}"}
                    
                    return {
                        "status": "error",
                        "message": f"Failed to update motor {motor_id}"
                    }
                else:
                    return {
                        "status": "error",
                        "message": "Invalid motor_id or value"
                    }
            
            else:
                return {
                    "status": "error",
                    "message": f"Unknown command type: {command_type}"
                }
            
        except Exception as e:
            return {
                "status": "error",
                "message": f"Error executing command: {str(e)}"
            }
    
    def run(self):
        """Main server loop"""
        try:
            while self.running:
                # Wait for next request from client
                message = self.socket.recv_json()
                print(f"Received command: {message['type']}")
                
                # Process the command
                response = self.handle_command(message)
                
                # Add any pending responses from serial communication
                while not self.response_queue.empty():
                    serial_response = self.response_queue.get()
                    if 'message' in response:
                        response['message'] += f"\nSerial: {serial_response}"
                    else:
                        response['message'] = f"Serial: {serial_response}"
                
                # Send reply back to client
                self.socket.send_json(response)
        
        except KeyboardInterrupt:
            print("\nShutting down server...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        time.sleep(0.1)  # Give threads time to finish
        self.socket.close()
        self.context.term()
        self.serial_comm.close_serial()

def main():
    server = HexapodServer()
    server.run()

if __name__ == "__main__":
    main() 