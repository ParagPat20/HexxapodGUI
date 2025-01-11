import zmq
import json
import platform
import threading
import time
from serial_communication import SerialCommunicator

class HexapodServer:
    def __init__(self, zmq_port=5555):
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{zmq_port}")
        
        # Determine the correct serial port based on the platform
        if platform.system() == 'Windows':
            serial_port = 'COM7'  # Update this to match your Windows COM port
        else:
            serial_port = '/dev/ttyUSB0'  # Default for Raspberry Pi
            
        self.serial_comm = SerialCommunicator(port=serial_port)
        self.running = True
        
        # Add debug flag
        self.debug = True  # Enable debug logging
        
        print(f"Hexapod server started on port {zmq_port}")
    
    def log(self, message):
        """Centralized logging function"""
        if self.debug:
            print(f"[DEBUG] {message}")
        else:
            print(message)

    def handle_command(self, message):
        """Handle incoming ZMQ commands"""
        try:
            command_type = message.get('type')
            data = message.get('data')
            
            self.log(f"Received command: {command_type} with data: {data}")
            
            if command_type == 'update_motor':
                result = self.handle_motor_update(data)
                self.log(f"Motor update result: {result}")
                return result
            else:
                return {
                    'status': 'error',
                    'message': f'Unknown command type: {command_type}'
                }
            
        except Exception as e:
            self.log(f"Error handling command: {str(e)}")
            return {
                'status': 'error',
                'message': f'Error executing command: {str(e)}'
            }

    def handle_motor_update(self, data):
        """Handle motor update commands"""
        try:
            motor_id = data.get('motor_id')
            value = data.get('value')
            
            self.log(f"Processing motor update: {motor_id} = {value}")
            
            if motor_id and value is not None:
                # Handle DC motor commands (LDC and RDC)
                if motor_id in ['LDC', 'RDC']:
                    value = max(-255, min(255, int(value)))
                    result = self.serial_comm.send_command(motor_id, value)
                    if result['status'] == 'success':
                        self.log(f"DC Motor {motor_id} updated to speed {value}")
                        return {'status': 'success', 'message': f'DC Motor {motor_id} updated to speed {value}'}
                else:
                    # Handle servo motor commands
                    value = max(0, min(180, float(value)))
                    result = self.serial_comm.send_command(motor_id, int(value))
                    if result['status'] == 'success':
                        self.log(f"Servo Motor {motor_id} updated to position {value}")
                        return {'status': 'success', 'message': f'Servo Motor {motor_id} updated to position {value}'}
                
                return result
            else:
                self.log("Invalid motor_id or value")
                return {
                    'status': 'error',
                    'message': 'Invalid motor_id or value'
                }
        except Exception as e:
            self.log(f"Error in motor update: {str(e)}")
            return {
                'status': 'error',
                'message': f'Error in motor update: {str(e)}'
            }
    
    def run(self):
        """Main server loop"""
        try:
            self.log("Server starting main loop")
            while self.running:
                try:
                    # Wait for next request from client with timeout
                    message = self.socket.recv_json()
                    self.log(f"Received message: {message}")
                    
                    # Process the command and send response immediately
                    response = self.handle_command(message)
                    self.log(f"Sending response: {response}")
                    self.socket.send_json(response)
                    
                except zmq.error.Again:
                    continue  # Timeout, continue loop
                except Exception as e:
                    self.log(f"Error in main loop: {str(e)}")
                    # Try to send error response
                    try:
                        self.socket.send_json({
                            'status': 'error',
                            'message': f'Server error: {str(e)}'
                        })
                    except:
                        pass
        
        except KeyboardInterrupt:
            self.log("\nShutting down server...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.log("Cleaning up resources...")
        self.running = False
        self.socket.close()
        self.context.term()
        self.serial_comm.close_serial()

def main():
    server = HexapodServer()
    server.run()

if __name__ == "__main__":
    main() 