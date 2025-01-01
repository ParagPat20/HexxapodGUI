import zmq
import json
import threading
import queue
import time
from HexapodController import HexapodController
from serial_communication import SerialCommunicator
import platform

class HexapodServer:
    def __init__(self, zmq_port=5555):
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{zmq_port}")
        
        # Initialize components
        self.hexapod = HexapodController()
        
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
    
    def _send_motor_commands(self, leg_group, angles):
        """Send commands to motors and wait for responses"""
        if angles is not None:
            motors = self.hexapod.motor_groups[leg_group]
            for motor_id, angle in zip(motors.keys(), angles):
                self.command_queue.put({
                    'motor_id': motor_id,
                    'value': int(angle)
                })
    
    def handle_command(self, message):
        """Handle incoming ZMQ commands"""
        try:
            command_type = message.get('type')
            data = message.get('data')
            print(f"Received command: {command_type}")
            
            if command_type == 'update_lengths':
                if 'leg_group' in data:
                    self.hexapod.set_leg_lengths(data['leg_group'], data['lengths'])
                else:
                    for leg_group, lengths in data.items():
                        self.hexapod.set_leg_lengths(leg_group, lengths)
                return {"status": "success", "message": "Leg lengths updated"}
            
            elif command_type == 'update_targets':
                if 'leg_group' in data:
                    target = [data['target']['x'], data['target']['y'], data['target']['z']]
                    angles = self.hexapod.move_leg(data['leg_group'], target)
                    if angles is not None:
                        # Send angles to motors
                        motors = self.hexapod.motor_groups[data['leg_group']]
                        for motor_id, angle in zip(motors.keys(), angles):
                            if not self.serial_comm.send_command(motor_id, int(angle)):
                                return {
                                    "status": "error",
                                    "message": f"Failed to update motor {motor_id}"
                                }
                else:
                    targets = {leg: [data[leg]['x'], data[leg]['y'], data[leg]['z']] 
                             for leg in data}
                    angles_dict = self.hexapod.move_all_legs(targets)
                    if angles_dict:
                        for leg_group, angles in angles_dict.items():
                            motors = self.hexapod.motor_groups[leg_group]
                            for motor_id, angle in zip(motors.keys(), angles):
                                if not self.serial_comm.send_command(motor_id, int(angle)):
                                    return {
                                        "status": "error",
                                        "message": f"Failed to update motor {motor_id}"
                                    }
                return {"status": "success", "message": "Targets updated and movements executed"}
            
            elif command_type == 'update_offsets':
                if 'leg_group' in data:
                    self.hexapod.add_angle_offsets(data['leg_group'], data['offsets'])
                else:
                    for leg_group, offsets in data.items():
                        self.hexapod.add_angle_offsets(leg_group, offsets)
                return {"status": "success", "message": "Servo offsets updated"}
            
            elif command_type == 'update_motor':
                motor_id = data.get('motor_id')
                value = data.get('value')
                if motor_id and value is not None:
                    # Ensure value is within 0-180 range
                    value = max(0, min(180, float(value)))
                    # Send command to the motor using SerialCommunicator
                    if self.serial_comm.send_command(motor_id, int(value)):
                        print(f"Motor {motor_id} updated to position {value}")
                        return {"status": "success", "message": f"Motor {motor_id} updated to position {value}"}
                    else:
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