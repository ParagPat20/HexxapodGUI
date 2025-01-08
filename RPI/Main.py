import zmq
import json
import platform
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

        # PID and Balance Control Variables
        self.balance_variables = {
            'Kp': 30.0,
            'Ki': 1.5,
            'Kd': 1.2,
            'target_angle': 0.0,
            'deadband': 5.0,
            'motor_scale': 2.5,
            'comp_filter_alpha': 0.93,
            'is_balancing': False
        }
        
        print(f"Hexapod server started on port {zmq_port}")
    
    def handle_command(self, message):
        """Handle incoming ZMQ commands"""
        try:
            command_type = message.get('type')
            data = message.get('data')
            print(f"Received command: {command_type}")
            
            if command_type == 'update_motor':
                return self.handle_motor_update(data)
            elif command_type == 'update_balance_params':
                return self.handle_balance_params_update(data)
            elif command_type == 'get_balance_params':
                return {
                    'status': 'success',
                    'data': self.balance_variables
                }
            else:
                return {
                    'status': 'error',
                    'message': f'Unknown command type: {command_type}'
                }
            
        except Exception as e:
            return {
                'status': 'error',
                'message': f'Error executing command: {str(e)}'
            }

    def handle_motor_update(self, data):
        """Handle motor update commands"""
        motor_id = data.get('motor_id')
        value = data.get('value')
        
        if motor_id and value is not None:
            # Handle DC motor commands (LDC and RDC)
            if motor_id in ['LDC', 'RDC']:
                value = max(-255, min(255, int(value)))
                result = self.serial_comm.send_command(motor_id, value)
                if result['status'] == 'success':
                    print(f"DC Motor {motor_id} updated to speed {value}")
                    return {'status': 'success', 'message': f'DC Motor {motor_id} updated to speed {value}'}
            else:
                # Handle servo motor commands
                value = max(0, min(180, float(value)))
                result = self.serial_comm.send_command(motor_id, int(value))
                if result['status'] == 'success':
                    print(f"Servo Motor {motor_id} updated to position {value}")
                    return {'status': 'success', 'message': f'Servo Motor {motor_id} updated to position {value}'}
            
            return result
        else:
            return {
                'status': 'error',
                'message': 'Invalid motor_id or value'
            }

    def handle_balance_params_update(self, data):
        """Handle balance parameters update"""
        try:
            # Update balance variables
            for key, value in data.items():
                if key in self.balance_variables:
                    if key == 'is_balancing':
                        # Send balance command to ESP32
                        cmd = "BALANCE:ON" if value else "BALANCE:OFF"
                        self.serial_comm.send_command('CMD', cmd)
                    else:
                        # Send parameter update to ESP32
                        self.serial_comm.send_command('PARAM', f"{key}:{value}")
                    self.balance_variables[key] = value
            
            return {
                'status': 'success',
                'message': 'Balance parameters updated',
                'data': self.balance_variables
            }
        except Exception as e:
            return {
                'status': 'error',
                'message': f'Error updating balance parameters: {str(e)}'
            }
    
    def run(self):
        """Main server loop"""
        try:
            while self.running:
                # Wait for next request from client
                message = self.socket.recv_json()
                print(f"Received command: {message['type']}")
                
                # Process the command and send response immediately
                response = self.handle_command(message)
                self.socket.send_json(response)
        
        except KeyboardInterrupt:
            print("\nShutting down server...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        self.socket.close()
        self.context.term()
        self.serial_comm.close_serial()

def main():
    server = HexapodServer()
    server.run()

if __name__ == "__main__":
    main() 