import zmq
import json
import platform
import threading
import time
from serial_communication import SerialCommunicator
from balance_controller import BalanceController

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
        
        # Initialize balance controller
        self.balance_controller = BalanceController(self.serial_comm)
        self.auto_balance_enabled = False
        self.balance_thread = None
        
        # Add debug flag
        self.debug = True  # Set to True to see more detailed information
        
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
            data = message.get('data', {})
            
            if command_type == 'update_motor':
                return self.handle_motor_update(data)
            elif command_type == 'calibrate_motors':
                return self.handle_motor_calibration()
            elif command_type == 'toggle_auto_balance':
                return self.handle_auto_balance_toggle(data)
            elif command_type == 'update_config':
                return self.handle_config_update(data)
            elif command_type == 'get_balance_info':
                return self.handle_get_balance_info()
            elif command_type == 'start_imu':
                return self.handle_start_imu()
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

    def handle_start_imu(self):
        """Handle IMU start command"""
        try:
            self.balance_controller.start()
            return {
                'status': 'success',
                'message': 'IMU started successfully'
            }
        except Exception as e:
            return {
                'status': 'error',
                'message': f'Failed to start IMU: {str(e)}'
            }

    def handle_motor_calibration(self):
        """Handle motor calibration command"""
        try:
            if not hasattr(self.balance_controller, 'realsense') or not self.balance_controller.realsense.running:
                self.balance_controller.start()
                time.sleep(2)  # Wait for IMU to stabilize
                
            self.balance_controller.calibrate_motors()
            return {
                'status': 'success',
                'message': 'Motor calibration completed'
            }
        except Exception as e:
            return {
                'status': 'error',
                'message': f'Calibration failed: {str(e)}'
            }

    def handle_auto_balance_toggle(self, data):
        """Handle auto balance enable/disable"""
        enabled = data.get('enabled', False)
        
        if enabled and not self.auto_balance_enabled:
            # Make sure IMU is started and motors are calibrated
            if not hasattr(self.balance_controller, 'realsense') or not self.balance_controller.realsense.running:
                self.balance_controller.start()
                time.sleep(2)
                
            if not self.balance_controller.calibrated:
                self.balance_controller.calibrate_motors()
                
            self.auto_balance_enabled = True
            self.balance_thread = threading.Thread(target=self._auto_balance_loop)
            self.balance_thread.daemon = True
            self.balance_thread.start()
            return {'status': 'success', 'message': 'Auto-balance enabled'}
            
        elif not enabled and self.auto_balance_enabled:
            self.auto_balance_enabled = False
            if self.balance_thread:
                self.balance_thread.join(timeout=1.0)
            return {'status': 'success', 'message': 'Auto-balance disabled'}
        
        return {'status': 'success', 'message': 'Auto-balance state unchanged'}

    def handle_get_balance_info(self):
        """Get current balance status and debug info"""
        if not self.auto_balance_enabled:
            return {
                'status': 'success',
                'message': 'Balance control is disabled',
                'data': {
                    'enabled': False,
                    'calibrated': self.balance_controller.calibrated
                }
            }
            
        # Get the latest debug info
        debug_info = getattr(self, 'last_debug_info', {})
        return {
            'status': 'success',
            'message': 'Balance control is active',
            'data': {
                'enabled': True,
                'calibrated': self.balance_controller.calibrated,
                'angle': debug_info.get('angle', 0),
                'error': debug_info.get('error', 0),
                'left_speed': debug_info.get('left_speed', 0),
                'right_speed': debug_info.get('right_speed', 0)
            }
        }

    def _auto_balance_loop(self):
        """Background thread for auto-balance updates"""
        print("Starting balance control loop...")
        update_count = 0
        start_time = time.time()
        
        while self.auto_balance_enabled and self.running:
            try:
                # Update balance control
                debug_info = self.balance_controller.balance_update()
                
                if debug_info:  # Store latest debug info
                    self.last_debug_info = debug_info
                    
                    # Print debug info every 10 updates (10Hz)
                    update_count += 1
                    if update_count % 10 == 0 and self.debug:
                        print(f"\rAngle: {debug_info['angle']:6.2f}° | "
                              f"Error: {debug_info['error']:6.2f}° | "
                              f"Motors: L={debug_info['left_speed']:4d} R={debug_info['right_speed']:4d}", 
                              end='')
                
                # Calculate and maintain update rate
                update_count += 1
                if update_count % 100 == 0:  # Log performance every 100 updates
                    elapsed = time.time() - start_time
                    rate = 100 / elapsed if elapsed > 0 else 0
                    if self.debug:
                        print(f"\nBalance update rate: {rate:.1f} Hz")
                    start_time = time.time()
                
                time.sleep(0.01)  # 100Hz update rate
                
            except Exception as e:
                print(f"Error in balance loop: {str(e)}")
                self.auto_balance_enabled = False
                break

    def handle_motor_update(self, data):
        """Handle motor update commands"""
        if self.auto_balance_enabled:
            return {
                'status': 'error',
                'message': 'Cannot update motors while auto-balance is enabled'
            }
            
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
    
    def handle_config_update(self, data):
        """Handle configuration update command"""
        try:
            result = self.balance_controller.update_config(data)
            return result
        except Exception as e:
            return {
                'status': 'error',
                'message': f'Failed to update configuration: {str(e)}'
            }
    
    def run(self):
        """Main server loop"""
        try:
            while self.running:
                # Wait for next request from client
                message = self.socket.recv_json()
                self.log(f"Received command: {message['type']}")
                
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
        self.auto_balance_enabled = False
        if self.balance_thread and self.balance_thread.is_alive():
            self.balance_thread.join(timeout=1.0)
        self.balance_controller.stop()
        self.socket.close()
        self.context.term()
        self.serial_comm.close_serial()

def main():
    server = HexapodServer()
    server.run()

if __name__ == "__main__":
    main() 