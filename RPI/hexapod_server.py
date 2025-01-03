import zmq
import json
from serial_communication import SerialCommunicator

def main():
    # Initialize ZMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    
    # Initialize serial communicator
    serial_comm = SerialCommunicator(port='/dev/ttyUSB0')
    
    print("Hexapod server started, waiting for commands...")
    
    try:
        while True:
            # Wait for next request from client
            message = socket.recv_json()
            response = {"status": "success", "message": "Command executed successfully"}
            
            try:
                command_type = message.get('type')
                data = message.get('data')
                
                if command_type == 'update_motor':
                    motor_id = data.get('motor_id')
                    value = data.get('value')
                    if motor_id and value is not None:
                        # Handle DC motor commands (LDC and RDC)
                        if motor_id in ['LDC', 'RDC']:
                            # Ensure value is within -255 to 255 range
                            value = max(-255, min(255, int(value)))
                            if serial_comm.send_command(motor_id, value):
                                print(f"DC Motor {motor_id} updated to speed {value}")
                            else:
                                response = {
                                    "status": "error",
                                    "message": f"Failed to update motor {motor_id}"
                                }
                        else:
                            # Handle servo motor commands
                            value = max(0, min(180, float(value)))
                            if serial_comm.send_command(motor_id, int(value)):
                                print(f"Servo Motor {motor_id} updated to position {value}")
                            else:
                                response = {
                                    "status": "error",
                                    "message": f"Failed to update motor {motor_id}"
                                }
                    else:
                        response = {
                            "status": "error",
                            "message": "Invalid motor_id or value"
                        }
                
                else:
                    response = {
                        "status": "error",
                        "message": f"Unknown command type: {command_type}"
                    }
                
            except Exception as e:
                response = {
                    "status": "error",
                    "message": f"Error executing command: {str(e)}"
                }
            
            # Send reply back to client
            socket.send_json(response)
    
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        serial_comm.close_serial()
        socket.close()
        context.term()

if __name__ == "__main__":
    main() 