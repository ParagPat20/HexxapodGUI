import zmq
import json
from HexapodController import HexapodController
from serial_communication import SerialCommunicator

def main():
    # Initialize ZMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    
    # Initialize hexapod controller
    hexapod = HexapodController()
    
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
                
                if command_type == 'update_lengths':
                    if 'leg_group' in data:
                        # Update specific leg
                        hexapod.set_leg_lengths(data['leg_group'], data['lengths'])
                    else:
                        # Update all legs
                        for leg_group, lengths in data.items():
                            hexapod.set_leg_lengths(leg_group, lengths)
                
                elif command_type == 'update_targets':
                    if 'leg_group' in data:
                        # Move specific leg
                        target = [data['target']['x'], data['target']['y'], data['target']['z']]
                        angles = hexapod.move_leg(data['leg_group'], target)
                        if angles is not None:
                            # Send angles to motors
                            motors = hexapod.motor_groups[data['leg_group']]
                            for motor_id, angle in zip(motors.keys(), angles):
                                if not serial_comm.send_command(motor_id, int(angle)):
                                    response = {
                                        "status": "error",
                                        "message": f"Failed to update motor {motor_id}"
                                    }
                                    break
                    else:
                        # Move all legs
                        targets = {leg: [data[leg]['x'], data[leg]['y'], data[leg]['z']] 
                                 for leg in data}
                        angles_dict = hexapod.move_all_legs(targets)
                        if angles_dict:
                            for leg_group, angles in angles_dict.items():
                                motors = hexapod.motor_groups[leg_group]
                                for motor_id, angle in zip(motors.keys(), angles):
                                    if not serial_comm.send_command(motor_id, int(angle)):
                                        response = {
                                            "status": "error",
                                            "message": f"Failed to update motor {motor_id}"
                                        }
                                        break
                
                elif command_type == 'update_offsets':
                    if 'leg_group' in data:
                        # Update specific leg offsets
                        hexapod.add_angle_offsets(data['leg_group'], data['offsets'])
                    else:
                        # Update all leg offsets
                        for leg_group, offsets in data.items():
                            hexapod.add_angle_offsets(leg_group, offsets)
                
                elif command_type == 'update_motor':
                    motor_id = data.get('motor_id')
                    value = data.get('value')
                    if motor_id and value is not None:
                        # Send command to the motor using SerialCommunicator
                        if serial_comm.send_command(motor_id, int(value)):
                            print(f"Motor {motor_id} updated to position {value}")
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