import zmq
import json
from HexapodController import HexapodController

def main():
    # Initialize ZMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    
    # Initialize hexapod controller
    hexapod = HexapodController()
    
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
                        hexapod.move_leg(data['leg_group'], target)
                    else:
                        # Move all legs
                        targets = {leg: [data[leg]['x'], data[leg]['y'], data[leg]['z']] 
                                 for leg in data}
                        hexapod.move_all_legs(targets)
                
                elif command_type == 'update_offsets':
                    if 'leg_group' in data:
                        # Update specific leg offsets
                        hexapod.add_angle_offsets(data['leg_group'], data['offsets'])
                    else:
                        # Update all leg offsets
                        for leg_group, offsets in data.items():
                            hexapod.add_angle_offsets(leg_group, offsets)
                
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
        socket.close()
        context.term()

if __name__ == "__main__":
    main() 