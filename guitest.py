import tkinter as tk
from tkinter import ttk
import zmq
import threading
import queue
import time
from config_handler import ConfigHandler

class HexapodGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Hexapod Controller")
        
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://192.168.50.39:5555")  # Replace with your RPI's IP
        
        # Message queue for communication
        self.message_queue = queue.Queue()
        self.command_queue = queue.Queue()
        
        # Start communication threads
        self.comm_thread = threading.Thread(target=self.communication_handler, daemon=True)
        self.comm_thread.start()
        
        self.response_thread = threading.Thread(target=self.response_handler, daemon=True)
        self.response_thread.start()
        
        # Initialize config handler and load last values
        self.config_handler = ConfigHandler()
        self.motor_values = self.config_handler.load_config()
        
        # Create serial monitor
        self.create_serial_monitor()
        
        # Define standby angles for stable standing position
        self.standby_angles = {
            # Left Front Leg
            'L1': 90,   # Hip centered
            'L2': 135,  # Knee bent for stability
            'L3': 45,   # Ankle angled for ground contact
            
            # Left Center Leg
            'L5': 90,   # Hip centered
            'L6': 135,  # Knee bent
            'L7': 45,   # Ankle angled
            'L8': 90,   # Extra joint centered
            
            # Left Back Leg
            'L9': 90,   # Hip centered
            'L10': 135, # Knee bent
            'L12': 45,  # Ankle angled
            
            # Right Front Leg
            'R14': 135, # Knee bent
            'R15': 90,  # Hip centered
            'R16': 45,  # Ankle angled
            
            # Right Center Leg
            'R6': 90,   # Hip centered
            'R8': 45,   # Ankle angled
            'R10': 135, # Knee bent
            'R12': 90,  # Extra joint centered
            
            # Right Back Leg
            'R1': 135,  # Knee bent
            'R2': 90,   # Hip centered
            'R3': 45    # Ankle angled
        }
        
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
        
        # Setup UI
        self.setup_styles()
        self.create_gui()
        
        # Setup keyboard controls
        self.setup_keyboard_controls()
    
    def setup_styles(self):
        """Setup ttk styles"""
        style = ttk.Style()
        style.configure('Header.TLabel', font=('Helvetica', 12, 'bold'))
        style.configure('Monitor.TLabel', font=('Consolas', 10))
    
    def send_zmq_command(self, command_type, data):
        """Send command to RPI"""
        try:
            message = {
                'type': command_type,
                'data': data
            }
            self.socket.send_json(message)
            response = self.socket.recv_json()
            return response
        except Exception as e:
            print(f"Error sending command: {e}")
            return None
    
    def validate_input(self, P):
        """Validate numeric input"""
        if P == "" or P == "-":
            return True
        try:
            float(P)
            return True
        except ValueError:
            return False
    
    def create_gui(self):
        """Create the main GUI"""
        # Create notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=5, pady=5)
        
        # DC Motors Tab
        dc_frame = ttk.Frame(notebook)
        notebook.add(dc_frame, text="DC Motors")
        self.create_dc_motor_controls(dc_frame)
        
        # Servo Motors Tab
        servo_frame = ttk.Frame(notebook)
        notebook.add(servo_frame, text="Servo Motors")
        self.create_servo_motor_controls(servo_frame)
        
        # Movement Controls Tab
        movement_frame = ttk.Frame(notebook)
        notebook.add(movement_frame, text="Movement")
        self.create_movement_control_panel(movement_frame)
    
    def create_dc_motor_controls(self, parent):
        """Create DC motor control interface"""
        # Create DC motor control frame
        dc_frame = ttk.LabelFrame(parent, text="DC Motor Control")
        dc_frame.pack(fill='x', padx=10, pady=5)
        
        # Create control elements for each DC motor
        dc_motors = {
            'LDC': 'Left DC Motor',
            'RDC': 'Right DC Motor'
        }
        
        vcmd = (self.root.register(self.validate_input), '%P')
        
        for motor_id, motor_name in dc_motors.items():
            motor_frame = ttk.Frame(dc_frame)
            motor_frame.pack(fill='x', padx=5, pady=2)
            
            ttk.Label(motor_frame, text=motor_name).pack(side='left', padx=5)
            
            # Create slider
            slider = ttk.Scale(motor_frame, from_=-255, to=255, orient='horizontal')
            # Set initial value from config
            initial_value = self.motor_values['dc_motors'].get(motor_id, 0)
            slider.set(initial_value)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            # Create entry for direct value input
            value_var = tk.StringVar(value=str(initial_value))
            entry = ttk.Entry(motor_frame, textvariable=value_var, width=8, 
                            validate='key', validatecommand=vcmd)
            entry.pack(side='left', padx=5)
            
            # Connect slider and entry
            slider.configure(command=lambda v, var=value_var, e=entry, m=motor_id: 
                            self._on_dc_slider_change(v, var, e, m))
            slider.bind('<ButtonRelease-1>', lambda e, m=motor_id, s=slider: 
                        self.update_dc_motor(m, float(s.get())))
            entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry, m=motor_id: 
                        self._on_dc_entry_change(e, s, var, ent, m))

    def create_servo_motor_controls(self, parent):
        """Create servo motor control interface"""
        # Create scrollable frame
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Pack scrollbar and canvas
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        
        # Create a frame for each leg group
        vcmd = (self.root.register(self.validate_input), '%P')
        
        for leg_group, motors in self.motor_groups.items():
            group_frame = ttk.LabelFrame(scrollable_frame, text=leg_group.replace('_', ' ').title())
            group_frame.pack(fill='x', padx=5, pady=5)
            
            for motor_id, motor_name in motors.items():
                motor_frame = ttk.Frame(group_frame)
                motor_frame.pack(fill='x', padx=5, pady=2)
                
                ttk.Label(motor_frame, text=f"{motor_name} ({motor_id})").pack(side='left', padx=5)
                
                # Create slider
                slider = ttk.Scale(motor_frame, from_=0, to=180, orient='horizontal')
                # Set initial value from config
                initial_value = self.motor_values['servo_motors'].get(motor_id, 90)
                slider.set(initial_value)
                slider.pack(side='left', fill='x', expand=True, padx=5)
                
                # Create entry
                value_var = tk.StringVar(value=str(initial_value))
                entry = ttk.Entry(motor_frame, textvariable=value_var, width=8,
                                validate='key', validatecommand=vcmd)
                entry.pack(side='left', padx=5)
                
                # Add offset entry
                offset_var = tk.StringVar(value=str(self.motor_values['offsets'].get(motor_id, 0)))
                ttk.Label(motor_frame, text="Offset:").pack(side='left', padx=2)
                offset_entry = ttk.Entry(motor_frame, textvariable=offset_var, width=5,
                                       validate='key', validatecommand=vcmd)
                offset_entry.pack(side='left', padx=2)
                
                # Add inversion checkbox
                invert_var = tk.BooleanVar(value=self.motor_values['inverted_motors'].get(motor_id, False))
                ttk.Checkbutton(motor_frame, text="Inverted", variable=invert_var,
                              command=lambda m=motor_id, v=invert_var: self.update_motor_inversion(m, v)).pack(side='left', padx=5)
                
                # Add update button
                ttk.Button(motor_frame, text="Update",
                          command=lambda m=motor_id, s=slider, o=offset_var, inv=invert_var: 
                          self.update_servo_with_offset(m, s.get(), o.get(), inv.get())).pack(side='left', padx=5)
                
                # Connect slider and entry
                slider.configure(command=lambda v, var=value_var, e=entry, m=motor_id: 
                               self._on_servo_slider_change(v, var, e, m))
                slider.bind('<ButtonRelease-1>', lambda e, m=motor_id, s=slider, o=offset_var, inv=invert_var: 
                          self.update_servo_with_offset(m, float(s.get()), o.get(), inv.get()))
                entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry, m=motor_id, o=offset_var, inv=invert_var: 
                          self._on_servo_entry_change(e, s, var, ent, m))
                offset_entry.bind('<Return>', lambda e, m=motor_id, s=slider, o=offset_var:
                                self.update_offset(m, o.get()))

    def _on_servo_slider_change(self, value, value_var, entry, motor_id):
        """Handle slider value changes for servo control"""
        try:
            value = float(value)
            value_var.set(f"{value:.1f}")
            entry.delete(0, tk.END)
            entry.insert(0, f"{value:.1f}")
            
            # Send the raw value to update_servo
            self.update_servo(motor_id, value)
        except ValueError:
            pass
    
    def _on_servo_entry_change(self, event, slider, value_var, entry, motor_id):
        """Handle entry value changes for servo control"""
        try:
            value = float(entry.get())
            value = max(0, min(180, value))  # Clamp value
            slider.set(value)
            value_var.set(f"{value:.1f}")
            
            # Send the raw value to update_servo
            self.update_servo(motor_id, value)
        except ValueError:
            pass
    
    def update_servo(self, motor_id, value):
        """Send servo update command and save to config"""
        # Save the raw value to config
        self.motor_values['servo_motors'][motor_id] = value
        self.config_handler.save_config(self.motor_values)
        
        # Apply inversion if needed
        if self.motor_values['inverted_motors'].get(motor_id, False):
            value = 180 - value
            
        # Apply offset
        offset = self.motor_values['offsets'].get(motor_id, 0)
        adjusted_value = value + offset
        
        # Ensure the adjusted value is within valid range
        adjusted_value = max(0, min(180, adjusted_value))
        
        # Send the adjusted value to the motor
        self.command_queue.put(('update_motor', {
            'motor_id': motor_id,
            'value': int(adjusted_value)
        }))

    def _on_dc_slider_change(self, value, value_var, entry, motor_id):
        """Handle slider value changes for DC motor control"""
        try:
            value = float(value)
            value_var.set(f"{value:.1f}")
            entry.delete(0, tk.END)
            entry.insert(0, f"{value:.1f}")
        except ValueError:
            pass

    def _on_dc_entry_change(self, event, slider, value_var, entry, motor_id):
        """Handle entry value changes for DC motor control"""
        try:
            value = float(entry.get())
            value = max(-255, min(255, value))  # Clamp value
            slider.set(value)
            value_var.set(f"{value:.1f}")
            
            # Update the motor
            self.update_dc_motor(motor_id, value)
        except ValueError:
            pass

    def update_dc_motor(self, motor_id, value):
        """Send DC motor update command and save to config"""
        self.command_queue.put(('update_motor', {
            'motor_id': motor_id,
            'value': int(value)
        }))
        # Update config
        self.config_handler.update_motor_value(motor_id, int(value))

    def communication_handler(self):
        """Handle sending commands to RPI"""
        while True:
            try:
                if not self.command_queue.empty():
                    command_type, data = self.command_queue.get()
                    response = self.send_zmq_command(command_type, data)
                    if response:
                        self.message_queue.put(f"Command response: {response['message']}")
                time.sleep(0.01)
            except Exception as e:
                self.message_queue.put(f"Communication error: {e}")
    
    def response_handler(self):
        """Handle responses and update UI"""
        while True:
            try:
                if not self.message_queue.empty():
                    message = self.message_queue.get()
                    self.add_to_monitor(message)
                time.sleep(0.01)
            except Exception as e:
                print(f"Response handler error: {e}")

    def setup_keyboard_controls(self):
        """Setup keyboard controls for movement"""
        self.root.bind('<KeyPress-w>', lambda e: self.handle_movement('forward'))
        self.root.bind('<KeyPress-s>', lambda e: self.handle_movement('backward'))
        self.root.bind('<KeyPress-a>', lambda e: self.handle_movement('left'))
        self.root.bind('<KeyPress-d>', lambda e: self.handle_movement('right'))
        self.root.bind('<KeyPress-space>', lambda e: self.handle_movement('stop'))
        
        # Add key release handlers to stop movement
        self.root.bind('<KeyRelease-w>', lambda e: self.handle_movement('stop'))
        self.root.bind('<KeyRelease-s>', lambda e: self.handle_movement('stop'))
        self.root.bind('<KeyRelease-a>', lambda e: self.handle_movement('stop'))
        self.root.bind('<KeyRelease-d>', lambda e: self.handle_movement('stop'))

    def handle_movement(self, direction):
        """Handle movement commands"""
        try:
            if direction == 'forward':
                self.command_queue.put(('update_motor', {'motor_id': 'LDC', 'value': 200}))
                self.command_queue.put(('update_motor', {'motor_id': 'RDC', 'value': 200}))
            elif direction == 'backward':
                self.command_queue.put(('update_motor', {'motor_id': 'LDC', 'value': -200}))
                self.command_queue.put(('update_motor', {'motor_id': 'RDC', 'value': -200}))
            elif direction == 'left':
                self.command_queue.put(('update_motor', {'motor_id': 'LDC', 'value': -200}))
                self.command_queue.put(('update_motor', {'motor_id': 'RDC', 'value': 200}))
            elif direction == 'right':
                self.command_queue.put(('update_motor', {'motor_id': 'LDC', 'value': 200}))
                self.command_queue.put(('update_motor', {'motor_id': 'RDC', 'value': -200}))
            elif direction == 'stop':
                self.command_queue.put(('update_motor', {'motor_id': 'LDC', 'value': 0}))
                self.command_queue.put(('update_motor', {'motor_id': 'RDC', 'value': 0}))
        except Exception as e:
            self.message_queue.put(f"Error in movement control: {e}")

    def create_movement_control_panel(self, parent):
        """Create movement control buttons"""
        control_frame = ttk.LabelFrame(parent, text="Movement Controls")
        control_frame.pack(fill='x', padx=10, pady=5)
        
        # Add standby button
        standby_button = ttk.Button(control_frame, text="Standby Position", command=self.enter_standby)
        standby_button.pack(pady=5)
        
        # Add walking control button
        walk_button = ttk.Button(control_frame, text="Start Walking", command=self.toggle_walking)
        walk_button.pack(pady=5)
        self.is_walking = False
        self.walk_button = walk_button
        
        ttk.Label(control_frame, text="Keyboard Controls:", style='Header.TLabel').pack(pady=5)
        ttk.Label(control_frame, text="W: Forward\nS: Backward\nA: Turn Left\nD: Turn Right\nSpace: Stop",
                 justify='left').pack(padx=10, pady=5)

    def enter_standby(self):
        """Put hexapod in standby position with offsets"""
        self.message_queue.put("Entering standby position...")
        for motor_id, angle in self.standby_angles.items():
            # Send raw standby angles to update_servo which will handle inversion and offset
            self.update_servo(motor_id, angle)
        
        self.message_queue.put("Standby position reached")

    def toggle_walking(self):
        """Toggle walking sequence"""
        self.is_walking = not self.is_walking
        if self.is_walking:
            self.walk_button.configure(text="Stop Walking")
            self.walking_thread = threading.Thread(target=self.walking_sequence, daemon=True)
            self.walking_thread.start()
        else:
            self.walk_button.configure(text="Start Walking")
            # Return to standby position when stopping
            self.enter_standby()

    def walking_sequence(self):
        """Execute walking sequence"""
        # Default angles for each motor
        default_angles = {
            # Left Front Leg
            'L1': 90,  # Hip
            'L2': 120, # Knee
            'L3': 90,  # Ankle
            
            # Left Center Leg
            'L5': 90,  # Hip
            'L6': 120, # Knee
            'L7': 90,  # Ankle
            'L8': 90,  # Extra joint
            
            # Left Back Leg
            'L9': 90,  # Hip
            'L10': 120, # Knee
            'L12': 90,  # Ankle
            
            # Right Front Leg
            'R14': 90, # Knee
            'R15': 90, # Hip
            'R16': 90, # Ankle
            
            # Right Center Leg
            'R6': 90,  # Hip
            'R8': 90,  # Ankle
            'R10': 120, # Knee
            'R12': 90, # Extra joint
            
            # Right Back Leg
            'R1': 120, # Knee
            'R2': 90,  # Hip
            'R3': 90   # Ankle
        }

        # Define leg groups for tripod gait
        tripod_1 = {
            'left_front': ['L1', 'L2', 'L3'],
            'right_center': ['R6', 'R10', 'R8'],
            'left_back': ['L9', 'L10', 'L12']
        }
        
        tripod_2 = {
            'right_front': ['R15', 'R14', 'R16'],
            'left_center': ['L5', 'L6', 'L7'],
            'right_back': ['R2', 'R1', 'R3']
        }

        def move_leg(leg_motors, phase):
            """Move a single leg through a walking phase
            phase: 'lift', 'forward', 'down', or 'backward'"""
            hip, knee, ankle = leg_motors
            
            if phase == 'lift':
                # Lift leg by adjusting knee and ankle
                self.command_queue.put(('update_motor', {'motor_id': knee, 'value': default_angles[knee] - 30}))
                self.command_queue.put(('update_motor', {'motor_id': ankle, 'value': default_angles[ankle] + 20}))
            
            elif phase == 'forward':
                # Move leg forward by rotating hip
                self.command_queue.put(('update_motor', {'motor_id': hip, 'value': default_angles[hip] + 25}))
            
            elif phase == 'down':
                # Lower leg by adjusting knee and ankle back to default
                self.command_queue.put(('update_motor', {'motor_id': knee, 'value': default_angles[knee]}))
                self.command_queue.put(('update_motor', {'motor_id': ankle, 'value': default_angles[ankle]}))
            
            elif phase == 'backward':
                # Move leg backward by rotating hip back to default
                self.command_queue.put(('update_motor', {'motor_id': hip, 'value': default_angles[hip]}))

        while self.is_walking:
            try:
                # Move tripod 1
                for leg_name, motors in tripod_1.items():
                    move_leg(motors, 'lift')
                time.sleep(0.2)
                
                for leg_name, motors in tripod_1.items():
                    move_leg(motors, 'forward')
                for leg_name, motors in tripod_2.items():
                    move_leg(motors, 'backward')
                time.sleep(0.2)
                
                for leg_name, motors in tripod_1.items():
                    move_leg(motors, 'down')
                time.sleep(0.2)

                # Move tripod 2
                for leg_name, motors in tripod_2.items():
                    move_leg(motors, 'lift')
                time.sleep(0.2)
                
                for leg_name, motors in tripod_2.items():
                    move_leg(motors, 'forward')
                for leg_name, motors in tripod_1.items():
                    move_leg(motors, 'backward')
                time.sleep(0.2)
                
                for leg_name, motors in tripod_2.items():
                    move_leg(motors, 'down')
                time.sleep(0.2)

            except Exception as e:
                self.message_queue.put(f"Walking sequence error: {e}")
                self.is_walking = False
                self.walk_button.configure(text="Start Walking")
                break

    def create_serial_monitor(self):
        """Create serial monitor frame"""
        monitor_frame = ttk.LabelFrame(self.root, text="Serial Monitor")
        monitor_frame.pack(fill='x', padx=10, pady=5)
        
        # Create text widget for displaying messages
        self.monitor_text = tk.Text(monitor_frame, height=10, width=50, font=('Consolas', 10))
        self.monitor_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(monitor_frame, orient='vertical', command=self.monitor_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.monitor_text.configure(yscrollcommand=scrollbar.set)
        
        # Add clear button
        clear_button = ttk.Button(monitor_frame, text="Clear Monitor", command=self.clear_monitor)
        clear_button.pack(pady=5)
        
        # Make text widget read-only
        self.monitor_text.configure(state='disabled')

    def clear_monitor(self):
        """Clear the serial monitor"""
        self.monitor_text.configure(state='normal')
        self.monitor_text.delete(1.0, tk.END)
        self.monitor_text.configure(state='disabled')

    def add_to_monitor(self, message):
        """Add message to serial monitor"""
        self.monitor_text.configure(state='normal')
        self.monitor_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} > {message}\n")
        self.monitor_text.see(tk.END)  # Auto-scroll to bottom
        self.monitor_text.configure(state='disabled')

    def update_offset(self, motor_id, offset_str):
        """Update motor offset and save to config"""
        try:
            offset = int(float(offset_str))
            self.motor_values['offsets'][motor_id] = offset
            
            # Re-send the current value to apply new offset
            current_value = self.motor_values['servo_motors'].get(motor_id, 90)
            self.update_servo(motor_id, current_value)
            
            self.message_queue.put(f"Updated offset for {motor_id} to {offset}")
        except ValueError:
            self.message_queue.put(f"Invalid offset value for {motor_id}")

    def update_servo_with_offset(self, motor_id, value, offset_str, is_inverted=False):
        """Update servo position considering offset and inversion"""
        try:
            # Update the configuration
            offset = int(float(offset_str))
            self.motor_values['offsets'][motor_id] = offset
            self.motor_values['inverted_motors'][motor_id] = is_inverted
            
            # Send the raw value to update_servo which will handle inversion and offset
            self.update_servo(motor_id, value)
        except ValueError:
            self.message_queue.put(f"Invalid offset value for {motor_id}")

    def update_motor_inversion(self, motor_id, invert_var):
        """Update motor inversion setting and save to config"""
        self.motor_values['inverted_motors'][motor_id] = invert_var.get()
        
        # Re-send the current value to apply new inversion setting
        current_value = self.motor_values['servo_motors'].get(motor_id, 90)
        self.update_servo(motor_id, current_value)
        
        self.message_queue.put(f"Updated inversion for {motor_id} to {invert_var.get()}")

def main():
    root = tk.Tk()
    app = HexapodGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
