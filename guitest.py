import tkinter as tk
from tkinter import ttk
import zmq
import threading
import queue
import time
from config_handler import ConfigHandler
import json
import os
from tkinter import filedialog

class HexapodGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Hexapod Controller")
        
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://192.168.146.192:5555")  # Replace with your RPI's IP
        
        # Initialize config handlers for both configs
        self.config_handler = ConfigHandler()
        self.standby_config_handler = ConfigHandler('standby.json')
        self.motor_values = self.config_handler.load_config()
        self.standby_values = self.standby_config_handler.load_config()
        
        # Initialize keyframe sequence list
        self.keyframe_sequences = []
        self.current_sequence = []
        self.sequence_running = False
        
        # Create serial monitor first (since other methods might need to log messages)
        self.create_serial_monitor()
        
        # Define standby angles for stable standing position
        self.standby_angles = {
            # Left Front Leg
            'L1': 0.0,   # Hip centered
            'L2': 45.0,  # Knee bent for stability
            'L3': 115.0,   # Ankle angled for ground contact
            
            # Left Center Leg
            'L5': 50.0,   # Hip centered
            'L6': 180.0,  # Knee bent
            'L7': 90.0,   # Ankle angled
            'L8': 0.0,   # Extra joint centered
            
            # Left Back Leg
            'L9': 115.0,   # Hip centered
            'L10': 45.0, # Knee bent
            'L12': 0.0,  # Ankle angled
            
            # Right Front Leg
            'R14': 45.0, # Knee bent
            'R15': 115.0,  # Hip centered
            'R16': 0.0,  # Ankle angled
            
            # Right Center Leg
            'R6': 50.0,   # Hip centered
            'R8': 0.0,   # Ankle angled
            'R10': 180.0, # Knee bent
            'R12': 90.0,  # Extra joint centered
            
            # Right Back Leg
            'R1': 45.0,  # Knee bent
            'R2': 115.0,   # Hip centered   
            'R3': 0.0    # Ankle angled
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
        style.configure('Emergency.TButton', 
                        background='red', 
                        foreground='white',
                        font=('Helvetica', 10, 'bold'))
    
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
        
        # Balance Control Tab
        balance_frame = ttk.Frame(notebook)
        notebook.add(balance_frame, text="Balance Control")
        self.create_balance_control_panel(balance_frame)
    
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

        # Add 'Update All' button
        update_all_button = ttk.Button(scrollable_frame, text="Update All", command=self.update_all_servos)
        update_all_button.pack(pady=5)

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
        
        # Send the adjusted value directly to the motor
        self.send_zmq_command('update_motor', {
            'motor_id': motor_id,
            'value': int(adjusted_value)
        })

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
        self.send_zmq_command('update_motor', {
            'motor_id': motor_id,
            'value': int(value)
        })
        self.motor_values['dc_motors'][motor_id] = int(value)
        self.config_handler.save_config(self.motor_values)

    def handle_movement(self, direction):
        """Handle movement commands"""
        try:
            # Speed levels
            SPEED_LOW = 50
            SPEED_MED = 100
            SPEED_HIGH = 200
            
            if direction == 'forward':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': SPEED_MED})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': SPEED_MED})
            elif direction == 'forward_fast':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': SPEED_HIGH})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': SPEED_HIGH})
            elif direction == 'forward_slow':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': SPEED_LOW})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': SPEED_LOW})
            elif direction == 'backward':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': -SPEED_MED})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': -SPEED_MED})
            elif direction == 'backward_fast':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': -SPEED_HIGH})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': -SPEED_HIGH})
            elif direction == 'backward_slow':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': -SPEED_LOW})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': -SPEED_LOW})
            elif direction == 'left':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': -SPEED_MED})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': SPEED_MED})
            elif direction == 'right':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': SPEED_MED})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': -SPEED_MED})
            elif direction == 'rotate_left':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': -SPEED_LOW})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': SPEED_LOW})
            elif direction == 'rotate_right':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': SPEED_LOW})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': -SPEED_LOW})
            elif direction == 'left_motor_forward':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': SPEED_MED})
            elif direction == 'left_motor_backward':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': -SPEED_MED})
            elif direction == 'right_motor_forward':
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': SPEED_MED})
            elif direction == 'right_motor_backward':
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': -SPEED_MED})
            elif direction == 'stop':
                self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': 0})
                self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': 0})
        except Exception as e:
            print(f"Error in movement control: {e}")

    def create_balance_control_panel(self, parent):
        """Create movement control interface"""
        # Create main frame with notebook for sub-tabs
        main_frame = ttk.Notebook(parent)
        main_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Movement Control Tab
        movement_tab = ttk.Frame(main_frame)
        main_frame.add(movement_tab, text="Movement Control")
        self.create_movement_controls(movement_tab)
        
        # Keyframe Control Tab
        keyframe_tab = ttk.Frame(main_frame)
        main_frame.add(keyframe_tab, text="Keyframe Control")
        self.create_keyframe_controls(keyframe_tab)
    
    def create_movement_controls(self, parent):
        """Create movement control panel"""
        # Create left frame for controls
        left_frame = ttk.LabelFrame(parent, text="Controls")
        left_frame.pack(side='left', fill='both', expand=True, padx=5, pady=5)
        
        # Create right frame for instructions
        right_frame = ttk.LabelFrame(parent, text="Instructions")
        right_frame.pack(side='right', fill='both', expand=True, padx=5, pady=5)
        
        # Add control sections to left frame
        # Special Functions Section
        special_frame = ttk.LabelFrame(left_frame, text="Special Functions")
        special_frame.pack(fill='x', padx=5, pady=5)
        
        # Add standby and walking controls in a row
        standby_button = ttk.Button(special_frame, text="Standby Position", command=self.enter_standby)
        standby_button.pack(side='left', fill='x', expand=True, padx=5, pady=5)
        
        walk_button = ttk.Button(special_frame, text="Start Walking", command=self.toggle_walking)
        walk_button.pack(side='left', fill='x', expand=True, padx=5, pady=5)
        self.is_walking = False
        self.walk_button = walk_button
        
        emergency_stop = ttk.Button(special_frame, text="EMERGENCY STOP", 
                                  command=lambda: self.handle_movement('stop'),
                                  style='Emergency.TButton')
        emergency_stop.pack(side='left', fill='x', expand=True, padx=5, pady=5)
        
        # Movement Controls Section
        movement_frame = ttk.LabelFrame(left_frame, text="Movement Controls")
        movement_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Create a 3x3 grid for movement buttons
        buttons = [
            (None, "Forward", 'forward', None),
            (None, "Rotate Left", 'rotate_left', "Rotate Right"),
            ("Left", "Stop", 'stop', "Right"),
            (None, "Backward", 'backward', None)
        ]
        
        for row, (left_cmd, center_cmd, center_action, right_cmd) in enumerate(buttons):
            frame = ttk.Frame(movement_frame)
            frame.pack(fill='x', padx=5, pady=2)
            
            # Left button
            if left_cmd:
                btn = ttk.Button(frame, text=left_cmd, 
                               command=lambda cmd=left_cmd.lower(): self.handle_movement(cmd))
                btn.pack(side='left', fill='x', expand=True, padx=2)
            
            # Center button
            if center_cmd:
                btn = ttk.Button(frame, text=center_cmd, 
                               command=lambda cmd=center_action: self.handle_movement(cmd))
                btn.pack(side='left', fill='x', expand=True, padx=2)
            
            # Right button
            if right_cmd:
                btn = ttk.Button(frame, text=right_cmd, 
                               command=lambda cmd=right_cmd.lower(): self.handle_movement(cmd))
                btn.pack(side='left', fill='x', expand=True, padx=2)
        
        # Speed Control Section
        speed_frame = ttk.LabelFrame(left_frame, text="Speed Control")
        speed_frame.pack(fill='x', padx=5, pady=5)
        
        speeds = [
            ("Slow Speed", 'slow', 50),
            ("Medium Speed", 'medium', 100),
            ("High Speed", 'high', 200)
        ]
        
        for text, speed, value in speeds:
            btn = ttk.Radiobutton(speed_frame, text=f"{text} ({value})",
                                value=value, command=lambda v=value: self.set_movement_speed(v))
            btn.pack(side='left', fill='x', expand=True, padx=5, pady=2)
        
        # Add instructions to right frame
        ttk.Label(right_frame, text="Keyboard Controls:", style='Header.TLabel').pack(pady=5)
        controls_text = """
Movement Controls:
W/S: Forward/Backward
A/D: Turn Left/Right
Q/E: Rotate Left/Right
Space: Emergency Stop

Speed Controls:
Shift + W/S: Fast Forward/Backward
Ctrl + W/S: Slow Forward/Backward
Normal W/S: Medium Speed

Individual Motor Controls:
U/J: Left Motor Forward/Backward
I/K: Right Motor Forward/Backward

Note: 
- All motors stop automatically when keys are released
- Use Shift/Ctrl for speed control
- Individual motor controls help in fine adjustments
"""
        ttk.Label(right_frame, text=controls_text, justify='left').pack(padx=10, pady=5)

    def enter_standby(self):
        """Put hexapod in standby position with offsets"""
        self.add_to_monitor("Entering standby position...")
        
        # Load standby positions from standby.json
        standby_config = self.standby_config_handler.load_config()
        
        # Send commands sequentially with delays
        for motor_id, angle in standby_config['servo_motors'].items():
            # Send raw standby angles to update_servo which will handle inversion and offset
            self.update_servo(motor_id, angle)
            time.sleep(0.1)  # Add delay between servo commands
        
        # Stop DC motors
        self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': 0})
        self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': 0})
        
        self.add_to_monitor("Standby position reached")

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
        """Execute walking sequence using tripod gait"""
        try:
            # Load angles from standby.json for default position
            original_config_file = self.config_handler.config_file
            self.config_handler.config_file = 'standby.json'
            default_angles = self.config_handler.load_config()['servo_motors']
            self.config_handler.config_file = original_config_file

            # Define leg groups for tripod gait
            tripod_1 = {
                'left_front': ['L1', 'L2', 'L3'],
                'right_center': ['R6', 'R10', 'R8'],
                'left_back': ['L9', 'L10', 'L12']
            }
            
            tripod_2 = {
                'right_front': ['R14', 'R15', 'R16'],
                'left_center': ['L5', 'L6', 'L7'],
                'right_back': ['R1', 'R2', 'R3']
            }

            def move_leg(leg_motors, phase):
                """Move a single leg through a walking phase
                phase: 'lift', 'forward', 'down', or 'backward'"""
                hip, knee, ankle = leg_motors
                
                if phase == 'lift':
                    # Lift leg by adjusting knee and ankle
                    self.update_servo(knee, default_angles[knee] - 30)
                    self.update_servo(ankle, default_angles[ankle] + 20)
                
                elif phase == 'forward':
                    # Move leg forward by rotating hip
                    self.update_servo(hip, default_angles[hip] + 25)
                
                elif phase == 'down':
                    # Lower leg by adjusting knee and ankle back to default
                    self.update_servo(knee, default_angles[knee])
                    self.update_servo(ankle, default_angles[ankle])
                
                elif phase == 'backward':
                    # Move leg backward by rotating hip back to default
                    self.update_servo(hip, default_angles[hip])

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
                    self.add_to_monitor(f"Walking sequence error: {e}")
                    self.is_walking = False
                    self.walk_button.configure(text="Start Walking")
                    break

        except Exception as e:
            self.add_to_monitor(f"Walking sequence error: {e}")
            self.is_walking = False
            self.walk_button.configure(text="Start Walking")

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
            
            self.add_to_monitor(f"Updated offset for {motor_id} to {offset}")
        except ValueError:
            self.add_to_monitor(f"Invalid offset value for {motor_id}")

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
            self.add_to_monitor(f"Invalid offset value for {motor_id}")

    def update_motor_inversion(self, motor_id, invert_var):
        """Update motor inversion setting and save to config"""
        self.motor_values['inverted_motors'][motor_id] = invert_var.get()
        
        # Re-send the current value to apply new inversion setting
        current_value = self.motor_values['servo_motors'].get(motor_id, 90)
        self.update_servo(motor_id, current_value)
        
        self.add_to_monitor(f"Updated inversion for {motor_id} to {invert_var.get()}")

    def update_all_servos(self):
        """Update all servos with current slider values, offsets, and inversion settings"""
        for leg_group, motors in self.motor_groups.items():
            for motor_id in motors.keys():
                # Get current slider value
                slider_value = self.motor_values['servo_motors'].get(motor_id, 90)
                # Get current offset
                offset = self.motor_values['offsets'].get(motor_id, 0)
                # Get inversion setting
                is_inverted = self.motor_values['inverted_motors'].get(motor_id, False)
                # Update servo with current settings
                self.update_servo_with_offset(motor_id, slider_value, offset, is_inverted)

    def setup_keyboard_controls(self):
        """Setup keyboard controls for movement"""
        # Main movement controls
        self.root.bind('<KeyPress-w>', lambda e: self.handle_movement('forward'))
        self.root.bind('<KeyPress-s>', lambda e: self.handle_movement('backward'))
        self.root.bind('<KeyPress-a>', lambda e: self.handle_movement('left'))
        self.root.bind('<KeyPress-d>', lambda e: self.handle_movement('right'))
        self.root.bind('<KeyPress-space>', lambda e: self.handle_movement('stop'))
        
        # Speed variations with Shift key
        self.root.bind('<Shift-W>', lambda e: self.handle_movement('forward_fast'))
        self.root.bind('<Shift-S>', lambda e: self.handle_movement('backward_fast'))
        
        # Speed variations with Control key
        self.root.bind('<Control-w>', lambda e: self.handle_movement('forward_slow'))
        self.root.bind('<Control-s>', lambda e: self.handle_movement('backward_slow'))
        
        # Rotation controls
        self.root.bind('<KeyPress-q>', lambda e: self.handle_movement('rotate_left'))
        self.root.bind('<KeyPress-e>', lambda e: self.handle_movement('rotate_right'))
        
        # Individual motor controls
        self.root.bind('<KeyPress-u>', lambda e: self.handle_movement('left_motor_forward'))
        self.root.bind('<KeyPress-j>', lambda e: self.handle_movement('left_motor_backward'))
        self.root.bind('<KeyPress-i>', lambda e: self.handle_movement('right_motor_forward'))
        self.root.bind('<KeyPress-k>', lambda e: self.handle_movement('right_motor_backward'))
        
        # Stop on key release for all movement keys
        movement_keys = ['w', 's', 'a', 'd', 'q', 'e', 'u', 'j', 'i', 'k']
        for key in movement_keys:
            self.root.bind(f'<KeyRelease-{key}>', lambda e: self.handle_movement('stop'))
            self.root.bind(f'<KeyRelease-{key.upper()}>', lambda e: self.handle_movement('stop'))

    def set_movement_speed(self, speed):
        """Set the movement speed for DC motors"""
        self.current_speed = speed
        self.add_to_monitor(f"Movement speed set to: {speed}")

    def create_keyframe_controls(self, parent):
        """Create keyframe control interface"""
        # Create main container
        container = ttk.Frame(parent)
        container.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Left panel for sequence controls
        left_panel = ttk.LabelFrame(container, text="Sequence Controls")
        left_panel.pack(side='left', fill='both', expand=True, padx=5, pady=5)
        
        # Right panel for keyframe editor
        right_panel = ttk.LabelFrame(container, text="Keyframe Editor")
        right_panel.pack(side='right', fill='both', expand=True, padx=5, pady=5)
        
        # Sequence Controls
        sequence_frame = ttk.Frame(left_panel)
        sequence_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(sequence_frame, text="Sequence Name:").pack(side='left', padx=5)
        self.sequence_name = tk.StringVar(value="New Sequence")
        ttk.Entry(sequence_frame, textvariable=self.sequence_name).pack(side='left', fill='x', expand=True, padx=5)
        
        # Buttons for sequence control
        btn_frame = ttk.Frame(left_panel)
        btn_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(btn_frame, text="New Sequence", 
                  command=self.new_sequence).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Save Sequence", 
                  command=self.save_sequence).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Load Sequence", 
                  command=self.load_sequence).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Play Sequence", 
                  command=self.play_sequence).pack(side='left', padx=2)
        
        # Keyframe list
        ttk.Label(left_panel, text="Keyframes:").pack(anchor='w', padx=5)
        self.keyframe_listbox = tk.Listbox(left_panel, height=10)
        self.keyframe_listbox.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Keyframe Editor
        editor_frame = ttk.Frame(right_panel)
        editor_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Time control
        time_frame = ttk.Frame(editor_frame)
        time_frame.pack(fill='x', pady=5)
        ttk.Label(time_frame, text="Duration (ms):").pack(side='left', padx=5)
        self.duration_var = tk.StringVar(value="1000")
        ttk.Entry(time_frame, textvariable=self.duration_var, width=10).pack(side='left', padx=5)
        
        # Servo angle controls
        angle_frame = ttk.LabelFrame(editor_frame, text="Servo Angles")
        angle_frame.pack(fill='both', expand=True, pady=5)
        
        # Create servo angle controls by leg groups
        self.angle_vars = {}
        row = 0
        col = 0
        for leg_group, motors in self.motor_groups.items():
            group_frame = ttk.LabelFrame(angle_frame, text=leg_group.replace('_', ' ').title())
            group_frame.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')
            
            for motor_id, motor_name in motors.items():
                motor_frame = ttk.Frame(group_frame)
                motor_frame.pack(fill='x', pady=2)
                
                ttk.Label(motor_frame, text=f"{motor_name}:").pack(side='left', padx=2)
                self.angle_vars[motor_id] = tk.StringVar(value="0")
                ttk.Entry(motor_frame, textvariable=self.angle_vars[motor_id], 
                         width=5, validate='key', 
                         validatecommand=(self.root.register(self.validate_input), '%P')).pack(side='right', padx=2)
            
            col += 1
            if col > 2:  # 3 columns layout
                col = 0
                row += 1
        
        # Keyframe control buttons
        control_frame = ttk.Frame(editor_frame)
        control_frame.pack(fill='x', pady=5)
        
        ttk.Button(control_frame, text="Add Keyframe", 
                  command=self.add_keyframe).pack(side='left', padx=2)
        ttk.Button(control_frame, text="Update Keyframe", 
                  command=self.update_keyframe).pack(side='left', padx=2)
        ttk.Button(control_frame, text="Delete Keyframe", 
                  command=self.delete_keyframe).pack(side='left', padx=2)
        ttk.Button(control_frame, text="Test Keyframe", 
                  command=self.test_keyframe).pack(side='left', padx=2)
        
        # Instructions
        ttk.Label(right_panel, text="Instructions:", style='Header.TLabel').pack(anchor='w', padx=5, pady=5)
        instructions = """
1. Create a new sequence or load an existing one
2. Set the duration for the movement
3. Set angles for the servos you want to move
4. Add the keyframe to the sequence
5. Repeat steps 2-4 for each position
6. Save the sequence when done
7. Use Play to test the sequence

Tips:
- Leave angle empty to keep current position
- Use Test Keyframe to preview position
- Duration is the time to reach the position
"""
        ttk.Label(right_panel, text=instructions, justify='left').pack(fill='x', padx=5)

    def new_sequence(self):
        """Create a new keyframe sequence"""
        self.current_sequence = []
        self.keyframe_listbox.delete(0, tk.END)
        self.sequence_name.set("New Sequence")
        self.add_to_monitor("Created new sequence")

    def add_keyframe(self):
        """Add current angles as a keyframe"""
        try:
            duration = int(self.duration_var.get())
            angles = {}
            
            # Collect only the angles that are specified
            for motor_id, var in self.angle_vars.items():
                if var.get().strip():  # Only add if value is specified
                    angles[motor_id] = float(var.get())
            
            if angles:  # Only add if at least one angle is specified
                keyframe = {
                    'duration': duration,
                    'angles': angles
                }
                self.current_sequence.append(keyframe)
                self.keyframe_listbox.insert(tk.END, 
                    f"Frame {len(self.current_sequence)}: {len(angles)} servos, {duration}ms")
                self.add_to_monitor(f"Added keyframe with {len(angles)} servo positions")
            else:
                self.add_to_monitor("Error: No angles specified for keyframe")
        except ValueError as e:
            self.add_to_monitor(f"Error adding keyframe: {e}")

    def update_keyframe(self):
        """Update selected keyframe"""
        selection = self.keyframe_listbox.curselection()
        if not selection:
            self.add_to_monitor("No keyframe selected")
            return
        
        try:
            index = selection[0]
            duration = int(self.duration_var.get())
            angles = {}
            
            for motor_id, var in self.angle_vars.items():
                if var.get().strip():
                    angles[motor_id] = float(var.get())
            
            if angles:
                self.current_sequence[index] = {
                    'duration': duration,
                    'angles': angles
                }
                self.keyframe_listbox.delete(index)
                self.keyframe_listbox.insert(index, 
                    f"Frame {index + 1}: {len(angles)} servos, {duration}ms")
                self.add_to_monitor(f"Updated keyframe {index + 1}")
        except Exception as e:
            self.add_to_monitor(f"Error updating keyframe: {e}")

    def delete_keyframe(self):
        """Delete selected keyframe"""
        selection = self.keyframe_listbox.curselection()
        if not selection:
            self.add_to_monitor("No keyframe selected")
            return
        
        index = selection[0]
        self.current_sequence.pop(index)
        self.keyframe_listbox.delete(index)
        self.add_to_monitor(f"Deleted keyframe {index + 1}")

    def test_keyframe(self):
        """Test current keyframe settings"""
        try:
            angles = {}
            for motor_id, var in self.angle_vars.items():
                if var.get().strip():
                    angles[motor_id] = float(var.get())
            
            if angles:
                for motor_id, angle in angles.items():
                    self.update_servo(motor_id, angle)
                self.add_to_monitor("Testing keyframe positions")
            else:
                self.add_to_monitor("No angles specified to test")
        except Exception as e:
            self.add_to_monitor(f"Error testing keyframe: {e}")

    def play_sequence(self):
        """Play the current sequence"""
        if not self.current_sequence:
            self.add_to_monitor("No sequence to play")
            return
        
        if self.sequence_running:
            self.sequence_running = False
            self.add_to_monitor("Stopping sequence")
            return
        
        self.sequence_running = True
        threading.Thread(target=self._play_sequence_thread, daemon=True).start()

    def _play_sequence_thread(self):
        """Thread function to play the sequence"""
        try:
            while self.sequence_running:
                for frame in self.current_sequence:
                    if not self.sequence_running:
                        break
                    
                    for motor_id, angle in frame['angles'].items():
                        self.update_servo(motor_id, angle)
                    
                    time.sleep(frame['duration'] / 1000.0)
                
                if not self.sequence_running:
                    break
        except Exception as e:
            self.add_to_monitor(f"Error playing sequence: {e}")
        finally:
            self.sequence_running = False

    def save_sequence(self):
        """Save the current sequence to a file"""
        if not self.current_sequence:
            self.add_to_monitor("No sequence to save")
            return
        
        try:
            filename = f"sequences/{self.sequence_name.get()}.json"
            os.makedirs("sequences", exist_ok=True)
            
            with open(filename, 'w') as f:
                json.dump({
                    'name': self.sequence_name.get(),
                    'frames': self.current_sequence
                }, f, indent=4)
            
            self.add_to_monitor(f"Saved sequence to {filename}")
        except Exception as e:
            self.add_to_monitor(f"Error saving sequence: {e}")

    def load_sequence(self):
        """Load a sequence from a file"""
        try:
            filename = filedialog.askopenfilename(
                initialdir="sequences",
                title="Select Sequence File",
                filetypes=(("JSON files", "*.json"), ("all files", "*.*"))
            )
            
            if filename:
                with open(filename, 'r') as f:
                    data = json.load(f)
                
                self.current_sequence = data['frames']
                self.sequence_name.set(data['name'])
                
                self.keyframe_listbox.delete(0, tk.END)
                for i, frame in enumerate(self.current_sequence):
                    self.keyframe_listbox.insert(tk.END, 
                        f"Frame {i + 1}: {len(frame['angles'])} servos, {frame['duration']}ms")
                
                self.add_to_monitor(f"Loaded sequence from {filename}")
        except Exception as e:
            self.add_to_monitor(f"Error loading sequence: {e}")

def main():
    root = tk.Tk()
    print("Root created")
    app = HexapodGUI(root)
    print("App created")
    root.mainloop()
    print("Mainloop started")
    

if __name__ == "__main__":
    main()
