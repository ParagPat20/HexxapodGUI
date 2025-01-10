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

class KeyframePopup:
    def __init__(self, parent, motor_groups, update_servo_callback):
        self.window = tk.Toplevel(parent)
        self.window.title("Add Keyframe")
        # self.window.geometry("600x800")
        
        self.motor_groups = motor_groups
        self.update_servo = update_servo_callback
        self.servo_entries = {}
        self.dc_entries = {}
        
        # Create main container with scrollbar
        container = ttk.Frame(self.window)
        container.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Create canvas for scrolling
        canvas = tk.Canvas(container)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Pack scrollbar and canvas
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        
        # Duration control
        duration_frame = ttk.LabelFrame(self.scrollable_frame, text="Duration")
        duration_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(duration_frame, text="Duration (ms):").pack(side='left', padx=5)
        self.duration_var = tk.StringVar(value="1000")
        ttk.Entry(duration_frame, textvariable=self.duration_var, width=10).pack(side='left', padx=5)
        
        # DC Motors control
        dc_frame = ttk.LabelFrame(self.scrollable_frame, text="DC Motors")
        dc_frame.pack(fill='x', padx=5, pady=5)
        
        dc_motors = {
            'LDC': 'Left DC Motor',
            'RDC': 'Right DC Motor'
        }
        
        for motor_id, motor_name in dc_motors.items():
            motor_frame = ttk.Frame(dc_frame)
            motor_frame.pack(fill='x', pady=2)
            
            ttk.Label(motor_frame, text=f"{motor_name}:").pack(side='left', padx=5)
            value_var = tk.StringVar(value="0")
            self.dc_entries[motor_id] = value_var
            entry = ttk.Entry(motor_frame, textvariable=value_var, width=8)
            entry.pack(side='left', padx=5)
            
            # Add test button
            ttk.Button(motor_frame, text="Test",
                      command=lambda m=motor_id, v=value_var: self.test_dc_motor(m, v)).pack(side='left', padx=5)
        
        # Servo Motors by group
        for group_name, motors in self.motor_groups.items():
            group_frame = ttk.LabelFrame(self.scrollable_frame, text=group_name.replace('_', ' ').title())
            group_frame.pack(fill='x', padx=5, pady=5)
            
            for motor_id, motor_name in motors.items():
                motor_frame = ttk.Frame(group_frame)
                motor_frame.pack(fill='x', pady=2)
                
                ttk.Label(motor_frame, text=f"{motor_name} ({motor_id}):").pack(side='left', padx=5)
                value_var = tk.StringVar(value="")
                self.servo_entries[motor_id] = value_var
                entry = ttk.Entry(motor_frame, textvariable=value_var, width=8)
                entry.pack(side='left', padx=5)
                
                # Add test button
                ttk.Button(motor_frame, text="Test",
                          command=lambda m=motor_id, v=value_var: self.test_servo(m, v)).pack(side='left', padx=5)
        
        # Control buttons
        button_frame = ttk.Frame(self.scrollable_frame)
        button_frame.pack(fill='x', pady=10)
        
        ttk.Button(button_frame, text="Add Keyframe", command=self.add_keyframe).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Cancel", command=self.window.destroy).pack(side='left', padx=5)
        
        self.result = None
        
    def test_servo(self, motor_id, value_var):
        """Test a servo motor position"""
        try:
            if value_var.get().strip():
                angle = float(value_var.get())
                self.update_servo(motor_id, angle)
        except ValueError:
            pass
    
    def test_dc_motor(self, motor_id, value_var):
        """Test a DC motor speed"""
        try:
            if value_var.get().strip():
                speed = float(value_var.get())
                self.update_servo(motor_id, speed)
        except ValueError:
            pass
    
    def add_keyframe(self):
        """Collect all values and create keyframe"""
        try:
            duration = int(self.duration_var.get())
            
            # Collect servo angles
            angles = {}
            for motor_id, var in self.servo_entries.items():
                if var.get().strip():
                    angles[motor_id] = float(var.get())
            
            # Collect DC motor speeds
            dc_speeds = {}
            for motor_id, var in self.dc_entries.items():
                if var.get().strip():
                    dc_speeds[motor_id] = float(var.get())
            
            self.result = {
                'duration': duration,
                'angles': angles,
                'dc_speeds': dc_speeds
            }
            
            self.window.destroy()
        except ValueError:
            pass

class HexapodGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Hexapod Controller")
        
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://192.168.8.39:5555")  # Replace with your RPI's IP
        
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
                "L1": "Coxa",    # Front Leg
                "L3": "Femur",   # Front Mid
                "L2": "Tibia"    # Front Lower
            },
            'left_center': {
                "L8": "Coxa",      # Center Leg
                "L5": "Femur2",    # Center Upper
                "L6": "Femur",     # Center Lower 2
                "L7": "Tibia"      # Center Lower
            },
            'left_back': {
                "L12": "Coxa",   # Back Leg
                "L9": "Femur",   # Back Mid
                "L10": "Tibia"   # Back Lower
            },
            'right_front': {
                "R16": "Coxa",   # Front Leg
                "R15": "Femur",  # Front Mid
                "R14": "Tibia"   # Front Lower
            },
            'right_center': {
                "R8": "Coxa",     # Center Leg
                "R6": "Femur2",   # Center Upper
                "R10": "Femur",   # Center Lower 2
                "R12": "Tibia"    # Center Lower
            },
            'right_back': {
                "R3": "Coxa",    # Back Leg
                "R2": "Femur",   # Back Mid
                "R1": "Tibia"    # Back Lower
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
            
            # Create slider with 50-unit increments
            slider = ttk.Scale(motor_frame, from_=-250, to=250, orient='horizontal')
            slider.configure(command=lambda v, s=slider: self._snap_to_increment(s, float(v), 50))
            
            # Set initial value from config (rounded to nearest 50)
            initial_value = self.motor_values['dc_motors'].get(motor_id, 0)
            initial_value = round(initial_value / 50) * 50  # Round to nearest 50
            slider.set(initial_value)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            # Create entry for direct value input
            value_var = tk.StringVar(value=str(initial_value))
            entry = ttk.Entry(motor_frame, textvariable=value_var, width=8, 
                            validate='key', validatecommand=vcmd)
            entry.pack(side='left', padx=5)
            
            # Connect slider and entry
            slider.configure(command=lambda v, var=value_var, e=entry, m=motor_id, s=slider: 
                            self._on_dc_slider_change(v, var, e, m, s))
            slider.bind('<ButtonRelease-1>', lambda e, m=motor_id, s=slider: 
                        self.update_dc_motor(m, float(s.get())))
            entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry, m=motor_id: 
                        self._on_dc_entry_change(e, s, var, ent, m))

    def _snap_to_increment(self, slider, value, increment):
        """Snap slider value to nearest increment"""
        snapped = round(float(value) / increment) * increment
        if float(slider.get()) != snapped:
            slider.set(snapped)
        return snapped

    def _on_dc_slider_change(self, value, value_var, entry, motor_id, slider):
        """Handle slider value changes for DC motor control"""
        try:
            # Snap to nearest 50
            value = self._snap_to_increment(slider, float(value), 50)
            value_var.set(f"{value:.0f}")
            entry.delete(0, tk.END)
            entry.insert(0, f"{value:.0f}")
        except ValueError:
            pass

    def _on_dc_entry_change(self, event, slider, value_var, entry, motor_id):
        """Handle entry value changes for DC motor control"""
        try:
            value = float(entry.get())
            # Snap to nearest 50
            value = round(value / 50) * 50
            value = max(-250, min(250, value))  # Clamp value
            slider.set(value)
            value_var.set(f"{value:.0f}")
            
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
        
        # Sequence Controls
        sequence_frame = ttk.LabelFrame(container, text="Sequence Controls")
        sequence_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Sequence name
        name_frame = ttk.Frame(sequence_frame)
        name_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(name_frame, text="Sequence Name:").pack(side='left', padx=5)
        self.sequence_name = tk.StringVar(value="New Sequence")
        ttk.Entry(name_frame, textvariable=self.sequence_name).pack(side='left', fill='x', expand=True, padx=5)
        
        # Buttons frame
        btn_frame = ttk.Frame(sequence_frame)
        btn_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(btn_frame, text="New Sequence", command=self.new_sequence).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Add Keyframe", command=self.show_keyframe_popup).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Save Sequence", command=self.save_sequence).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Load Sequence", command=self.load_sequence).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Play Sequence", command=self.play_sequence).pack(side='left', padx=2)
        
        # Keyframe list
        list_frame = ttk.LabelFrame(sequence_frame, text="Keyframes")
        list_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        self.keyframe_listbox = tk.Listbox(list_frame, height=15)
        self.keyframe_listbox.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Keyframe control buttons
        control_frame = ttk.Frame(list_frame)
        control_frame.pack(fill='x', pady=5)
        
        ttk.Button(control_frame, text="Delete Keyframe", command=self.delete_keyframe).pack(side='left', padx=2)
        ttk.Button(control_frame, text="Move Up", command=lambda: self.move_keyframe(-1)).pack(side='left', padx=2)
        ttk.Button(control_frame, text="Move Down", command=lambda: self.move_keyframe(1)).pack(side='left', padx=2)
        
        # Instructions
        instructions = """
Instructions:
1. Enter a sequence name
2. Click 'Add Keyframe' to add a new position
3. In the popup, set servo angles and DC motor speeds
4. Use 'Test' buttons to preview positions
5. Click 'Add Keyframe' in popup to add to sequence
6. Arrange keyframes using Move Up/Down
7. Save sequence when done
8. Use Play to test the sequence

Tips:
- Leave values empty to keep current position
- Use Test buttons to preview positions
- Duration is the time to reach the position
"""
        ttk.Label(sequence_frame, text=instructions, justify='left').pack(fill='x', padx=5, pady=5)

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
                    
                    # Update servo positions
                    for motor_id, angle in frame['angles'].items():
                        self.update_servo(motor_id, angle)
                    
                    # Update DC motor speeds
                    for motor_id, speed in frame['dc_speeds'].items():
                        self.update_dc_motor(motor_id, speed)
                    
                    time.sleep(frame['duration'] / 1000.0)
                
                if not self.sequence_running:
                    break
        except Exception as e:
            self.add_to_monitor(f"Error playing sequence: {e}")
        finally:
            self.sequence_running = False
            # Return to standby position when sequence ends
            self.enter_standby()

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

    def show_keyframe_popup(self):
        """Show popup for adding a keyframe"""
        popup = KeyframePopup(self.root, self.motor_groups, self.update_servo)
        self.root.wait_window(popup.window)
        
        if popup.result:
            self.current_sequence.append(popup.result)
            frame_num = len(self.current_sequence)
            servo_count = len(popup.result['angles'])
            dc_count = len(popup.result['dc_speeds'])
            self.keyframe_listbox.insert(tk.END, 
                f"Frame {frame_num}: {servo_count} servos, {dc_count} DC motors, {popup.result['duration']}ms")
            self.add_to_monitor(f"Added keyframe with {servo_count} servo positions and {dc_count} DC motor speeds")

    def move_keyframe(self, direction):
        """Move selected keyframe up or down"""
        selection = self.keyframe_listbox.curselection()
        if not selection:
            return
        
        index = selection[0]
        new_index = index + direction
        
        if 0 <= new_index < self.keyframe_listbox.size():
            # Move in sequence list
            frame = self.current_sequence.pop(index)
            self.current_sequence.insert(new_index, frame)
            
            # Move in listbox
            text = self.keyframe_listbox.get(index)
            self.keyframe_listbox.delete(index)
            self.keyframe_listbox.insert(new_index, text)
            self.keyframe_listbox.selection_set(new_index)

    def handle_movement(self, command):
        """Handle movement commands"""
        # Default speeds
        SLOW_SPEED = 50
        MEDIUM_SPEED = 100
        FAST_SPEED = 200
        
        # Initialize speeds for both motors
        left_speed = 0
        right_speed = 0
        
        # Handle different movement commands
        if command == 'forward':
            left_speed = right_speed = MEDIUM_SPEED
        elif command == 'forward_fast':
            left_speed = right_speed = FAST_SPEED
        elif command == 'forward_slow':
            left_speed = right_speed = SLOW_SPEED
        elif command == 'backward':
            left_speed = right_speed = -MEDIUM_SPEED
        elif command == 'backward_fast':
            left_speed = right_speed = -FAST_SPEED
        elif command == 'backward_slow':
            left_speed = right_speed = -SLOW_SPEED
        elif command == 'left':
            left_speed = -MEDIUM_SPEED
            right_speed = MEDIUM_SPEED
        elif command == 'right':
            left_speed = MEDIUM_SPEED
            right_speed = -MEDIUM_SPEED
        elif command == 'rotate_left':
            left_speed = -MEDIUM_SPEED
            right_speed = MEDIUM_SPEED
        elif command == 'rotate_right':
            left_speed = MEDIUM_SPEED
            right_speed = -MEDIUM_SPEED
        elif command == 'left_motor_forward':
            left_speed = MEDIUM_SPEED
        elif command == 'left_motor_backward':
            left_speed = -MEDIUM_SPEED
        elif command == 'right_motor_forward':
            right_speed = MEDIUM_SPEED
        elif command == 'right_motor_backward':
            right_speed = -MEDIUM_SPEED
        elif command == 'stop':
            left_speed = right_speed = 0
            
        # Send commands to motors
        self.send_zmq_command('update_motor', {'motor_id': 'LDC', 'value': left_speed})
        self.send_zmq_command('update_motor', {'motor_id': 'RDC', 'value': right_speed})
        
        # Log the movement
        if command != 'stop':
            self.add_to_monitor(f"Movement: {command} (L:{left_speed}, R:{right_speed})")

def main():
    root = tk.Tk()
    app = HexapodGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
