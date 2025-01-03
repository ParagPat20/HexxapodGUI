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
                
                # Add update button
                ttk.Button(motor_frame, text="Update",
                          command=lambda m=motor_id, s=slider: self.update_servo(m, s.get())).pack(side='left', padx=5)
                
                # Connect slider and entry
                slider.configure(command=lambda v, var=value_var, e=entry, m=motor_id: 
                               self._on_servo_slider_change(v, var, e, m))
                slider.bind('<ButtonRelease-1>', lambda e, m=motor_id, s=slider: 
                          self.update_servo(m, float(s.get())))
                entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry, m=motor_id: 
                          self._on_servo_entry_change(e, s, var, ent, m))
    
    def _on_servo_slider_change(self, value, value_var, entry, motor_id):
        """Handle slider value changes for servo control"""
        try:
            value = float(value)
            value_var.set(f"{value:.1f}")
            entry.delete(0, tk.END)
            entry.insert(0, f"{value:.1f}")
        except ValueError:
            pass
    
    def _on_servo_entry_change(self, event, slider, value_var, entry, motor_id):
        """Handle entry value changes for servo control"""
        try:
            value = float(entry.get())
            value = max(0, min(180, value))  # Clamp value
            slider.set(value)
            value_var.set(f"{value:.1f}")
            
            # Update the servo
            self.update_servo(motor_id, value)
        except ValueError:
            pass
    
    def update_servo(self, motor_id, value):
        """Send servo update command and save to config"""
        self.command_queue.put(('update_motor', {
            'motor_id': motor_id,
            'value': int(value)
        }))
        # Update config
        self.config_handler.update_motor_value(motor_id, int(value))

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
                    print(f"Response: {message}")
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
        
        # Add walking control button
        walk_button = ttk.Button(control_frame, text="Start Walking", command=self.toggle_walking)
        walk_button.pack(pady=5)
        self.is_walking = False
        self.walk_button = walk_button
        
        ttk.Label(control_frame, text="Keyboard Controls:", style='Header.TLabel').pack(pady=5)
        ttk.Label(control_frame, text="W: Forward\nS: Backward\nA: Turn Left\nD: Turn Right\nSpace: Stop",
                 justify='left').pack(padx=10, pady=5)

    def toggle_walking(self):
        """Toggle walking sequence"""
        self.is_walking = not self.is_walking
        if self.is_walking:
            self.walk_button.configure(text="Stop Walking")
            self.walking_thread = threading.Thread(target=self.walking_sequence, daemon=True)
            self.walking_thread.start()
        else:
            self.walk_button.configure(text="Start Walking")

    def walking_sequence(self):
        """Execute walking sequence"""
        # Define leg groups for tripod gait
        tripod_1 = {
            'L1': 90, 'L8': 90, 'L12': 90,  # Left legs (front, center, back)
            'R14': 90, 'R8': 90, 'R3': 90   # Right legs (front, center, back)
        }
        tripod_2 = {
            'L2': 90, 'L6': 90, 'L10': 90,  # Left legs (front, center, back)
            'R15': 90, 'R10': 90, 'R2': 90   # Right legs (front, center, back)
        }
        
        while self.is_walking:
            try:
                # Lift tripod 1
                for motor_id, value in tripod_1.items():
                    self.command_queue.put(('update_motor', {
                        'motor_id': motor_id,
                        'value': value + 30  # Lift legs
                    }))
                time.sleep(0.3)
                
                # Move tripod 1 forward and tripod 2 backward
                for motor_id, value in tripod_2.items():
                    self.command_queue.put(('update_motor', {
                        'motor_id': motor_id,
                        'value': value - 20  # Move backward
                    }))
                time.sleep(0.3)
                
                # Lower tripod 1
                for motor_id, value in tripod_1.items():
                    self.command_queue.put(('update_motor', {
                        'motor_id': motor_id,
                        'value': value  # Return to normal position
                    }))
                time.sleep(0.3)
                
                # Lift tripod 2
                for motor_id, value in tripod_2.items():
                    self.command_queue.put(('update_motor', {
                        'motor_id': motor_id,
                        'value': value + 30  # Lift legs
                    }))
                time.sleep(0.3)
                
                # Move tripod 2 forward and tripod 1 backward
                for motor_id, value in tripod_1.items():
                    self.command_queue.put(('update_motor', {
                        'motor_id': motor_id,
                        'value': value - 20  # Move backward
                    }))
                time.sleep(0.3)
                
                # Lower tripod 2
                for motor_id, value in tripod_2.items():
                    self.command_queue.put(('update_motor', {
                        'motor_id': motor_id,
                        'value': value  # Return to normal position
                    }))
                time.sleep(0.3)
                
            except Exception as e:
                self.message_queue.put(f"Walking sequence error: {e}")
                self.is_walking = False
                self.walk_button.configure(text="Start Walking")
                break

def main():
    root = tk.Tk()
    app = HexapodGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
