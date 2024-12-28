import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import time
import threading
import queue
import os
import zmq
import json
import numpy as np
from config_handler import ConfigHandler
from SingleLegSimulation import LegSimulation

class HexapodGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Hexapod Controller")
        
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://raspberrypi:5555")  # Replace with your RPI's IP
        
        # Message queue for communication
        self.message_queue = queue.Queue()
        
        # Initialize config handler
        self.config_handler = ConfigHandler()
        
        # Default values
        self.DEFAULT_LEG_LENGTHS = {
            'S1': 32.25,
            'S2': 44,
            'S3': 69.5
        }
        
        self.DEFAULT_LEG_TARGETS = {
            'left_front': {'x': 70, 'y': 20, 'z': -30},
            'left_center': {'x': 70, 'y': 0, 'z': -30},
            'left_back': {'x': 70, 'y': -20, 'z': -30},
            'right_front': {'x': 70, 'y': -20, 'z': -30},
            'right_center': {'x': 70, 'y': 0, 'z': -30},
            'right_back': {'x': 70, 'y': 20, 'z': -30}
        }
        
        self.DEFAULT_SERVO_OFFSETS = {
            'left_front': [0, 0, 0],
            'left_center': [0, 0, 0],
            'left_back': [0, 0, 0],
            'right_front': [0, 0, 0],
            'right_center': [0, 0, 0],
            'right_back': [0, 0, 0]
        }
        
        # Current values
        self.leg_lengths = self.DEFAULT_LEG_LENGTHS.copy()
        self.leg_targets = self.DEFAULT_LEG_TARGETS.copy()
        self.servo_offsets = self.DEFAULT_SERVO_OFFSETS.copy()
        
        # UI elements
        self.length_sliders = {}
        self.target_sliders = {}
        self.offset_sliders = {}
        self.leg_simulations = {}
        
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
        self.start_monitor()
    
    def setup_styles(self):
        """Configure ttk styles"""
        style = ttk.Style()
        style.configure('Header.TLabel', font=('Arial', 20, 'bold'), padding=10)
        style.configure('Motor.TLabel', font=('Arial', 11))
        style.configure('Group.TLabelframe.Label', font=('Arial', 12, 'bold'))
        style.configure('TLabelframe', padding=5)
    
    def send_zmq_command(self, command_type, data):
        """Send command to RPI via ZMQ"""
        try:
            message = {
                'type': command_type,
                'data': data
            }
            self.socket.send_json(message)
            response = self.socket.recv_json()
            if response['status'] == 'error':
                self.message_queue.put(f"Error: {response['message']}")
            else:
                self.message_queue.put(f"Success: {response['message']}")
            return response
        except Exception as e:
            error_msg = f"ZMQ communication error: {e}"
            self.message_queue.put(error_msg)
            return None
    
    def validate_input(self, P):
        """Validate input to only allow numbers and empty string"""
        if P == "": return True
        try:
            value = float(P)
            return -360 <= value <= 360
        except ValueError:
            return False
    
    def on_slider_change(self, value_var, slider, entry):
        """Called when a slider value changes."""
        value = slider.get()
        value_var.set(str(value))
        entry.delete(0, tk.END)
        entry.insert(0, str(value))
    
    def update_leg_lengths(self, leg_group=None):
        """Update leg lengths and send to RPI"""
        if leg_group:
            data = {
                'leg_group': leg_group,
                'lengths': [
                    self.length_sliders[f'{leg_group}_S1'].get(),
                    self.length_sliders[f'{leg_group}_S2'].get(),
                    self.length_sliders[f'{leg_group}_S3'].get()
                ]
            }
        else:
            data = {leg: [
                self.length_sliders[f'{leg}_S1'].get(),
                self.length_sliders[f'{leg}_S2'].get(),
                self.length_sliders[f'{leg}_S3'].get()
            ] for leg in self.motor_groups.keys()}
        
        self.send_zmq_command('update_lengths', data)
        self.update_simulations()
    
    def update_targets(self, leg_group=None):
        """Update leg targets and send to RPI"""
        if leg_group:
            data = {
                'leg_group': leg_group,
                'target': {
                    'x': self.target_sliders[f'{leg_group}_x'].get(),
                    'y': self.target_sliders[f'{leg_group}_y'].get(),
                    'z': self.target_sliders[f'{leg_group}_z'].get()
                }
            }
        else:
            data = {leg: {
                'x': self.target_sliders[f'{leg}_x'].get(),
                'y': self.target_sliders[f'{leg}_y'].get(),
                'z': self.target_sliders[f'{leg}_z'].get()
            } for leg in self.motor_groups.keys()}
        
        self.send_zmq_command('update_targets', data)
        self.update_simulations()
    
    def update_offsets(self, leg_group=None):
        """Update servo offsets and send to RPI"""
        if leg_group:
            data = {
                'leg_group': leg_group,
                'offsets': [
                    self.offset_sliders[f'{leg_group}_1'].get(),
                    self.offset_sliders[f'{leg_group}_2'].get(),
                    self.offset_sliders[f'{leg_group}_3'].get()
                ]
            }
        else:
            data = {leg: [
                self.offset_sliders[f'{leg}_1'].get(),
                self.offset_sliders[f'{leg}_2'].get(),
                self.offset_sliders[f'{leg}_3'].get()
            ] for leg in self.motor_groups.keys()}
        
        self.send_zmq_command('update_offsets', data)
        self.update_simulations()
    
    def create_leg_simulation(self, leg_group):
        """Create a simulation window for a specific leg"""
        sim = LegSimulation()
        sim.fig.canvas.manager.set_window_title(f"{leg_group.replace('_', ' ').title()} Simulation")
        self.leg_simulations[leg_group] = sim
        return sim
    
    def update_simulations(self):
        """Update all leg simulations with current values"""
        for leg_group, sim in self.leg_simulations.items():
            # Update leg lengths
            sim.L1 = self.length_sliders[f'{leg_group}_S1'].get()
            sim.L2 = self.length_sliders[f'{leg_group}_S2'].get()
            sim.L3 = self.length_sliders[f'{leg_group}_S3'].get()
            
            # Update target position
            target = np.array([
                self.target_sliders[f'{leg_group}_x'].get(),
                self.target_sliders[f'{leg_group}_y'].get(),
                self.target_sliders[f'{leg_group}_z'].get()
            ])
            
            # Calculate inverse kinematics
            angles = sim.inverse_kinematics(target)
            if angles is not None:
                # Add offsets
                angles = angles + np.array([
                    self.offset_sliders[f'{leg_group}_1'].get(),
                    self.offset_sliders[f'{leg_group}_2'].get(),
                    self.offset_sliders[f'{leg_group}_3'].get()
                ])
                
                # Update simulation
                sim.update(None)
    
    def create_parameter_sliders(self, parent_frame, leg_group):
        """Create sliders for all parameters of a leg group"""
        frame = ttk.LabelFrame(parent_frame, text=leg_group.replace('_', ' ').title())
        frame.pack(fill='x', padx=5, pady=5)
        
        # Validation command for entries
        vcmd = (self.root.register(self.validate_input), '%P')
        
        # Length sliders
        length_frame = ttk.LabelFrame(frame, text="Lengths")
        length_frame.pack(fill='x', padx=5, pady=5)
        
        for i, segment in enumerate(['S1', 'S2', 'S3']):
            ttk.Label(length_frame, text=f"Length {segment}").grid(row=i, column=0, padx=5, pady=2)
            
            # Create slider
            slider = ttk.Scale(length_frame, from_=0, to=100, orient='horizontal')
            slider.set(self.leg_lengths[segment])
            slider.grid(row=i, column=1, padx=5, pady=2, sticky='ew')
            self.length_sliders[f'{leg_group}_{segment}'] = slider
            
            # Create entry for direct value input
            value_var = tk.StringVar(value=str(slider.get()))
            entry = ttk.Entry(length_frame, textvariable=value_var, width=8, validate='key', validatecommand=vcmd)
            entry.grid(row=i, column=2, padx=5, pady=2)
            
            # Connect slider and entry
            slider.configure(command=lambda v, var=value_var, e=entry: self._on_slider_change(v, var, e))
            entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry: 
                      self._on_entry_change(e, s, var, ent))
        
        # Target sliders
        target_frame = ttk.LabelFrame(frame, text="Target Position")
        target_frame.pack(fill='x', padx=5, pady=5)
        
        for i, axis in enumerate(['x', 'y', 'z']):
            ttk.Label(target_frame, text=f"{axis.upper()} Position").grid(row=i, column=0, padx=5, pady=2)
            
            # Create slider
            slider = ttk.Scale(target_frame, from_=-200, to=200, orient='horizontal')
            slider.set(self.leg_targets[leg_group][axis])
            slider.grid(row=i, column=1, padx=5, pady=2, sticky='ew')
            self.target_sliders[f'{leg_group}_{axis}'] = slider
            
            # Create entry for direct value input
            value_var = tk.StringVar(value=str(slider.get()))
            entry = ttk.Entry(target_frame, textvariable=value_var, width=8, validate='key', validatecommand=vcmd)
            entry.grid(row=i, column=2, padx=5, pady=2)
            
            # Connect slider and entry
            slider.configure(command=lambda v, var=value_var, e=entry: self._on_slider_change(v, var, e))
            entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry: 
                      self._on_entry_change(e, s, var, ent))
        
        # Offset sliders
        offset_frame = ttk.LabelFrame(frame, text="Servo Offsets")
        offset_frame.pack(fill='x', padx=5, pady=5)
        
        for i in range(3):
            ttk.Label(offset_frame, text=f"Servo {i+1} Offset").grid(row=i, column=0, padx=5, pady=2)
            
            # Create slider
            slider = ttk.Scale(offset_frame, from_=-30, to=30, orient='horizontal')
            slider.set(self.servo_offsets[leg_group][i])
            slider.grid(row=i, column=1, padx=5, pady=2, sticky='ew')
            self.offset_sliders[f'{leg_group}_{i+1}'] = slider
            
            # Create entry for direct value input
            value_var = tk.StringVar(value=str(slider.get()))
            entry = ttk.Entry(offset_frame, textvariable=value_var, width=8, validate='key', validatecommand=vcmd)
            entry.grid(row=i, column=2, padx=5, pady=2)
            
            # Connect slider and entry
            slider.configure(command=lambda v, var=value_var, e=entry: self._on_slider_change(v, var, e))
            entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry: 
                      self._on_entry_change(e, s, var, ent))
        
        # Add update buttons
        button_frame = ttk.Frame(frame)
        button_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(button_frame, text="Update Lengths", 
                   command=lambda: self.update_leg_lengths(leg_group)).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Update Targets", 
                   command=lambda: self.update_targets(leg_group)).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Update Offsets", 
                   command=lambda: self.update_offsets(leg_group)).pack(side='left', padx=5)
        
        return frame
    
    def _on_slider_change(self, value, value_var, entry):
        """Handle slider value changes"""
        try:
            value = float(value)
            value_var.set(f"{value:.1f}")
            entry.delete(0, tk.END)
            entry.insert(0, f"{value:.1f}")
        except ValueError:
            pass
    
    def _on_entry_change(self, event, slider, value_var, entry):
        """Handle entry value changes"""
        try:
            value = float(entry.get())
            if slider['from'] <= value <= slider['to']:
                slider.set(value)
                value_var.set(f"{value:.1f}")
            else:
                entry.delete(0, tk.END)
                entry.insert(0, value_var.get())
        except ValueError:
            entry.delete(0, tk.END)
            entry.insert(0, value_var.get())
    
    def save_current_values(self):
        """Save current configuration to file"""
        try:
            config = {
                'leg_lengths': {leg_group: {
                    'S1': self.length_sliders[f'{leg_group}_S1'].get(),
                    'S2': self.length_sliders[f'{leg_group}_S2'].get(),
                    'S3': self.length_sliders[f'{leg_group}_S3'].get()
                } for leg_group in self.motor_groups.keys()},
                'targets': {leg_group: {
                    'x': self.target_sliders[f'{leg_group}_x'].get(),
                    'y': self.target_sliders[f'{leg_group}_y'].get(),
                    'z': self.target_sliders[f'{leg_group}_z'].get()
                } for leg_group in self.motor_groups.keys()},
                'offsets': {leg_group: [
                    self.offset_sliders[f'{leg_group}_1'].get(),
                    self.offset_sliders[f'{leg_group}_2'].get(),
                    self.offset_sliders[f'{leg_group}_3'].get()
                ] for leg_group in self.motor_groups.keys()}
            }
            
            filename = filedialog.asksaveasfilename(
                initialdir=self.config_handler.config_dir,
                title="Save Configuration",
                defaultextension=".json",
                filetypes=[("JSON files", "*.json")]
            )
            
            if filename:
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=4)
                self.message_queue.put(f"Configuration saved to {filename}")
        
        except Exception as e:
            self.message_queue.put(f"Error saving configuration: {e}")
            messagebox.showerror("Error", f"Failed to save configuration: {e}")
    
    def load_configuration(self):
        """Load configuration from file"""
        try:
            filename = filedialog.askopenfilename(
                initialdir=self.config_handler.config_dir,
                title="Load Configuration",
                filetypes=[("JSON files", "*.json")]
            )
            
            if filename:
                with open(filename, 'r') as f:
                    config = json.load(f)
                
                # Update leg lengths
                for leg_group, lengths in config['leg_lengths'].items():
                    for segment, value in lengths.items():
                        self.length_sliders[f'{leg_group}_{segment}'].set(value)
                
                # Update targets
                for leg_group, target in config['targets'].items():
                    for axis, value in target.items():
                        self.target_sliders[f'{leg_group}_{axis}'].set(value)
                
                # Update offsets
                for leg_group, offsets in config['offsets'].items():
                    for i, value in enumerate(offsets):
                        self.offset_sliders[f'{leg_group}_{i+1}'].set(value)
                
                # Update all legs
                self.update_leg_lengths()
                self.update_targets()
                self.update_offsets()
                
                self.message_queue.put(f"Configuration loaded from {filename}")
        
        except Exception as e:
            self.message_queue.put(f"Error loading configuration: {e}")
            messagebox.showerror("Error", f"Failed to load configuration: {e}")
    
    def create_gui(self):
        """Create the main GUI layout"""
        # Create main container with scrollbar
        main_container = ttk.Frame(self.root)
        main_container.pack(fill='both', expand=True)
        
        # Create canvas with scrollbar
        canvas = tk.Canvas(main_container)
        scrollbar = ttk.Scrollbar(main_container, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Pack the scrollbar and canvas
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        
        # Header
        header = ttk.Label(scrollable_frame, text="Hexapod Controller", style='Header.TLabel')
        header.pack(fill='x', pady=10)
        
        # Create frames for left and right sides
        left_frame = ttk.Frame(scrollable_frame)
        left_frame.pack(side='left', fill='both', padx=10, pady=5)
        
        right_frame = ttk.Frame(scrollable_frame)
        right_frame.pack(side='left', fill='both', padx=10, pady=5)
        
        # Create parameter controls for each leg group
        left_legs = ['left_front', 'left_center', 'left_back']
        right_legs = ['right_front', 'right_center', 'right_back']
        
        for leg_group in left_legs:
            self.create_parameter_sliders(left_frame, leg_group)
            self.create_leg_simulation(leg_group)
        
        for leg_group in right_legs:
            self.create_parameter_sliders(right_frame, leg_group)
            self.create_leg_simulation(leg_group)
        
        # Create global control buttons
        control_frame = ttk.LabelFrame(scrollable_frame, text="Global Controls")
        control_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Button(control_frame, text="Update All Lengths", 
                   command=lambda: self.update_leg_lengths()).pack(side='left', padx=5)
        ttk.Button(control_frame, text="Update All Targets", 
                   command=lambda: self.update_targets()).pack(side='left', padx=5)
        ttk.Button(control_frame, text="Update All Offsets", 
                   command=lambda: self.update_offsets()).pack(side='left', padx=5)
        
        # Add save/load configuration buttons
        config_frame = ttk.Frame(scrollable_frame)
        config_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Button(config_frame, text="Save Configuration", 
                   command=self.save_current_values).pack(side='left', padx=5)
        ttk.Button(config_frame, text="Load Configuration", 
                   command=self.load_configuration).pack(side='left', padx=5)
        
        # Communication monitor
        monitor_frame = ttk.LabelFrame(scrollable_frame, text="Communication Monitor")
        monitor_frame.pack(fill='x', padx=10, pady=5)
        
        self.text_widget = tk.Text(monitor_frame, height=6, width=80, font=("Consolas", 10))
        self.text_widget.pack(side='left', fill='both', expand=True)
        
        monitor_scrollbar = ttk.Scrollbar(monitor_frame, orient="vertical", 
                                        command=self.text_widget.yview)
        monitor_scrollbar.pack(side='right', fill='y')
        self.text_widget.configure(yscrollcommand=monitor_scrollbar.set)
    
    def update_monitor(self):
        """Update the monitor text widget with messages from the queue"""
        while not self.message_queue.empty():
            message = self.message_queue.get()
            self.text_widget.insert(tk.END, message + '\n')
            self.text_widget.see(tk.END)
        self.root.after(50, self.update_monitor)
    
    def start_monitor(self):
        """Start the monitor update loop"""
        self.update_monitor()
    
    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, 'socket'):
            self.socket.close()
        if hasattr(self, 'context'):
            self.context.term()

def main():
    root = tk.Tk()
    app = HexapodGUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.cleanup(), root.destroy()))
    root.mainloop()

if __name__ == "__main__":
    main()
