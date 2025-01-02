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
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D

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
        
        # Simulation variables
        self.leg_count = 6
        self.parameter_count = 3
        self.kartesian = np.zeros(6)
        self.kartesianX = np.zeros(self.leg_count)
        self.kartesianY = np.zeros(self.leg_count)
        self.kartesianZ = np.zeros(self.leg_count)
        self.kartesianAngles = np.zeros(self.leg_count * self.parameter_count)
        self.kartesian_ref = np.zeros((self.leg_count, 3))
        self.all_angles = np.zeros((self.leg_count, self.parameter_count))
        self.defaultLegTransform = np.array([35.74, 56.0, -40.0, 0.0, 0.0, 0.0])
        
        # Walking variables
        self.walking_direction = 'none'
        self.turn_direction = 'none'
        self.is_walking = False
        self.walking_mode = 1
        self.walking_phases = np.zeros(self.leg_count)
        self.last_timestamp = time.time()
        
        # Initialize reference positions
        for i in range(self.leg_count):
            self.kartesian_ref[i] = [self.defaultLegTransform[0], self.defaultLegTransform[1], self.defaultLegTransform[2]]
        
        # Simulation plot elements
        self.ref_lines = [[] for i in range(0,7,1)]
        self.quivers = [0 for i in range(0,25,1)]
        self.lines = [[] for i in range(0,24,1)]
        self.quiv_colors = ['r','g','b']
        self.x_lim = [-200,200]
        self.y_lim = [-200,200]
        self.z_lim = [-200,200]
        
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
            return 0 <= value <= 180  # Allow values between 0 and 180
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
        if not hasattr(self, 'simulation_window'):
            # Create a new top-level window for simulations
            self.simulation_window = tk.Toplevel(self.root)
            self.simulation_window.title("Hexapod Simulation")
            self.simulation_window.geometry("800x800")  # Adjust size as needed
            
            # Create a single figure with one 3D plot
            self.simulation_figure = plt.figure(figsize=(10, 10))
            self.ax = self.simulation_figure.add_subplot(111, projection='3d')
            self.ax.set_title("Hexapod Leg Positions")
            
            # Set axis limits and labels
            limit = 200  # Adjust based on your needs
            self.ax.set_xlim([-limit, limit])
            self.ax.set_ylim([-limit, limit])
            self.ax.set_zlim([-limit, limit])
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
            
            # Set initial view angle for better visualization
            self.ax.view_init(elev=30, azim=45)
            
            # Create a canvas to embed the matplotlib figure
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            self.canvas = FigureCanvasTkAgg(self.simulation_figure, master=self.simulation_window)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
            
            # Add navigation toolbar
            from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
            toolbar = NavigationToolbar2Tk(self.canvas, self.simulation_window)
            toolbar.update()
            self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
            
            # Add legend frame
            legend_frame = ttk.LabelFrame(self.simulation_window, text="Leg Angles")
            legend_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
            
            # Create text widgets for each leg's angles
            self.angle_texts = {}
            for i, leg in enumerate(self.motor_groups.keys()):
                label = ttk.Label(legend_frame, text=f"{leg.replace('_', ' ').title()}:",
                                font=("Consolas", 8, "bold"))
                label.grid(row=i//3, column=(i%3)*2, padx=5, pady=2, sticky='e')
                
                text = ttk.Label(legend_frame, text="θ1: 0.0°\nθ2: 0.0°\nθ3: 0.0°",
                               font=("Consolas", 8))
                text.grid(row=i//3, column=(i%3)*2+1, padx=5, pady=2, sticky='w')
                self.angle_texts[leg] = text
        
        # Create simulation object for this leg
        sim = LegSimulation()
        sim.fig = self.simulation_figure
        sim.ax = self.ax
        
        # Set initial leg lengths
        sim.L1 = self.length_sliders[f'{leg_group}_S1'].get()
        sim.L2 = self.length_sliders[f'{leg_group}_S2'].get()
        sim.L3 = self.length_sliders[f'{leg_group}_S3'].get()
        
        # Initialize the plot with a unique color for each leg
        colors = {
            'left_front': '#FF0000',    # Red
            'left_center': '#0000FF',   # Blue
            'left_back': '#00FF00',     # Green
            'right_front': '#FFA500',   # Orange
            'right_center': '#800080',  # Purple
            'right_back': '#A52A2A'     # Brown
        }
        
        # Initialize the plot with proper matplotlib formatting
        sim.leg_line, = self.ax.plot([], [], [], '-', color=colors[leg_group], linewidth=2,
                                    label=leg_group.replace('_', ' ').title())
        sim.target_point, = self.ax.plot([], [], [], 'o', color=colors[leg_group])
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Store the simulation object
        self.leg_simulations[leg_group] = sim
        
        # Update the simulation immediately
        self.update_simulations()
        
        return sim
    
    def update_simulations(self):
        """Update all leg simulations with current values"""
        if not hasattr(self, 'simulation_window'):
            return
            
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
                
                # Calculate joint positions
                points = sim.forward_kinematics(angles)
                
                # Update the visualization
                sim.leg_line.set_data_3d(points[:,0], points[:,1], points[:,2])
                sim.target_point.set_data_3d([target[0]], [target[1]], [target[2]])
                
                # Update angle text
                if leg_group in self.angle_texts:
                    angle_text = f'θ1: {math.degrees(angles[0]):.1f}°\n' \
                               f'θ2: {math.degrees(angles[1]):.1f}°\n' \
                               f'θ3: {math.degrees(angles[2]):.1f}°'
                    self.angle_texts[leg_group].configure(text=angle_text)
        
        # Redraw the canvas
        if hasattr(self, 'canvas'):
            self.canvas.draw()
    
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
            
            # Connect slider and entry with lambda to capture current values
            slider.configure(command=lambda v, var=value_var, e=entry, s=slider: 
                           self._on_slider_change(s.get(), var, e))
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
            
            # Connect slider and entry with lambda to capture current values
            slider.configure(command=lambda v, var=value_var, e=entry, s=slider: 
                           self._on_slider_change(s.get(), var, e))
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
            
            # Connect slider and entry with lambda to capture current values
            slider.configure(command=lambda v, var=value_var, e=entry, s=slider: 
                           self._on_slider_change(s.get(), var, e))
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
            # Update simulation after slider change
            self.update_simulations()
        except ValueError:
            pass
    
    def _on_entry_change(self, event, slider, value_var, entry):
        """Handle entry value changes"""
        try:
            value = float(entry.get())
            if slider['from'] <= value <= slider['to']:
                slider.set(value)
                value_var.set(f"{value:.1f}")
                # Update simulation after entry change
                self.update_simulations()
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
        
        # Add motor control menu button
        ttk.Button(scrollable_frame, text="Motor Control", 
                   command=self.create_motor_control_menu).pack(fill='x', padx=10, pady=5)
        
        # Add simulation button
        ttk.Button(scrollable_frame, text="Open Simulation", 
                   command=self.create_simulation_window).pack(fill='x', padx=10, pady=5)
    
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
        # Signal communication thread to exit
        self.command_queue.put(None)
        
        # Wait for threads to finish
        if hasattr(self, 'comm_thread'):
            self.comm_thread.join(timeout=1.0)
        if hasattr(self, 'response_thread'):
            self.response_thread.join(timeout=1.0)
            
        if hasattr(self, 'socket'):
            self.socket.close()
        if hasattr(self, 'context'):
            self.context.term()
    
    def create_motor_control_menu(self):
        """Create a menu for direct motor control using sliders and value inputs"""
        motor_control_window = tk.Toplevel(self.root)
        motor_control_window.title("Motor Control")
        motor_control_window.geometry("800x800")  # Increased height for DC motors
        
        # Add toggle button for continuous update
        self.continuous_update = tk.BooleanVar(value=False)
        toggle_frame = ttk.Frame(motor_control_window)
        toggle_frame.pack(fill='x', padx=5, pady=5)
        ttk.Label(toggle_frame, text="Update Mode:").pack(side='left', padx=5)
        self.toggle_button = ttk.Checkbutton(toggle_frame, text="Continuous Update", 
                                           variable=self.continuous_update)
        self.toggle_button.pack(side='left', padx=5)
        
        # Create notebook for tabs
        notebook = ttk.Notebook(motor_control_window)
        notebook.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Servo Motors Tab
        servo_frame = ttk.Frame(notebook)
        notebook.add(servo_frame, text="Servo Motors")
        
        # Create a frame for each motor group in servo tab
        for leg_group, motors in self.motor_groups.items():
            frame = ttk.LabelFrame(servo_frame, text=leg_group.replace('_', ' ').title())
            frame.pack(fill='x', padx=5, pady=5)
            
            # Configure column weights to make slider column expandable
            frame.columnconfigure(1, weight=3)  # Make slider column take more space
            
            for i, (motor_id, motor_name) in enumerate(motors.items()):
                ttk.Label(frame, text=motor_name, width=15).grid(row=i, column=0, padx=5, pady=2, sticky='w')
                
                # Create slider with range 0-180
                slider = ttk.Scale(frame, from_=0, to=180, orient='horizontal', length=500)
                slider.set(90)  # Set default position to middle
                slider.grid(row=i, column=1, padx=5, pady=2, sticky='ew')
                
                # Create entry for direct value input
                value_var = tk.StringVar(value="90")
                entry = ttk.Entry(frame, textvariable=value_var, width=8)
                entry.grid(row=i, column=2, padx=5, pady=2)
                
                # Connect slider and entry with continuous update check
                slider.configure(command=lambda v, var=value_var, e=entry, s=slider, m_id=motor_id: 
                               self._on_motor_slider_change(s.get(), var, e, m_id))
                entry.bind('<Return>', lambda e, s=slider, var=value_var, ent=entry: 
                          self._on_entry_change(e, s, var, ent))
                
                # Add update button
                ttk.Button(frame, text="Update", 
                           command=lambda m_id=motor_id, s=slider: self.update_motor(m_id, s.get())).grid(row=i, column=3, padx=5)
        
        # DC Motors Tab
        dc_frame = ttk.Frame(notebook)
        notebook.add(dc_frame, text="DC Motors")
        
        # Create frame for DC motors
        dc_motors_frame = ttk.LabelFrame(dc_frame, text="DC Motors Control")
        dc_motors_frame.pack(fill='x', padx=5, pady=5)
        
        # Left DC Motor
        left_dc_frame = ttk.Frame(dc_motors_frame)
        left_dc_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(left_dc_frame, text="Left DC Motor", width=15).pack(side='left', padx=5)
        left_dc_slider = ttk.Scale(left_dc_frame, from_=-255, to=255, orient='horizontal', length=500)
        left_dc_slider.set(0)
        left_dc_slider.pack(side='left', fill='x', expand=True, padx=5)
        
        left_dc_var = tk.StringVar(value="0")
        left_dc_entry = ttk.Entry(left_dc_frame, textvariable=left_dc_var, width=8)
        left_dc_entry.pack(side='left', padx=5)
        
        ttk.Button(left_dc_frame, text="Update", 
                   command=lambda: self.update_dc_motor("LDC", left_dc_slider.get())).pack(side='left', padx=5)
        
        # Right DC Motor
        right_dc_frame = ttk.Frame(dc_motors_frame)
        right_dc_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(right_dc_frame, text="Right DC Motor", width=15).pack(side='left', padx=5)
        right_dc_slider = ttk.Scale(right_dc_frame, from_=-255, to=255, orient='horizontal', length=500)
        right_dc_slider.set(0)
        right_dc_slider.pack(side='left', fill='x', expand=True, padx=5)
        
        right_dc_var = tk.StringVar(value="0")
        right_dc_entry = ttk.Entry(right_dc_frame, textvariable=right_dc_var, width=8)
        right_dc_entry.pack(side='left', padx=5)
        
        ttk.Button(right_dc_frame, text="Update", 
                   command=lambda: self.update_dc_motor("RDC", right_dc_slider.get())).pack(side='left', padx=5)
        
        # Connect DC motor sliders and entries
        left_dc_slider.configure(command=lambda v: self._on_dc_slider_change(v, left_dc_var, left_dc_entry, "LDC"))
        right_dc_slider.configure(command=lambda v: self._on_dc_slider_change(v, right_dc_var, right_dc_entry, "RDC"))
        
        left_dc_entry.bind('<Return>', lambda e, s=left_dc_slider, var=left_dc_var, ent=left_dc_entry, m_id="LDC": 
                          self._on_dc_entry_change(e, s, var, ent, m_id))
        right_dc_entry.bind('<Return>', lambda e, s=right_dc_slider, var=right_dc_var, ent=right_dc_entry, m_id="RDC": 
                          self._on_dc_entry_change(e, s, var, ent, m_id))

    def _on_dc_slider_change(self, value, value_var, entry, motor_id):
        """Handle DC motor slider value changes"""
        try:
            value = float(value)
            value_var.set(f"{value:.0f}")
            entry.delete(0, tk.END)
            entry.insert(0, f"{value:.0f}")
            
            # If continuous update is enabled, update the motor immediately
            if self.continuous_update.get():
                self.update_dc_motor(motor_id, value)
        except ValueError:
            pass

    def _on_dc_entry_change(self, event, slider, value_var, entry, motor_id):
        """Handle DC motor entry value changes"""
        try:
            value = float(entry.get())
            if slider['from'] <= value <= slider['to']:
                slider.set(value)
                value_var.set(f"{value:.0f}")
                # Update motor if continuous update is enabled
                if self.continuous_update.get():
                    self.update_dc_motor(motor_id, value)
            else:
                entry.delete(0, tk.END)
                entry.insert(0, value_var.get())
        except ValueError:
            entry.delete(0, tk.END)
            entry.insert(0, value_var.get())

    def update_dc_motor(self, motor_id, value):
        """Update DC motor speed"""
        try:
            self.command_queue.put(('update_motor', {'motor_id': motor_id, 'value': int(value)}))
            print(f"Queued update for DC motor {motor_id} to speed {value}")
        except Exception as e:
            print(f"Error queueing DC motor update {motor_id}: {e}")

    def communication_handler(self):
        """Handle sending commands in a separate thread"""
        while True:
            try:
                command = self.command_queue.get()
                if command is None:  # Exit signal
                    break
                    
                command_type, data = command
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
                    
            except Exception as e:
                self.message_queue.put(f"Communication error: {e}")
            finally:
                self.command_queue.task_done()
    
    def response_handler(self):
        """Handle receiving responses in a separate thread"""
        while True:
            try:
                # Update GUI with messages from the queue
                while not self.message_queue.empty():
                    message = self.message_queue.get_nowait()
                    self.text_widget.insert(tk.END, message + '\n')
                    self.text_widget.see(tk.END)
                    self.message_queue.task_done()
            except queue.Empty:
                pass
            except Exception as e:
                print(f"Response handler error: {e}")
            
            # Sleep briefly to prevent high CPU usage
            time.sleep(0.05)

    def dh_transform(self, theta, s, a, alpha):
        """Generate transform from D&H parameters"""
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)
        return np.array([[ct, -st*ca, st*sa, a*ct],
                        [st, ct*ca, -ct*sa, a*st],
                        [0, sa, ca, s],
                        [0, 0, 0, 1]])

    def update_transforms(self):
        """Update transforms for all legs"""
        # transform constants based on geometry
        x_r1 = 35.74+20.15
        x_r2 = 37.5+28.5
        y_r1 = 56+4.24+20.15
        y_r1a = y_r1 - x_r1
        x_r3 = np.sqrt(x_r1**2+x_r1**2)
        z_r = 12.6
        l2 = 32.25
        l3 = 44
        l4 = 69.5
        a = self.kartesian[3]
        b = self.kartesian[4]
        g = self.kartesian[5]
        
        # loop over all legs
        for i in range(len(self.leg_transforms)):
            # origin to robot
            self.leg_transforms[i][0] = np.array([[np.cos(g)*np.cos(b),-np.sin(g)*np.cos(a)+np.cos(g)*np.sin(b)*np.sin(a),
                                                 np.sin(g)*np.sin(a)+np.cos(g)*np.sin(b)*np.cos(a),self.kartesian[0]],
                                                [np.sin(g)*np.cos(b),np.cos(g)*np.cos(a)+np.sin(g)*np.sin(b)*np.sin(a),
                                                 -np.cos(g)*np.sin(a)+np.sin(g)*np.sin(b)*np.cos(a),self.kartesian[1]],
                                                [-np.sin(b),np.cos(b)*np.sin(a),np.cos(b)*np.cos(a),self.kartesian[2]],
                                                [0,0,0,1]])
            
            # robot to joint 1
            if i == 0:
                t1 = self.dh_transform(np.radians(90),z_r,y_r1a,0)
                t2 = self.dh_transform(np.radians(45),0,x_r3,0)
                self.leg_transforms[i][1] = np.dot(t1,t2)
            elif i == 1:
                t1 = self.dh_transform(np.radians(90),z_r,y_r1a,0)
                t2 = self.dh_transform(-np.radians(45),0,x_r3,0)
                self.leg_transforms[i][1] = np.dot(t1,t2)
            elif i == 2:
                self.leg_transforms[i][1] = self.dh_transform(np.radians(180),z_r,x_r2,0)
            elif i == 3:
                self.leg_transforms[i][1] = self.dh_transform(0,z_r,x_r2,0)
            elif i == 4:
                t1 = self.dh_transform(-np.radians(90),z_r,y_r1a,0)
                t2 = self.dh_transform(-np.radians(45),0,x_r3,0)
                self.leg_transforms[i][1] = np.dot(t1,t2)
            elif i == 5:
                t1 = self.dh_transform(-np.radians(90),z_r,y_r1a,0)
                t2 = self.dh_transform(np.radians(45),0,x_r3,0)
                self.leg_transforms[i][1] = np.dot(t1,t2)
            
            # split by side of robot
            if i%2 == 0:
                # joint 1 to joint 2
                self.leg_transforms[i][2] = self.dh_transform(-self.all_angles[i][0],0,l2,np.pi/2)
                # joint 2 to joint 3
                self.leg_transforms[i][3] = self.dh_transform(self.all_angles[i][1],0,l3,0)
                # joint 3 to end
                self.leg_transforms[i][4] = self.dh_transform(-self.all_angles[i][2],0,l4,-np.pi/2)
            else:
                # joint 1 to joint 2
                self.leg_transforms[i][2] = self.dh_transform(self.all_angles[i][0],0,l2,-np.pi/2)
                # joint 2 to joint 3
                self.leg_transforms[i][3] = self.dh_transform(-self.all_angles[i][1],0,l3,0)
                # joint 3 to end
                self.leg_transforms[i][4] = self.dh_transform(self.all_angles[i][2],0,l4,np.pi/2)
            
            # determine sum_transforms
            self.sum_transforms[i][0] = np.dot(self.leg_transforms[i][0],self.leg_transforms[i][1])
            # loop over rest of sum_transforms
            for j in range(1,len(self.sum_transforms[i])):
                self.sum_transforms[i][j] = np.dot(self.sum_transforms[i][j-1],self.leg_transforms[i][j+1])

    def initial_draw(self):
        """Initialize the simulation visualization"""
        if self.mode:
            self.inverse_kinematics_4()
        self.update_transforms()
        
        # Draw coordinate system origin
        quiv_set = []
        for i in range(3):
            quiv_set.append(self.ax.quiver(self.leg_transforms[0][0][0,-1],
                                         self.leg_transforms[0][0][1,-1],
                                         self.leg_transforms[0][0][2,-1],
                                         self.leg_transforms[0][0][i,0:3][0],
                                         self.leg_transforms[0][0][i,0:3][1],
                                         self.leg_transforms[0][0][i,0:3][2],
                                         color=self.quiv_colors[i],length=25,normalize=True))
        self.quivers[0] = quiv_set
        
        # Draw legs
        colors = ["black","blue","black","blue"]
        for i in range(len(self.sum_transforms)):
            # First line
            self.lines[i*len(self.sum_transforms[i])] = self.ax.plot([0,self.sum_transforms[i][0][0,-1]],
                                                                    [0,self.sum_transforms[i][0][1,-1]],
                                                                    [0,self.sum_transforms[i][0][2,-1]],
                                                                    color=colors[0])[0]
            
            # Draw coordinate systems
            quiv_set = []
            for k in range(3):
                quiv_set.append(self.ax.quiver(self.sum_transforms[i][0][0,-1],
                                             self.sum_transforms[i][0][1,-1],
                                             self.sum_transforms[i][0][2,-1],
                                             self.sum_transforms[i][0][0:3,k][0],
                                             self.sum_transforms[i][0][0:3,k][1],
                                             self.sum_transforms[i][0][0:3,k][2],
                                             color=self.quiv_colors[k],length=25,normalize=True))
            self.quivers[i*len(self.sum_transforms[i])+1] = quiv_set
            
            # Rest of the transforms
            for j in range(1,len(self.sum_transforms[i])):
                self.lines[i*len(self.sum_transforms[i])+j].set_data_3d([self.sum_transforms[i][j-1][0,-1],
                                                                          self.sum_transforms[i][j][0,-1]],
                                                                         [self.sum_transforms[i][j-1][1,-1],
                                                                          self.sum_transforms[i][j][1,-1]],
                                                                         [self.sum_transforms[i][j-1][2,-1],
                                                                          self.sum_transforms[i][j][2,-1]])
                
                quiv_set = []
                for k in range(3):
                    quiv_set.append(self.ax.quiver(self.sum_transforms[i][j][0,-1],
                                                 self.sum_transforms[i][j][1,-1],
                                                 self.sum_transforms[i][j][2,-1],
                                                 self.sum_transforms[i][j][0:3,k][0],
                                                 self.sum_transforms[i][j][0:3,k][1],
                                                 self.sum_transforms[i][j][0:3,k][2],
                                                 color=self.quiv_colors[k],length=25,normalize=True))
                self.quivers[i*len(self.sum_transforms[i])+j+1] = quiv_set
        
        self.canvas.draw()
        self.root.after(50, self.update_simulation)

    def update_simulation(self):
        """Update the simulation visualization"""
        if self.mode:
            if self.walking_mode == 1:
                self.predefine_positions()
                self.walking()
            self.inverse_kinematics_4()
        
        self.update_transforms()
        
        # Update coordinate systems
        for i in range(len(self.quivers)):
            for j in range(len(self.quivers[i])):
                self.quivers[i][j].remove()
        
        # Update origin coordinate system
        quiv_set = []
        for i in range(3):
            quiv_set.append(self.ax.quiver(self.leg_transforms[0][0][0,-1],
                                         self.leg_transforms[0][0][1,-1],
                                         self.leg_transforms[0][0][2,-1],
                                         self.leg_transforms[0][0][i,0:3][0],
                                         self.leg_transforms[0][0][i,0:3][1],
                                         self.leg_transforms[0][0][i,0:3][2],
                                         color=self.quiv_colors[i],length=25,normalize=True))
        self.quivers[0] = quiv_set
        
        # Update legs
        for i in range(len(self.sum_transforms)):
            self.lines[i*len(self.sum_transforms[i])].set_data_3d([self.kartesian[0],self.sum_transforms[i][0][0,-1]],
                                                                 [self.kartesian[1],self.sum_transforms[i][0][1,-1]],
                                                                 [self.kartesian[2],self.sum_transforms[i][0][2,-1]])
            
            quiv_set = []
            for k in range(3):
                quiv_set.append(self.ax.quiver(self.sum_transforms[i][0][0,-1],
                                             self.sum_transforms[i][0][1,-1],
                                             self.sum_transforms[i][0][2,-1],
                                             self.sum_transforms[i][0][0:3,k][0],
                                             self.sum_transforms[i][0][0:3,k][1],
                                             self.sum_transforms[i][0][0:3,k][2],
                                             color=self.quiv_colors[k],length=25,normalize=True))
            self.quivers[i*len(self.sum_transforms[i])+1] = quiv_set
            
            for j in range(1,len(self.sum_transforms[i])):
                self.lines[i*len(self.sum_transforms[i])+j].set_data_3d([self.sum_transforms[i][j-1][0,-1],
                                                                       self.sum_transforms[i][j][0,-1]],
                                                                      [self.sum_transforms[i][j-1][1,-1],
                                                                       self.sum_transforms[i][j][1,-1]],
                                                                      [self.sum_transforms[i][j-1][2,-1],
                                                                       self.sum_transforms[i][j][2,-1]])
                
                quiv_set = []
                for k in range(3):
                    quiv_set.append(self.ax.quiver(self.sum_transforms[i][j][0,-1],
                                                 self.sum_transforms[i][j][1,-1],
                                                 self.sum_transforms[i][j][2,-1],
                                                 self.sum_transforms[i][j][0:3,k][0],
                                                 self.sum_transforms[i][j][0:3,k][1],
                                                 self.sum_transforms[i][j][0:3,k][2],
                                                 color=self.quiv_colors[k],length=25,normalize=True))
                self.quivers[i*len(self.sum_transforms[i])+j+1] = quiv_set
        
        self.canvas.draw()
        self.root.after(50, self.update_simulation)

    def predefine_positions(self):
        """Generate reference leg positions"""
        # Constants used in calculations
        l1m = 32.25
        l1corr = 0.09
        
        # Get requested coordinates and angles
        x = self.defaultLegTransform[0]
        y = self.defaultLegTransform[1]
        z = self.defaultLegTransform[2]
        a = np.degrees(self.defaultLegTransform[3])
        b = np.degrees(self.defaultLegTransform[4])
        g = np.degrees(self.defaultLegTransform[5])
        
        # Set kartesian coordinates for all legs
        # Leg 1 (front left)
        atemp = np.arctan2(y, x+l1m)
        self.kartesian_ref[0,0] = -(np.sqrt((x+l1m)**2 + y**2))*np.cos(atemp)
        self.kartesian_ref[0,1] = (np.sqrt((x+l1m+l1corr)**2 + y**2))*np.sin(atemp)
        
        # Leg 2 (front right)
        atemp = np.arctan2(y, x+l1m)
        self.kartesian_ref[1,0] = (np.sqrt((x+l1m)**2 + y**2))*np.cos(atemp)
        self.kartesian_ref[1,1] = (np.sqrt((x+l1m+l1corr)**2 + y**2))*np.sin(atemp)
        
        # Leg 3 (middle left)
        self.kartesian_ref[2,0] = -(x+l1m)
        self.kartesian_ref[2,1] = y
        
        # Leg 4 (middle right)
        self.kartesian_ref[3,0] = x+l1m
        self.kartesian_ref[3,1] = y
        
        # Leg 5 (back left)
        atemp = np.arctan2(y, x+l1m)
        self.kartesian_ref[4,0] = -(np.sqrt((x+l1m)**2 + y**2))*np.cos(atemp)
        self.kartesian_ref[4,1] = (np.sqrt((x+l1m+l1corr)**2 + y**2))*np.sin(atemp)
        
        # Leg 6 (back right)
        atemp = np.arctan2(y, x+l1m)
        self.kartesian_ref[5,0] = (np.sqrt((x+l1m)**2 + y**2))*np.cos(atemp)
        self.kartesian_ref[5,1] = (np.sqrt((x+l1m+l1corr)**2 + y**2))*np.sin(atemp)
        
        # Set Z coordinates
        self.kartesian_ref[:,2] = z

    def walking(self):
        """Implement walking procedure for all legs"""
        # Only walk if a direction is set
        if self.walking_direction == 'none' and self.turn_direction == 'none':
            self.is_walking = False
            return
            
        # distances the leg should travel
        y_delta = 30
        z_delta = 30
        
        # Modify movement based on direction
        if self.walking_direction == 'forward':
            y_delta = 30
        elif self.walking_direction == 'backward':
            y_delta = -30
        elif self.walking_direction == 'left':
            # Use X movement instead of Y for sideways
            for i in range(self.leg_count):
                self.kartesianX[i] = self.kartesian_ref[i,0] - y_delta
            return
        elif self.walking_direction == 'right':
            # Use X movement instead of Y for sideways
            for i in range(self.leg_count):
                self.kartesianX[i] = self.kartesian_ref[i,0] + y_delta
            return
            
        if self.turn_direction == 'left':
            self.kartesian[5] += np.radians(2)  # Rotate left
            return
        elif self.turn_direction == 'right':
            self.kartesian[5] -= np.radians(2)  # Rotate right
            return

        stepsize = 5
        step_time = 2 # cycle time for one step
        
        # indices of leg groups
        indices1 = [0,3,4]
        indices2 = [1,2,5]
        
        # check flag for walking
        if not self.is_walking:
            if self.walking_mode == 1:
                # save timestamp for starting
                self.last_timestamp = time.time()
                # program start, time to set flags
                for i in range(len(indices1)):
                    self.walking_phases[indices1[i]] = step_time/2
                    self.walking_phases[indices2[i]] = step_time/2
            # set flag for walking
            self.is_walking = True
            
        if self.walking_mode == 1:
            # determine time since last loop
            current_timestamp = time.time()
            time_delay = current_timestamp - self.last_timestamp
            # assign last timestamp
            self.last_timestamp = current_timestamp
            # add time_delay to walking phases
            self.walking_phases += time_delay
            # loop over indices
            for i in range(len(indices1)):
                # check if phase is longer than step_time
                if self.walking_phases[indices1[i]] > step_time:
                    self.walking_phases[indices1[i]] -= step_time
                # check for first leg set if first or second phase is reached
                if self.walking_phases[indices1[i]] < step_time/2:
                    # use first equations for current positions
                    self.kartesianY[indices1[i]] = self.kartesian_ref[indices1[i],1] + y_delta*(4*self.walking_phases[indices1[i]]/step_time-1)
                    self.kartesianZ[indices1[i]] = self.kartesian_ref[indices1[i],2] + z_delta*np.sin(2*np.pi/step_time*self.walking_phases[indices1[i]])
                elif self.walking_phases[indices1[i]] <= step_time:
                    self.kartesianY[indices1[i]] = self.kartesian_ref[indices1[i],1] + y_delta*(3-4*self.walking_phases[indices1[i]]/step_time)
                    # kartesianZ[indices1[i]] += 0
                # check for second leg set if first or second phase is reached
                if self.walking_phases[indices2[i]] > step_time:
                    self.walking_phases[indices2[i]] -= step_time
                if self.walking_phases[indices2[i]] < step_time/2:
                    # use first equations for current positions
                    self.kartesianY[indices2[i]] = self.kartesian_ref[indices1[i],1] + y_delta*(1-4*self.walking_phases[indices2[i]]/step_time)
                    # kartesianZ[indices1[i]] += 0
                elif self.walking_phases[indices2[i]] <= step_time:
                    self.kartesianY[indices2[i]] = self.kartesian_ref[indices1[i],1] + y_delta*(4*self.walking_phases[indices2[i]]/step_time-3)
                    self.kartesianZ[indices2[i]] = self.kartesian_ref[indices1[i],2] + z_delta*np.sin(2*np.pi/step_time*(self.walking_phases[indices2[i]]-step_time/2))

    def inverse_kinematics_4(self):
        """Implement inverse kinematics for walking"""
        # constants used in the calculations
        y0 = 56+4.24
        x0 = 31.5
        x1 = 35.74
        l1m = 32.25
        l0m = 28.5
        l0o = 20.15
        l1corr = 0.09
        l2 = 44
        l3 = 69.5
        
        # check if walking is disabled, otherwise kartesianXYZ are already set
        if not self.is_walking:
            # get requested coordinates and angles first
            x1 = self.defaultLegTransform[0]+self.kartesian[0]
            x2 = self.defaultLegTransform[0]-self.kartesian[0]
            y = self.defaultLegTransform[1]-self.kartesian[1]
            z = self.defaultLegTransform[2]-self.kartesian[2]
            a = np.degrees(self.defaultLegTransform[3]+self.kartesian[3])
            b = np.degrees(self.defaultLegTransform[4]+self.kartesian[4])
            g = np.degrees(self.defaultLegTransform[5]+self.kartesian[5])
            
            # set supposed kartesian coordinates for all legs
            # -------------leg 1-------------
            atemp = np.arctan2(y,x1+l1m)
            self.kartesianX[0] = -(np.sqrt((x1+l1m)**2+y**2))*np.cos(atemp)
            self.kartesianY[0] = (np.sqrt((x1+l1m+l1corr)**2+y**2))*np.sin(atemp)
            # -------------leg 2-------------
            atemp = np.arctan2(y,x2+l1m)
            self.kartesianX[1] = (np.sqrt((x2+l1m)**2+y**2))*np.cos(atemp)
            self.kartesianY[1] = (np.sqrt((x2+l1m+l1corr)**2+y**2))*np.sin(atemp)
            # -------------leg 3-------------
            self.kartesianX[2] = -(x1+l1m)
            self.kartesianY[2] = y
            # -------------leg 4-------------
            self.kartesianX[3] = x2+l1m
            self.kartesianY[3] = y
            # -------------leg 5-------------
            atemp = np.arctan2(y,x1+l1m)
            self.kartesianX[4] = -(np.sqrt((x1+l1m)**2+y**2))*np.cos(atemp)
            self.kartesianY[4] = (np.sqrt((x1+l1m+l1corr)**2+y**2))*np.sin(atemp)
            # -------------leg 6-------------
            atemp = np.arctan2(y,x2+l1m)
            self.kartesianX[5] = (np.sqrt((x2+l1m)**2+y**2))*np.cos(atemp)
            self.kartesianY[5] = (np.sqrt((x2+l1m+l1corr)**2+y**2))*np.sin(atemp)
            # -------------z-------------
            self.kartesianZ[0] = z
            self.kartesianZ[1] = z
            self.kartesianZ[2] = z
            self.kartesianZ[3] = z
            self.kartesianZ[4] = z
            self.kartesianZ[5] = z
            
            # Apply rotations
            # -------------------1. yaw-----------------------
            # front servos
            dy = (y0+l0o)*(1-np.cos(np.radians(a)))
            dz = (y0+l0o)*np.sin(np.radians(a))
            self.kartesianY[0] -= dy
            self.kartesianZ[0] -= dz
            self.kartesianY[1] -= dy
            self.kartesianZ[1] -= dz
            # back servos
            self.kartesianY[4] += dy
            self.kartesianZ[4] += dz
            self.kartesianY[5] += dy
            self.kartesianZ[5] += dz
            
            # -------------------2. pitch---------------------
            # front and back servos
            dx = 2*(x1+l0o)*(1-np.cos(np.radians(b)))
            dz = 2*(x1+l0o)*np.sin(np.radians(b))
            self.kartesianX[0] += dx
            self.kartesianZ[0] -= dz
            self.kartesianX[1] -= dx
            self.kartesianZ[1] += dz
            self.kartesianX[4] += dx
            self.kartesianZ[4] -= dz
            self.kartesianX[5] -= dx
            self.kartesianZ[5] += dz
            # middle servos
            dx = 2*(x0+l0m)*(1-np.cos(np.radians(b)))
            dz = 2*(x0+l0m)*np.sin(np.radians(b))
            self.kartesianX[2] += dx
            self.kartesianZ[2] -= dz
            self.kartesianX[3] -= dx
            self.kartesianZ[3] += dz
            
            # -------------------3. roll----------------------
            # front servos
            g0 = np.degrees(np.arctan((y0+l0o)/(x1+l0o)))
            dx = 1.25*np.sqrt((y0+l0o)**2+(x1+l0o)**2)*np.sin(np.radians(g0+g))
            dy = 1.25*np.sqrt((y0+l0o)**2+(x1+l0o)**2)*np.sin(np.radians(g0+g))
            self.kartesianX[0] += dx
            self.kartesianY[0] += dy
            self.kartesianX[1] += dx
            self.kartesianY[1] -= dy
            # middle servos
            comp_fac = (l2+l3+l0m)/(x0+l0m)
            dy = comp_fac*(x0+l0m)*np.sin(np.radians(g))
            dx = comp_fac*(x0+l0m)*(1-np.cos(np.radians(g)))
            self.kartesianX[2] += dx
            self.kartesianY[2] += dy
            self.kartesianX[3] -= dx
            self.kartesianY[3] -= dy
            # back servos
            dx = np.sqrt((y0+l0o)**2+(x1+l0o)**2)*np.sin(np.radians(g0+g))
            dy = np.sqrt((y0+l0o)**2+(x1+l0o)**2)*np.sin(np.radians(g0+g))
            self.kartesianX[4] -= dx
            self.kartesianY[4] += dy
            self.kartesianX[5] -= dx
            self.kartesianY[5] -= dy
        
        # Calculate angles for each leg
        for i in range(self.leg_count):
            # Calculate intermediate values
            x1 = np.sqrt(self.kartesianX[i]**2 + self.kartesianY[i]**2)
            a1 = np.arcsin(self.kartesianY[i]/x1)
            x2 = np.sqrt((x1-l1m)**2 + self.kartesianZ[i]**2)
            
            try:
                a2 = np.arccos((l2**2 + l3**2 - x2**2)/(2*l2*l3))
                b1 = np.arccos((l2**2 + x2**2 - l3**2)/(2*l2*x2))
                g1 = np.arcsin(self.kartesianZ[i]/x2)
                b2 = b1 + g1
                
                # Convert to degrees and store
                if i == 0:  # front left
                    self.kartesianAngles[0] = np.degrees(a1) - 45
                    self.kartesianAngles[1] = np.degrees(b2)
                    self.kartesianAngles[2] = 180 - np.degrees(a2)
                elif i == 1:  # front right
                    self.kartesianAngles[3] = np.degrees(a1) - 45
                    self.kartesianAngles[4] = np.degrees(b2)
                    self.kartesianAngles[5] = 180 - np.degrees(a2)
                elif i == 2:  # middle left
                    self.kartesianAngles[6] = np.degrees(a1)
                    self.kartesianAngles[7] = np.degrees(b2)
                    self.kartesianAngles[8] = 180 - np.degrees(a2)
                elif i == 3:  # middle right
                    self.kartesianAngles[9] = np.degrees(a1)
                    self.kartesianAngles[10] = np.degrees(b2)
                    self.kartesianAngles[11] = 180 - np.degrees(a2)
                elif i == 4:  # back left
                    self.kartesianAngles[12] = np.degrees(a1) + 45
                    self.kartesianAngles[13] = np.degrees(b2)
                    self.kartesianAngles[14] = 180 - np.degrees(a2)
                elif i == 5:  # back right
                    self.kartesianAngles[15] = np.degrees(a1) + 45
                    self.kartesianAngles[16] = np.degrees(b2)
                    self.kartesianAngles[17] = 180 - np.degrees(a2)
            except:
                print(f"Inverse kinematics failed for leg {i}")
                continue
        
        # Transfer angles to all_angles array
        for i in range(len(self.all_angles)):
            for j in range(len(self.all_angles[i])):
                self.all_angles[i][j] = np.radians(self.kartesianAngles[i*len(self.all_angles[i])+j])

    def create_simulation_window(self):
        """Create 3D simulation window"""
        if not hasattr(self, 'simulation_window'):
            self.simulation_window = tk.Toplevel(self.root)
            self.simulation_window.title("Hexapod Simulation")
            self.simulation_window.geometry("800x800")

            # Create matplotlib figure
            self.simulation_figure = plt.figure(figsize=(10, 10))
            self.ax = self.simulation_figure.add_subplot(111, projection='3d')
            self.ax.set_title("Hexapod Simulation")

            # Set axis limits and labels
            self.ax.set_xlim(self.x_lim)
            self.ax.set_ylim(self.y_lim)
            self.ax.set_zlim(self.z_lim)
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")

            # Create canvas
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
            self.canvas = FigureCanvasTkAgg(self.simulation_figure, master=self.simulation_window)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

            # Add toolbar
            toolbar = NavigationToolbar2Tk(self.canvas, self.simulation_window)
            toolbar.update()

            # Initialize simulation
            self.initial_draw()
            self.start_simulation_update()

def main():
    root = tk.Tk()
    app = HexapodGUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.cleanup(), root.destroy()))
    root.mainloop()

if __name__ == "__main__":
    main()
