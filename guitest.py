import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import time
import threading
import queue
import os
from config_handler import ConfigHandler

# Create a queue to hold messages from the serial thread
message_queue = queue.Queue()

# Initialize config handler
config_handler = ConfigHandler()

try:
    ser = serial.Serial('COM7', 115200, timeout=1)
except serial.SerialException:
    ser = None
    print("Failed to connect to serial port initially. Will attempt to reconnect...")

def reconnect_serial():
    """Attempt to reconnect to the serial port."""
    global ser
    max_attempts = 5
    attempt = 0
    
    while attempt < max_attempts:
        try:
            # Close the existing connection if it exists
            if ser and ser.is_open:
                ser.close()
            
            # Try to establish a new connection
            ser = serial.Serial('COM7', 115200, timeout=1)
            
            if ser.is_open:
                message = "Successfully reconnected to serial port."
                print(message)
                message_queue.put(message)
                return True
                
        except serial.SerialException as e:
            attempt += 1
            message = f"Reconnection attempt {attempt}/{max_attempts} failed: {str(e)}"
            print(message)
            message_queue.put(message)
            
            if attempt < max_attempts:
                print(f"Retrying in 5 seconds...")
                time.sleep(5)
            else:
                final_message = "Maximum reconnection attempts reached. Please check your connection and restart the application."
                print(final_message)
                message_queue.put(final_message)
                messagebox.showerror("Connection Error", final_message)
                return False

def send_single_value(motor_id, value):
    """Send a single motor value over serial."""
    try:
        if not ser or not ser.is_open:
            if not reconnect_serial():
                return
        data = f"{motor_id}:{int(value)}\n"
        ser.write(data.encode('ascii'))
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {e}")

def validate_input(P):
    """Validate input to only allow numbers and empty string"""
    if P == "": return True
    try:
        value = int(P)
        return 0 <= value <= 2500
    except ValueError:
        return False

def on_slider_change(motor_id, value_var, slider, entry):
    """Called when a slider value changes."""
    value = int(slider.get())
    value_var.set(str(value))
    entry.delete(0, tk.END)
    entry.insert(0, str(value))

def on_slider_release(event, motor_id, slider):
    """Called when a slider is released."""
    value = int(slider.get())
    send_single_value(motor_id, value)

def on_entry_change(event, motor_id, slider, value_var, entry):
    """Called when entry value is changed and Enter is pressed."""
    try:
        value = entry.get()
        if value == "": value = "0"
        value = int(value)
        if 0 <= value <= 2500:
            slider.set(value)
            value_var.set(str(value))
            send_single_value(motor_id, value)
        else:
            entry.delete(0, tk.END)
            entry.insert(0, value_var.get())
    except ValueError:
        entry.delete(0, tk.END)
        entry.insert(0, value_var.get())

def save_current_values():
    """Save current slider values to a config file"""
    values = {}
    for group_name, motors in motor_groups.items():
        group_values = {}
        for motor_id in motors.keys():
            if motor_id in sliders:
                group_values[motor_id] = int(sliders[motor_id].get())
        values[group_name] = group_values
    
    try:
        filename = filedialog.asksaveasfilename(
            initialdir=config_handler.config_dir,
            title="Save Configuration",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")]
        )
        if filename:
            # Extract just the filename from the full path
            name = os.path.basename(filename)
            saved_file = config_handler.save_config(values, name)
            messagebox.showinfo("Success", f"Configuration saved to {saved_file}")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to save configuration: {str(e)}")

def load_saved_values():
    """Load values from a config file"""
    try:
        filename = filedialog.askopenfilename(
            initialdir=config_handler.config_dir,
            title="Load Configuration",
            filetypes=[("JSON files", "*.json")]
        )
        if filename:
            # Extract just the filename from the full path
            values = config_handler.load_config(filename)
            
            print(f"Loaded values: {values}")  # Debug statement
            for group_name, motors in values.items():
                for motor_id, value in motors.items():
                    if motor_id in sliders:
                        print(f"Processing motor_id: {motor_id}")  # Debug statement
                        if motor_id in sliders:
                            print(f"Found {motor_id} in sliders")  # Debug statement
                        if motor_id in entries:
                            print(f"Found {motor_id} in entries")  # Debug statement
                        if motor_id in sliders:
                            if motor_id in entries:
                                sliders[motor_id].set(value)
                                entries[motor_id].delete(0, tk.END)
                                entries[motor_id].insert(0, str(value))
                            else:
                                print(f"Warning: {motor_id} not found in entries")  # Debug statement
                        else:
                            print(f"Warning: {motor_id} not found in sliders")  # Debug statement
                            sliders[motor_id].set(value)
                            values[motor_id].set(str(value))
                            entries[motor_id].delete(0, tk.END)
                            entries[motor_id].insert(0, str(value))
                        # Send the updated value to the device
                        send_single_value(motor_id, value)
            messagebox.showinfo("Success", "Configuration loaded successfully")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load configuration: {str(e)}")

def create_slider(frame, motor_id, motor_name, row):
    label = ttk.Label(frame, text=f"{motor_id} ({motor_name})", style='Motor.TLabel')
    label.grid(row=row, column=0, padx=5, pady=2, sticky='w')
    
    slider = ttk.Scale(frame, from_=0, to=2500, orient=tk.HORIZONTAL, length=250)
    slider.grid(row=row, column=1, padx=5, pady=2, sticky='ew')
    
    value_var = tk.StringVar(value="0")
    
    entry = ttk.Entry(frame, width=6, validate='key', validatecommand=vcmd)
    entry.insert(0, "0")
    entry.grid(row=row, column=2, padx=(5,0), pady=2)
    
    slider.configure(command=lambda v, var=value_var, s=slider, e=entry: 
                    on_slider_change(None, var, s, e))
    slider.bind('<ButtonRelease-1>', 
                lambda e, id=motor_id, s=slider: on_slider_release(e, id, s))
    entry.bind('<Return>', lambda e, id=motor_id, s=slider, var=value_var, ent=entry: 
               on_entry_change(e, id, s, var, ent))
    
    return slider, value_var, entry

# Motor grouping
motor_groups = {
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

# GUI Setup
root = tk.Tk()
root.title("Hexapod Servo Controller")
root.configure(bg='#f0f0f0')

# Main container
main_frame = ttk.Frame(root, padding="10")
main_frame.grid(row=0, column=0, sticky='nsew')

# Style configuration
style = ttk.Style()
style.configure('Header.TLabel', font=('Arial', 20, 'bold'), padding=10)
style.configure('Motor.TLabel', font=('Arial', 11))
style.configure('Group.TLabelframe.Label', font=('Arial', 12, 'bold'))
style.configure('TLabelframe', padding=5)

# Header
header = ttk.Label(main_frame, text="Hexapod Servo Controller", style='Header.TLabel')
header.grid(row=0, column=0, columnspan=2, pady=(0, 20))

# Left side frame
left_side = ttk.Frame(main_frame)
left_side.grid(row=1, column=0, padx=10, sticky='n')

# Right side frame
right_side = ttk.Frame(main_frame)
right_side.grid(row=1, column=1, padx=10, sticky='n')

# Validation command for entries
vcmd = (root.register(validate_input), '%P')

# Create frames for each group
group_frames = {}
sliders = {}
values = {}
entries = {}

# Left side groups
for i, (group_name, motors) in enumerate([
    ('left_front', motor_groups['left_front']),
    ('left_center', motor_groups['left_center']),
    ('left_back', motor_groups['left_back'])
]):
    frame = ttk.LabelFrame(left_side, text=group_name.replace('_', ' ').title())
    frame.grid(row=i, column=0, pady=5, sticky='nsew')
    group_frames[group_name] = frame
    
    for row, (motor_id, motor_name) in enumerate(motors.items()):
        slider, value_var, entry = create_slider(frame, motor_id, motor_name, row)
        sliders[motor_id] = slider
        values[motor_id] = value_var
        entries[motor_id] = entry

# Right side groups
for i, (group_name, motors) in enumerate([
    ('right_front', motor_groups['right_front']),
    ('right_center', motor_groups['right_center']),
    ('right_back', motor_groups['right_back'])
]):
    frame = ttk.LabelFrame(right_side, text=group_name.replace('_', ' ').title())
    frame.grid(row=i, column=0, pady=5, sticky='nsew')
    group_frames[group_name] = frame
    
    for row, (motor_id, motor_name) in enumerate(motors.items()):
        slider, value_var, entry = create_slider(frame, motor_id, motor_name, row)
        sliders[motor_id] = slider
        values[motor_id] = value_var
        entries[motor_id] = entry

# Serial monitor at the bottom
monitor_frame = ttk.LabelFrame(main_frame, text="Serial Monitor")
monitor_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky='ew')

text_widget = tk.Text(monitor_frame, height=6, width=80, font=("Consolas", 10))
text_widget.grid(row=0, column=0, pady=5)

scrollbar = ttk.Scrollbar(monitor_frame, orient="vertical", command=text_widget.yview)
scrollbar.grid(row=0, column=1, sticky='ns')
text_widget.configure(yscrollcommand=scrollbar.set)

# Add buttons frame
buttons_frame = ttk.Frame(main_frame)
buttons_frame.grid(row=3, column=0, columnspan=2, pady=10)

# Add Save and Load buttons
save_button = ttk.Button(buttons_frame, text="Save Values", command=save_current_values)
save_button.pack(side=tk.LEFT, padx=5)

load_button = ttk.Button(buttons_frame, text="Load Values", command=load_saved_values)
load_button.pack(side=tk.LEFT, padx=5)

# Serial communication thread
def read_from_serial():
    while True:
        try:
            if not ser or not ser.is_open:
                if not reconnect_serial():
                    time.sleep(5)
                    continue
            if ser.in_waiting > 0:
                message = ser.readline().decode('ascii').strip()
                message_queue.put(message)
            else:
                time.sleep(0.01)
        except serial.SerialException:
            message_queue.put("Serial connection lost. Attempting to reconnect...")
            if not reconnect_serial():
                time.sleep(5)

def update_text_widget():
    while not message_queue.empty():
        message = message_queue.get()
        text_widget.insert(tk.END, message + '\n')
        text_widget.see(tk.END)
    root.after(50, update_text_widget)

# Start threads
serial_thread = threading.Thread(target=read_from_serial, daemon=True)
serial_thread.start()
update_text_widget()

# Configure weights
main_frame.columnconfigure(1, weight=1)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Run the GUI
root.mainloop()
