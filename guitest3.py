import tkinter as tk
from tkinter import ttk, filedialog, simpledialog
import serial
import os
import time

# Setup the serial communication with the ESP32 (replace 'COMx' with your port)
ser = serial.Serial('COM3', 9600)  # Replace with your correct port

class ServoEntry:
    def __init__(self, frame, label_text, row):
        self.label = tk.Label(frame, text=label_text)
        self.label.grid(row=row, column=0, padx=5, pady=5, sticky='w')

        self.entry = tk.Entry(frame, width=8)
        self.entry.grid(row=row, column=1, padx=5, pady=5)

class Leg:
    def __init__(self, frame, leg_label, servo_labels, row_offset):
        self.leg_title = tk.Label(frame, text=leg_label, font=("Helvetica", 12, "bold"))
        self.leg_title.grid(row=row_offset, column=0, columnspan=2, pady=10)

        self.servos = []
        for i, servo_label in enumerate(servo_labels):
            servo_entry = ServoEntry(frame, servo_label, row_offset + i + 1)
            self.servos.append(servo_entry)

    def get_values(self):
        return [servo.entry.get() for servo in self.servos if servo.entry.get()]

    def set_values(self, values):
        for servo, value in zip(self.servos, values):
            servo.entry.delete(0, tk.END)
            servo.entry.insert(0, value)

# Function to send commands to ESP32
def send_command(event=None):
    for leg_index, leg in enumerate(left_legs + right_legs):
        for servo_index, servo in enumerate(leg.servos):
            value = servo.entry.get()
            if value:
                side = 'L' if leg_index < len(left_legs) else 'R'
                command = f"{side}_{leg_index + 1}_{servo_index + 1}_{value}\n"
                ser.write(command.encode('utf-8'))
                print(f"Sent command: {command}")

# Function to receive data
def receive_data():
    if ser.in_waiting:
        incoming_data = ser.readline().decode('utf-8').strip()
        data_text.insert(tk.END, incoming_data + "\n")
        data_text.see(tk.END)

        # Call the parsing function to update the entries
        parse_and_update_entries(incoming_data)

    # Call this function again after 100ms
    root.after(100, receive_data)

def parse_and_update_entries(data):
    try:
        split_data = data.split('_')

        if len(split_data) < 3:
            print(f"Incomplete data: {data}")
            return

        side = split_data[0]
        leg_number = int(split_data[1])
        servo_values = list(map(int, split_data[2:]))

        update_servo_entries(side, leg_number, servo_values)

    except ValueError as e:
        print(f"Error converting values: {e}, data received: {data}")

def update_servo_entries(side, leg_number, servo_values):
    if side == 'L':
        leg = left_legs[leg_number - 1]
    else:
        leg = right_legs[leg_number - 1]

    leg.set_values(servo_values)

# Save values as txt in the positions folder
def save_values_as_txt():
    # Ensure the 'positions' folder exists
    if not os.path.exists('positions'):
        os.makedirs('positions')
    
    save_file = os.path.join('positions', "servo_positions.txt")
    
    with open(save_file, 'a') as file:
        for leg in left_legs + right_legs:
            leg_values = [v for v in leg.get_values() if v]  # Only save valid values
            if leg_values:
                leg_values_str = ', '.join(leg_values)
                leg_entry = f"{leg.leg_title.cget('text')}: {leg_values_str}"
                file.write(leg_entry + "\n")
    
    saved_text.insert(tk.END, "Positions saved to positions folder as txt.\n")
    print(f"Values saved to {save_file}.")

# Save movements as G-code type in the movement folder
def save_values_as_gcode():
    # Ensure the 'movement' folder exists
    if not os.path.exists('movement'):
        os.makedirs('movement')
    
    save_file = os.path.join('movement', "servo_movements.gcode")
    
    with open(save_file, 'a') as file:
        # Get the current servo values
        for leg in left_legs + right_legs:
            leg_values = [v for v in leg.get_values() if v]  # Only save valid values
            if leg_values:
                leg_values_str = ' '.join(leg_values)
                leg_entry = f"G1 {leg.leg_title.cget('text')}: {leg_values_str}"
                file.write(leg_entry + "\n")
        
        # Ask for a delay/timestamp before saving the next values
        delay = simpledialog.askinteger("Add Delay", "Enter delay (in seconds) before the next move:", minvalue=0)
        file.write(f"G4 P{delay * 1000}\n")  # Save the delay in milliseconds (G-code uses milliseconds)
    
    saved_text.insert(tk.END, f"Movement saved as G-code with {delay} seconds delay.\n")
    print(f"Values saved to {save_file}.")

# Load txt file and playback the movements
def load_and_playback():
    load_file = os.path.join('positions', "servo_positions.txt")
    
    if os.path.exists(load_file):
        with open(load_file, 'r') as file:
            lines = file.readlines()
        
        for line in lines:
            data_text.insert(tk.END, f"Executing: {line}\n")
            data_text.see(tk.END)
            root.update()

            if "WAIT" in line:
                delay = int(line.split(': ')[1].split()[0])
                time.sleep(delay)
            else:
                leg_label, values = line.split(": ")
                values_list = [v.strip() for v in values.strip().split(', ')]
                
                if 'Left' in leg_label:
                    index = left_leg_labels.index(leg_label)
                    left_legs[index].set_values(values_list)
                elif 'Right' in leg_label:
                    index = right_leg_labels.index(leg_label)
                    right_legs[index].set_values(values_list)
                
                send_command()

# Create the main window
root = tk.Tk()
root.title("Hexapod Servo Control GUI")

# Add a notebook (tabbed interface)
notebook = ttk.Notebook(root)
notebook.pack(padx=10, pady=10)

# Tab 1: Manual Control Tab
manual_tab = ttk.Frame(notebook)
notebook.add(manual_tab, text="Manual Control")

# Tab 2: Playback and Timestamps Tab
timestamp_tab = ttk.Frame(notebook)
notebook.add(timestamp_tab, text="Playback and Timestamps")

# Shared frame for buttons
shared_button_frame = tk.Frame(root)
shared_button_frame.pack(side=tk.BOTTOM, pady=10)

# Shared buttons for saving and loading values
save_txt_button = tk.Button(shared_button_frame, text="Save Values (txt)", command=save_values_as_txt)
save_txt_button.grid(row=0, column=0, padx=10)

save_gcode_button = tk.Button(shared_button_frame, text="Save Values (G-code)", command=save_values_as_gcode)
save_gcode_button.grid(row=0, column=1, padx=10)

load_button = tk.Button(shared_button_frame, text="Load and Playback", command=load_and_playback)
load_button.grid(row=0, column=2, padx=10)

# Frame for hexapod design and servo entry (Manual Control Tab)
hexapod_frame = tk.Frame(manual_tab)
hexapod_frame.pack(side=tk.LEFT, padx=10, pady=10)

# Left and Right Frames for motors
left_frame = tk.Frame(hexapod_frame)
left_frame.pack(side=tk.LEFT, padx=5)

right_frame = tk.Frame(hexapod_frame)
right_frame.pack(side=tk.RIGHT, padx=5)

# Frame for data display and save (Manual Control Tab)
data_frame = tk.Frame(manual_tab)
data_frame.pack(side=tk.RIGHT, padx=10, pady=10)

# Define servo labels
left_leg_labels = ["Front Left", "Middle Left", "Back Left"]
right_leg_labels = ["Front Right", "Middle Right", "Back Right"]

left_servo_labels = [
    ["LEFT_F_SERVO_1", "LEFT_F_SERVO_2", "LEFT_F_SERVO_3"],
    ["LEFT_M_SERVO_1", "LEFT_M_SERVO_2", "LEFT_M_SERVO_3", "LEFT_M_SERVO_4"],
    ["LEFT_B_SERVO_1", "LEFT_B_SERVO_2", "LEFT_B_SERVO_3"]
]
right_servo_labels = [
    ["RIGHT_F_SERVO_1", "RIGHT_F_SERVO_2", "RIGHT_F_SERVO_3"],
    ["RIGHT_M_SERVO_1", "RIGHT_M_SERVO_2", "RIGHT_M_SERVO_3", "RIGHT_M_SERVO_4"],
    ["RIGHT_B_SERVO_1", "RIGHT_B_SERVO_2", "RIGHT_B_SERVO_3"]
]

# Create legs for manual control
left_legs = [Leg(left_frame, label, servos, i * 5) for i, (label, servos) in enumerate(zip(left_leg_labels, left_servo_labels))]
right_legs = [Leg(right_frame, label, servos, i * 5) for i, (label, servos) in enumerate(zip(right_leg_labels, right_servo_labels))]

# Frame for receiving data from ESP32 (Manual Control Tab)
data_label = tk.Label(data_frame, text="Data from ESP32", font=("Helvetica", 12, "bold"))
data_label.pack(pady=10)

data_text = tk.Text(data_frame, height=15, width=40)
data_text.pack(padx=10, pady=10)

# Label for saved movements (Timestamp/Playback Tab)
saved_label = tk.Label(timestamp_tab, text="Saved Movements", font=("Helvetica", 12, "bold"))
saved_label.pack(pady=10)

# Text area for displaying saved servo values (Timestamp/Playback Tab)
saved_text = tk.Text(timestamp_tab, height=10, width=40)
saved_text.pack(padx=10, pady=10)

# Start receiving data from ESP32
root.after(100, receive_data)

# Bind 'Enter' key to send_command function
root.bind('<Return>', send_command)

# Start the Tkinter main loop
root.mainloop()
