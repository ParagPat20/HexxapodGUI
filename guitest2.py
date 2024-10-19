import tkinter as tk
from tkinter import filedialog, simpledialog
import serial
import os

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
        return [servo.entry.get() for servo in self.servos]

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
    
# Save values to a file in the "positions" folder
def save_values():
    # Ensure the 'positions' folder exists
    if not os.path.exists('positions'):
        os.makedirs('positions')
    
    save_name = simpledialog.askstring("Save As", "Enter a name for this servo configuration:")
    if save_name:
        save_file = os.path.join('positions', f"{save_name}.txt")  # Save in the 'positions' folder
        with open(save_file, 'w') as file:
            for leg in left_legs + right_legs:
                # Get the values and filter out any empty ones
                leg_values = [v for v in leg.get_values() if v]  # Only keep non-empty values
                
                if leg_values:  # Only save if there are valid values
                    leg_values_str = ', '.join(leg_values)
                    leg_entry = f"{leg.leg_title.cget('text')}: {leg_values_str}\n"
                    file.write(leg_entry)
                    
        saved_text.insert(tk.END, f"Saved as {save_file}\n")
        print(f"Values saved to {save_file}.")


# Load values from a file in the "positions" folder and update the entries
def load_values():
    # Ensure the 'positions' folder exists
    if not os.path.exists('positions'):
        os.makedirs('positions')
    
    load_file = filedialog.askopenfilename(
        initialdir='positions',  # Start in the 'positions' folder
        title="Select Servo Configuration", 
        filetypes=[("Text Files", "*.txt")]
    )
    
    if load_file:
        with open(load_file, 'r') as file:
            lines = file.readlines()
            for line in lines:
                leg_label, values = line.split(": ")
                # Clean the values to avoid issues
                values_list = [v.strip() for v in values.strip().split(', ')]  # Clean each value here
                if 'Left' in leg_label:
                    index = left_leg_labels.index(leg_label)
                    left_legs[index].set_values(values_list)
                elif 'Right' in leg_label:
                    index = right_leg_labels.index(leg_label)
                    right_legs[index].set_values(values_list)
        print(f"Loaded values from {load_file}")

        # Automatically send loaded values to ESP32
        # send_command()

        
# Create the main window
root = tk.Tk()
root.title("Hexapod Servo Control GUI")

# Frame for hexapod design and servo entry
hexapod_frame = tk.Frame(root)
hexapod_frame.pack(side=tk.LEFT, padx=10, pady=10)

# Left and Right Frames for motors
left_frame = tk.Frame(hexapod_frame)
left_frame.pack(side=tk.LEFT, padx=5)

right_frame = tk.Frame(hexapod_frame)
right_frame.pack(side=tk.RIGHT, padx=5)

# Frame for data display and save
data_frame = tk.Frame(root)
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

# Create leg instances for left and right legs
left_legs = [Leg(left_frame, label, servos, i * 6) for i, (label, servos) in enumerate(zip(left_leg_labels, left_servo_labels))]
right_legs = [Leg(right_frame, label, servos, i * 6) for i, (label, servos) in enumerate(zip(right_leg_labels, right_servo_labels))]

# Label for incoming data
data_label = tk.Label(data_frame, text="Incoming Data", font=("Helvetica", 12, "bold"))
data_label.pack(pady=10)

# Text area for displaying incoming serial data
data_text = tk.Text(data_frame, height=15, width=40)
data_text.pack(padx=10, pady=10)

# Label for saved data
saved_label = tk.Label(data_frame, text="Saved Values", font=("Helvetica", 12, "bold"))
saved_label.pack(pady=10)

# Text area for saving the current values
saved_text = tk.Text(data_frame, height=10, width=40)
saved_text.pack(padx=10, pady=10)

# Button to save current values
save_button = tk.Button(data_frame, text="Save Values", command=save_values, font=("Helvetica", 12))
save_button.pack(pady=10)

# Button to load saved values
load_button = tk.Button(data_frame, text="Load Values", command=load_values, font=("Helvetica", 12))
load_button.pack(pady=10)

# Bind the Enter key to send command
root.bind('<Return>', send_command)

# Start receiving data from serial
root.after(100, receive_data)

# Run the main Tkinter loop
root.mainloop()

# Close the serial connection when the GUI is closed
ser.close()
