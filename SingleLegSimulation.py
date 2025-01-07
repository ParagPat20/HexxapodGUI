# -*- coding: utf-8 -*-
"""
Simulation environment for inverse kinematic model of a single leg from a hexapod.
This script creates an animated plot with adjustable variables to verify the 
behavior of the inverse kinematic model.

The simulation provides:
1. Interactive 3D visualization of a 3-segment robotic leg
2. Real-time inverse kinematics calculation
3. Adjustable segment lengths and target position
4. Visual feedback of joint angles
5. Workspace validation

Key Components:
- Leg Segments:
    * Coxa (L1): First segment, rotates around z-axis
    * Femur (L2): Second segment, provides elevation
    * Tibia (L3): Third segment, final extension

- Control Parameters:
    * Target Position (X, Y, Z): End effector target coordinates
    * Segment Lengths (L1, L2, L3): Adjustable leg segment lengths

- Visualization:
    * 3D plot with leg segments
    * Joint angle indicators
    * Target point marker
    * Workspace boundaries

Usage:
    Run this script directly to launch the interactive simulation.
    Use sliders to adjust target position and leg segment lengths.
    The simulation will automatically update showing:
    - Leg configuration
    - Joint angles
    - Reachability warnings
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math
import time
from config_handler import ConfigHandler

# Initial leg dimensions (in millimeters)
L1_DEFAULT = 36   # Length of first segment (coxa) - connects body to first joint
L2_DEFAULT = 67   # Length of second segment (femur) - provides elevation
L3_DEFAULT = 230  # Length of third segment (tibia) - extends to end effector

# Simplified math functions for cleaner trigonometric calculations
PI = np.pi      # Mathematical constant π
sin = np.sin    # Sine function
cos = np.cos    # Cosine function
atan2 = np.arctan2  # Two-argument arctangent - returns angle in correct quadrant
acos = np.arccos    # Inverse cosine function
sqrt = np.sqrt      # Square root function

# Add walking control variables after the initial constants
# Walking control parameters
is_walking = False          # Flag to control walking state
walking_phase = 0           # Current phase of walking cycle
last_timestamp = 0          # Timestamp for timing control
walking_direction = 'none'  # Direction of movement
step_height = 80           # Height of leg lift during step (mm)
step_length = 150          # Length of step forward/backward (mm)
step_time = 2.0           # Time for complete step cycle (seconds)

# Initialize config handler
config_handler = ConfigHandler()
saved_config = config_handler.load_config()

# Initialize motor settings from saved config
motor_offsets = saved_config.get('offsets', {
    'coxa': 0,    # Offset for first joint (L1)
    'femur': 0,   # Offset for second joint (L2)
    'tibia': 0    # Offset for third joint (L3)
})

motor_inversions = saved_config.get('inverted_motors', {
    'coxa': False,   # Inversion flag for first joint
    'femur': False,  # Inversion flag for second joint
    'tibia': False   # Inversion flag for third joint
})

class LegSimulation:
    """Interactive simulation environment for a 3-segment robotic leg
    
    This class provides a complete simulation environment for testing and
    visualizing inverse kinematics of a robotic leg with three segments.
    It handles:
    - 3D visualization
    - Inverse kinematics calculations
    - Forward kinematics validation
    - Interactive parameter adjustment
    - Workspace visualization
    - Joint angle feedback
    
    The leg model consists of three segments:
    1. Coxa (L1): Base segment that rotates around z-axis
    2. Femur (L2): Middle segment that provides elevation
    3. Tibia (L3): End segment that extends to the target
    
    The simulation uses:
    - Inverse kinematics to calculate joint angles for a target position
    - Forward kinematics to validate the solution
    - Interactive matplotlib sliders for real-time control
    - 3D visualization for intuitive feedback
    """
    def __init__(self):
        """Initialize the leg simulation visualization environment
        
        Sets up:
        - 3D plotting environment
        - Leg segment lengths and workspace
        - Interactive sliders for position and segment lengths
        - Visualization elements (leg lines, angle arcs, text)
        """
        # Create main figure and 3D axes for visualization
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize leg segment lengths
        self.L1 = L1_DEFAULT
        self.L2 = L2_DEFAULT
        self.L3 = L3_DEFAULT
        
        # Set initial target point at a reasonable starting position
        # For the longer leg segments, we need a more conservative initial position
        self.target = np.array([150, 0, -150])  # Adjusted for longer segments
        
        # Initialize visualization elements
        self.angle_annotations = []
        self.angle_arcs = []
        self.angle_text = self.ax.text2D(0.02, 0.98, '', 
                                        transform=self.ax.transAxes, 
                                        fontsize=10, 
                                        verticalalignment='top')
        
        # Initialize plot elements
        self.leg_line, = self.ax.plot([], [], [], 'b-', linewidth=2)
        self.target_point, = self.ax.plot([], [], [], 'ro')
        
        # Set workspace limits based on leg dimensions
        self.limit = 500  # Fixed limit for better visualization with long tibia
        self.ax.set_xlim([-self.limit/2, self.limit])  # More space in forward direction
        self.ax.set_ylim([-self.limit/2, self.limit/2])
        self.ax.set_zlim([-self.limit/2, 0])  # Only show lower half for Z
        
        # Position sliders with adjusted ranges for longer segments
        self.slider_x = Slider(plt.axes([0.1, 0.25, 0.65, 0.03]), 'X', 
                              0, 350, valinit=150)  # Increased forward range
        self.slider_y = Slider(plt.axes([0.1, 0.20, 0.65, 0.03]), 'Y', 
                              -150, 150, valinit=0)  # Reduced side range
        self.slider_z = Slider(plt.axes([0.1, 0.15, 0.65, 0.03]), 'Z', 
                              -300, -50, valinit=-150)  # Adjusted Z range
        
        # Length sliders
        self.slider_l1 = Slider(plt.axes([0.1, 0.10, 0.65, 0.03]), 'L1', 
                               0, L1_DEFAULT*2, valinit=L1_DEFAULT)
        self.slider_l2 = Slider(plt.axes([0.1, 0.05, 0.65, 0.03]), 'L2', 
                               0, L2_DEFAULT*2, valinit=L2_DEFAULT)
        self.slider_l3 = Slider(plt.axes([0.1, 0.02, 0.65, 0.03]), 'L3', 
                               0, L3_DEFAULT*2, valinit=L3_DEFAULT)
        
        # Add offset and inversion controls
        plt.subplots_adjust(bottom=0.5)  # Make more room for controls
        
        # Offset sliders
        self.slider_offset_l1 = Slider(plt.axes([0.1, 0.08, 0.65, 0.02]), 'L1 Offset', 
                                     -45, 45, valinit=0)
        self.slider_offset_l2 = Slider(plt.axes([0.1, 0.05, 0.65, 0.02]), 'L2 Offset', 
                                     -45, 45, valinit=0)
        self.slider_offset_l3 = Slider(plt.axes([0.1, 0.02, 0.65, 0.02]), 'L3 Offset', 
                                     -45, 45, valinit=0)
        
        # Inversion checkboxes
        checkbox_ax1 = plt.axes([0.8, 0.08, 0.15, 0.02])
        checkbox_ax2 = plt.axes([0.8, 0.05, 0.15, 0.02])
        checkbox_ax3 = plt.axes([0.8, 0.02, 0.15, 0.02])
        
        self.invert_l1 = plt.Button(checkbox_ax1, 'Invert L1')
        self.invert_l2 = plt.Button(checkbox_ax2, 'Invert L2')
        self.invert_l3 = plt.Button(checkbox_ax3, 'Invert L3')
        
        # Connect events
        self.slider_x.on_changed(self.update)
        self.slider_y.on_changed(self.update)
        self.slider_z.on_changed(self.update)
        self.slider_l1.on_changed(self.update_lengths)
        self.slider_l2.on_changed(self.update_lengths)
        self.slider_l3.on_changed(self.update_lengths)
        
        # Connect offset and inversion events
        self.slider_offset_l1.on_changed(self.update_offsets)
        self.slider_offset_l2.on_changed(self.update_offsets)
        self.slider_offset_l3.on_changed(self.update_offsets)
        
        self.invert_l1.on_clicked(lambda x: self.toggle_inversion('coxa'))
        self.invert_l2.on_clicked(lambda x: self.toggle_inversion('femur'))
        self.invert_l3.on_clicked(lambda x: self.toggle_inversion('tibia'))
        
        # Set initial values from config
        self.slider_offset_l1.set_val(motor_offsets['coxa'])
        self.slider_offset_l2.set_val(motor_offsets['femur'])
        self.slider_offset_l3.set_val(motor_offsets['tibia'])
        
        # Initial update
        self.update(None)

    def update_lengths(self, val):
        """Update leg segment lengths and adjust plot limits
        
        Args:
            val: Value from the slider (not used directly)
            
        This function updates the leg segment lengths when sliders are moved and
        recalculates the workspace limits accordingly.
        """
        # Update the length of each segment from slider values
        self.L1 = self.slider_l1.val  # Length of coxa (first segment)
        self.L2 = self.slider_l2.val  # Length of femur (second segment)
        self.L3 = self.slider_l3.val  # Length of tibia (third segment)
        
        # Calculate new workspace limits based on maximum leg extension
        # Maximum reach is sum of all segment lengths
        self.limit = self.L1 + self.L2 + self.L3
        
        # Update 3D plot boundaries to show full workspace
        self.ax.set_xlim([-self.limit, self.limit])
        self.ax.set_ylim([-self.limit, self.limit])
        self.ax.set_zlim([-self.limit, self.limit])
        
        # Update the range of position sliders to match new workspace
        self.slider_x.valmin = -self.limit
        self.slider_x.valmax = self.limit
        self.slider_y.valmin = -self.limit
        self.slider_y.valmax = self.limit
        self.slider_z.valmin = -self.limit
        self.slider_z.valmax = self.limit
        
        # Redraw the leg with new dimensions
        self.update(None)

    def inverse_kinematics(self, target):
        """Calculate joint angles for given target point using inverse kinematics
        
        Args:
            target: numpy array [x, y, z] representing the target point in 3D space
            
        Returns:
            numpy array [theta1, theta2, theta3] representing the three joint angles in radians
            None if target is unreachable
        """
        x, y, z = target
        
        try:
            # Calculate theta1 (yaw angle) - This is the angle of rotation around the z-axis
            # Using atan2 to handle all quadrants correctly
            theta1 = atan2(y, x)
            
            # Calculate the distance 'r' from origin to target point projected on x-y plane
            # This is needed to solve the remaining angles in the vertical plane
            r = sqrt(x**2 + y**2)
            
            # Calculate the direct distance 'd' from the second joint to target point
            # This forms the third side of the triangle used for inverse kinematics
            # We subtract L1 from r because we need the distance from joint 2, not origin
            d = sqrt((r - self.L1)**2 + z**2)
            
            # Check if target is beyond the maximum reach of the leg
            # Maximum reach is the sum of L2 and L3 segments
            if d > self.L2 + self.L3:
                print("Target out of reach - Distance too far")
                return None
                
            # Using law of cosines to find angle alpha
            # alpha is the angle between L2 and the line from joint 2 to target (d)
            cos_alpha = (self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d)
            if abs(cos_alpha) > 1:
                print("Target out of reach - Invalid triangle formation")
                return None
                
            alpha = acos(cos_alpha)
            
            # Calculate beta - the angle between horizontal plane and line d
            # This is needed to determine the absolute angle of the second segment
            beta = atan2(z, r - self.L1)
            
            # Calculate theta2 (pitch angle of first arm segment)
            # This is the sum of beta and alpha to get the absolute angle
            theta2 = beta + alpha
            
            # Calculate gamma - the angle between segments L2 and L3
            # Again using law of cosines for the triangle formed by L2, L3, and d
            cos_gamma = (self.L2**2 + self.L3**2 - d**2) / (2 * self.L2 * self.L3)
            if abs(cos_gamma) > 1:
                print("Target out of reach - Segments cannot form triangle")
                return None
                
            gamma = acos(cos_gamma)
            # Adjust gamma to get theta3 (relative angle between segments)
            # Subtract PI to convert from absolute angle to relative angle
            theta3 = gamma - PI
            
            return np.array([theta1, theta2, theta3])
            
        except Exception as e:
            print(f"IK calculation error: {e}")
            return None

    def forward_kinematics(self, angles):
        """Calculate joint positions using forward kinematics
        
        Args:
            angles: numpy array [theta1, theta2, theta3] representing joint angles in radians
                    theta1: yaw angle (rotation around z-axis)
                    theta2: pitch angle of first arm segment
                    theta3: angle between second and third segments
            
        Returns:
            numpy array of shape (4,3) containing the [x,y,z] coordinates of:
            - Base joint (origin)
            - First joint (after coxa)
            - Second joint (after femur)
            - End effector (after tibia)
        """
        theta1, theta2, theta3 = angles
        
        # Base joint is always at origin
        p0 = np.array([0, 0, 0])
        
        # First joint position - after coxa rotation
        # Only rotates around z-axis by theta1
        p1 = np.array([
            self.L1 * cos(theta1),  # x = L1 * cos(θ1)
            self.L1 * sin(theta1),  # y = L1 * sin(θ1)
            0                       # z = 0 (coxa moves in horizontal plane)
        ])
        
        # Second joint position - after femur movement
        # Combines coxa rotation (theta1) and femur elevation (theta2)
        x2 = self.L1 * cos(theta1) + self.L2 * cos(theta1) * cos(theta2)
        y2 = self.L1 * sin(theta1) + self.L2 * sin(theta1) * cos(theta2)
        z2 = self.L2 * sin(theta2)  # Vertical component from femur elevation
        p2 = np.array([x2, y2, z2])
        
        # End effector position - after tibia movement
        # Combines all rotations: coxa (theta1), femur (theta2), and tibia (theta3)
        x3 = x2 + self.L3 * cos(theta1) * cos(theta2 + theta3)
        y3 = y2 + self.L3 * sin(theta1) * cos(theta2 + theta3)
        z3 = z2 + self.L3 * sin(theta2 + theta3)  # Add tibia's vertical component
        p3 = np.array([x3, y3, z3])
        
        # Return all joint positions as a single array
        return np.array([p0, p1, p2, p3])

    def draw_angle_arc(self, center, v1, v2, radius=20):
        """Draw an arc to visualize the angle between two vectors in 3D space
        
        Args:
            center: numpy array [x,y,z] - center point of the arc
            v1: first vector
            v2: second vector
            radius: radius of the visualization arc (default=20)
            
        Returns:
            tuple (arc_3d, angle):
                arc_3d: numpy array of points forming the arc
                angle: calculated angle between vectors in radians
        """
        # Project 3D vectors onto x-z plane for visualization
        # This simplifies the angle visualization while maintaining clarity
        v1_proj = np.array([v1[0], v1[2]])  # Project to x-z plane
        v2_proj = np.array([v2[0], v2[2]])  # Project to x-z plane
        
        # Normalize projected vectors for angle calculation
        v1_proj = v1_proj / np.linalg.norm(v1_proj)
        v2_proj = v2_proj / np.linalg.norm(v2_proj)
        
        # Calculate angle between projected vectors
        angle = np.arccos(np.clip(np.dot(v1_proj, v2_proj), -1.0, 1.0))
        
        # Create points for drawing the arc
        n_points = 20  # Number of points to form the arc
        theta = np.linspace(0, angle, n_points)
        
        # Create rotation matrix to align arc with first vector
        c = v1_proj[0]  # cosine component
        s = v1_proj[1]  # sine component
        R = np.array([[c, -s], [s, c]])
        
        # Generate arc points in 2D
        arc_points = radius * np.array([np.cos(theta), np.sin(theta)])
        arc_points = np.dot(R, arc_points)  # Rotate to align with first vector
        
        # Convert arc points back to 3D coordinates
        arc_3d = np.zeros((3, n_points))
        arc_3d[0] = center[0] + arc_points[0]  # X coordinates
        arc_3d[1] = center[1]                  # Y coordinates (constant)
        arc_3d[2] = center[2] + arc_points[1]  # Z coordinates
        
        return arc_3d, angle

    def start_walking(self, direction='forward'):
        """Start the leg walking motion
        
        Args:
            direction: Direction of movement ('forward', 'backward', 'none')
        """
        global is_walking, walking_direction, last_timestamp
        walking_direction = direction
        if direction != 'none':
            is_walking = True
            last_timestamp = time.time()
        else:
            is_walking = False

    def update_walking(self):
        """Update leg position for walking animation
        
        This implements a simple walking gait with four phases:
        1. Lift leg up
        2. Move leg forward
        3. Lower leg down
        4. Move leg backward (stance phase)
        """
        global is_walking, walking_phase, last_timestamp
        global walking_direction, step_height, step_length, step_time
        
        if not is_walking or walking_direction == 'none':
            return
            
        # Calculate time since last update
        current_time = time.time()
        dt = current_time - last_timestamp
        last_timestamp = current_time
        
        # Update walking phase
        walking_phase += dt
        if walking_phase > step_time:
            walking_phase -= step_time
            
        # Calculate leg position based on phase
        if walking_phase < step_time/2:
            # Swing phase (leg in air)
            progress = 4 * walking_phase/step_time - 1  # -1 to 1
            if walking_direction == 'forward':
                self.target[0] = 150 + step_length * progress  # Start from 150mm
            else:  # backward
                self.target[0] = 150 - step_length * progress
            self.target[2] = -150 + step_height * sin(2*PI/step_time * walking_phase)  # Start from -150mm
        else:
            # Stance phase (leg on ground)
            progress = 3 - 4 * walking_phase/step_time  # 1 to -1
            if walking_direction == 'forward':
                self.target[0] = 150 + step_length * progress
            else:  # backward
                self.target[0] = 150 - step_length * progress
            self.target[2] = -150  # Keep leg at -150mm height

    def update_offsets(self, val=None):
        """Update motor offsets and save to config"""
        motor_offsets['coxa'] = self.slider_offset_l1.val
        motor_offsets['femur'] = self.slider_offset_l2.val
        motor_offsets['tibia'] = self.slider_offset_l3.val
        
        # Save to config
        config = {
            'offsets': motor_offsets,
            'inverted_motors': motor_inversions
        }
        config_handler.save_config(config)
        
        self.update(None)

    def toggle_inversion(self, motor):
        """Toggle motor inversion and save to config"""
        motor_inversions[motor] = not motor_inversions[motor]
        
        # Save to config
        config = {
            'offsets': motor_offsets,
            'inverted_motors': motor_inversions
        }
        config_handler.save_config(config)
        
        self.update(None)

    def apply_offset_and_inversion(self, angles):
        """Apply offsets and inversions to angles
        
        Args:
            angles: numpy array [theta1, theta2, theta3] in radians
            
        Returns:
            numpy array of adjusted angles in radians
        """
        if angles is None:
            return None
            
        adjusted_angles = angles.copy()
        
        # Convert to degrees for easier manipulation
        degrees = np.degrees(adjusted_angles)
        
        # Apply inversions first
        if motor_inversions['coxa']:
            degrees[0] = 180 - degrees[0]
        if motor_inversions['femur']:
            degrees[1] = 180 - degrees[1]
        if motor_inversions['tibia']:
            degrees[2] = 180 - degrees[2]
            
        # Then apply offsets
        degrees[0] += motor_offsets['coxa']
        degrees[1] += motor_offsets['femur']
        degrees[2] += motor_offsets['tibia']
        
        # Convert back to radians
        return np.radians(degrees)

    def update(self, val):
        """Update the visualization when sliders change
        
        Args:
            val: Value from the slider (not used directly)
        """
        # Update walking if active
        if is_walking:
            self.update_walking()
        else:
            # Get current target position from sliders
            self.target = np.array([self.slider_x.val, 
                                  self.slider_y.val, 
                                  self.slider_z.val])
        
        # Calculate joint angles using inverse kinematics
        angles = self.inverse_kinematics(self.target)
        
        if angles is not None:
            # Apply offsets and inversions
            adjusted_angles = self.apply_offset_and_inversion(angles)
            
            # Calculate joint positions using forward kinematics with adjusted angles
            points = self.forward_kinematics(adjusted_angles)
            
            # Update the leg segment visualization
            self.leg_line.set_data_3d(points[:,0], points[:,1], points[:,2])
            self.target_point.set_data_3d([self.target[0]], 
                                        [self.target[1]], 
                                        [self.target[2]])
            
            # Clear previous angle visualizations
            for line in self.angle_arcs:
                if line:
                    line.remove()
            self.angle_arcs = []
            
            # Calculate vectors between joints for angle visualization
            v1 = points[1] - points[0]  # Vector from base to first joint
            v2 = points[2] - points[1]  # Vector from first to second joint
            v3 = points[3] - points[2]  # Vector from second joint to end effector
            
            # Draw arcs to visualize joint angles
            arc1_points, angle1 = self.draw_angle_arc(points[1], v1, v2)
            arc2_points, angle2 = self.draw_angle_arc(points[2], v2, v3)
            
            # Plot the angle arcs
            arc1 = self.ax.plot(arc1_points[0], arc1_points[1], arc1_points[2], 
                               'r-', alpha=0.5, linewidth=2)[0]
            arc2 = self.ax.plot(arc2_points[0], arc2_points[1], arc2_points[2], 
                               'r-', alpha=0.5, linewidth=2)[0]
            self.angle_arcs = [arc1, arc2]
            
            # Update angle text display
            angle_text = f'Raw Angles:\n' \
                        f'θ1: {math.degrees(angles[0]):.1f}°\n' \
                        f'θ2: {math.degrees(angles[1]):.1f}°\n' \
                        f'θ3: {math.degrees(angles[2]):.1f}°\n' \
                        f'\nAdjusted Angles:\n' \
                        f'θ1: {math.degrees(adjusted_angles[0]):.1f}°\n' \
                        f'θ2: {math.degrees(adjusted_angles[1]):.1f}°\n' \
                        f'θ3: {math.degrees(adjusted_angles[2]):.1f}°'
            self.angle_text.set_text(angle_text)
        
        # Trigger a redraw of the figure
        self.fig.canvas.draw_idle()

if __name__ == "__main__":
    """
    Main execution block for the leg simulation
    
    This block:
    1. Creates an instance of the LegSimulation class
    2. Displays the interactive visualization window
    3. Starts the matplotlib event loop
    
    Usage:
        Run this script directly to launch the simulation:
        $ python SingleLegSimulation.py
        
    Controls:
        - Use the sliders at the bottom to control:
            * X, Y, Z: Target position coordinates
            * L1, L2, L3: Leg segment lengths
        - Close the window to exit the simulation
    """
    try:
        # Create and initialize the simulation
        print("Initializing leg simulation...")
        sim = LegSimulation()
        
        # Add a title to the plot
        sim.ax.set_title("Hexapod Leg Inverse Kinematics Simulation")
        
        # Set initial target position
        sim.target = np.array([150, 0, -150])  # Adjusted initial position
        sim.slider_x.set_val(150)
        sim.slider_y.set_val(0)
        sim.slider_z.set_val(-150)
        
        # Display usage instructions in the console
        print("\nSimulation controls:")
        print("- Use sliders to adjust target position (X, Y, Z)")
        print("- Use sliders to adjust leg segment lengths (L1, L2, L3)")
        print("- Close the window to exit")
        print("\nVisualization:")
        print("- Blue line: Leg segments")
        print("- Red dot: Target position")
        print("- Red arcs: Joint angles")
        print("- Text: Current joint angles in degrees")
        
        # Add keyboard event handling for walking control
        def on_key(event):
            if event.key == 'w':
                sim.start_walking('forward')
            elif event.key == 's':
                sim.start_walking('backward')
            elif event.key == 'x':
                sim.start_walking('none')
        
        # Connect keyboard handler
        sim.fig.canvas.mpl_connect('key_press_event', on_key)
        
        # Add walking control instructions
        print("\nWalking controls:")
        print("- Press 'w' to walk forward")
        print("- Press 's' to walk backward")
        print("- Press 'x' to stop walking")
        
        # Start the matplotlib event loop
        print("\nStarting simulation... (close window to exit)")
        plt.show()
        
    except Exception as e:
        print(f"Error running simulation: {e}")
        raise
    finally:
        print("\nSimulation ended.")

