# -*- coding: utf-8 -*-
"""
Simulation environment for inverse kinematic model of a single leg from a hexapod.
This script creates an animated plot with adjustable variables to verify the 
behavior of the inverse kinematic model.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Arc
import math

# Initial leg dimensions
L1_DEFAULT = 32.25  # Length of first segment (coxa)
L2_DEFAULT = 44     # Length of second segment (femur)
L3_DEFAULT = 69.5   # Length of third segment (tibia)

# Simplified math functions
PI = np.pi
sin = np.sin
cos = np.cos
atan2 = np.arctan2
acos = np.arccos
sqrt = np.sqrt

class LegSimulation:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize leg lengths
        self.L1 = L1_DEFAULT
        self.L2 = L2_DEFAULT
        self.L3 = L3_DEFAULT
        
        # Target point (will be controlled by sliders)
        self.target = np.array([self.L1 + self.L2, 0, 0])
        
        # Initialize angle visualization attributes
        self.angle_annotations = []
        self.angle_arcs = []
        
        # Add angle text display
        self.angle_text = self.ax.text2D(0.02, 0.98, '', transform=self.ax.transAxes, 
                                        fontsize=10, verticalalignment='top')
        
        # Initialize the plot
        self.leg_line, = self.ax.plot([], [], [], 'b-', linewidth=2)
        self.target_point, = self.ax.plot([], [], [], 'ro')
        
        # Set axis limits and labels
        self.limit = self.L1 + self.L2 + self.L3
        self.ax.set_xlim([-self.limit, self.limit])
        self.ax.set_ylim([-self.limit, self.limit])
        self.ax.set_zlim([-self.limit, self.limit])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        
        # Create position sliders
        plt.subplots_adjust(bottom=0.4)  # Make room for more sliders
        self.slider_x = Slider(plt.axes([0.1, 0.25, 0.65, 0.03]), 'X', -self.limit, self.limit, valinit=self.L1+self.L2)
        self.slider_y = Slider(plt.axes([0.1, 0.20, 0.65, 0.03]), 'Y', -self.limit, self.limit, valinit=0)
        self.slider_z = Slider(plt.axes([0.1, 0.15, 0.65, 0.03]), 'Z', -self.limit, self.limit, valinit=0)
        
        # Create length sliders
        self.slider_l1 = Slider(plt.axes([0.1, 0.10, 0.65, 0.03]), 'L1', 0, L1_DEFAULT*2, valinit=L1_DEFAULT)
        self.slider_l2 = Slider(plt.axes([0.1, 0.05, 0.65, 0.03]), 'L2', 0, L2_DEFAULT*2, valinit=L2_DEFAULT)
        self.slider_l3 = Slider(plt.axes([0.1, 0.02, 0.65, 0.03]), 'L3', 0, L3_DEFAULT*2, valinit=L3_DEFAULT)
        
        # Connect slider events
        self.slider_x.on_changed(self.update)
        self.slider_y.on_changed(self.update)
        self.slider_z.on_changed(self.update)
        self.slider_l1.on_changed(self.update_lengths)
        self.slider_l2.on_changed(self.update_lengths)
        self.slider_l3.on_changed(self.update_lengths)
        
        # Initial update
        self.update(None)

    def update_lengths(self, val):
        """Update leg segment lengths and adjust plot limits"""
        self.L1 = self.slider_l1.val
        self.L2 = self.slider_l2.val
        self.L3 = self.slider_l3.val
        
        # Update plot limits
        self.limit = self.L1 + self.L2 + self.L3
        self.ax.set_xlim([-self.limit, self.limit])
        self.ax.set_ylim([-self.limit, self.limit])
        self.ax.set_zlim([-self.limit, self.limit])
        
        # Update position sliders limits
        self.slider_x.valmin = -self.limit
        self.slider_x.valmax = self.limit
        self.slider_y.valmin = -self.limit
        self.slider_y.valmax = self.limit
        self.slider_z.valmin = -self.limit
        self.slider_z.valmax = self.limit
        
        # Update visualization
        self.update(None)

    def inverse_kinematics(self, target):
        """Calculate joint angles for given target point"""
        x, y, z = target
        
        # Calculate angles
        try:
            # First joint angle (rotation around z)
            theta1 = atan2(y, x)
            
            # Distance in x-y plane from origin to target
            r = sqrt(x**2 + y**2)
            
            # Distance from second joint to target
            d = sqrt((r - self.L1)**2 + z**2)
            
            # Check if target is reachable
            if d > self.L2 + self.L3:
                print("Target out of reach")
                return None
                
            # Angle of the triangle formed by L2, L3, and d
            cos_alpha = (self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d)
            if abs(cos_alpha) > 1:
                print("Target out of reach")
                return None
                
            alpha = acos(cos_alpha)
            
            # Angle from horizontal to d
            beta = atan2(z, r - self.L1)
            
            # Second joint angle
            theta2 = beta + alpha
            
            # Third joint angle
            cos_gamma = (self.L2**2 + self.L3**2 - d**2) / (2 * self.L2 * self.L3)
            if abs(cos_gamma) > 1:
                print("Target out of reach")
                return None
                
            gamma = acos(cos_gamma)
            theta3 = gamma - PI  # Adjust to match our coordinate system
            
            return np.array([theta1, theta2, theta3])
            
        except Exception as e:
            print(f"IK calculation error: {e}")
            return None

    def forward_kinematics(self, angles):
        """Calculate joint positions for given angles"""
        theta1, theta2, theta3 = angles
        
        # First joint at origin
        p0 = np.array([0, 0, 0])
        
        # Second joint
        p1 = np.array([self.L1 * cos(theta1), self.L1 * sin(theta1), 0])
        
        # Third joint
        x2 = self.L1 * cos(theta1) + self.L2 * cos(theta1) * cos(theta2)
        y2 = self.L1 * sin(theta1) + self.L2 * sin(theta1) * cos(theta2)
        z2 = self.L2 * sin(theta2)
        p2 = np.array([x2, y2, z2])
        
        # End effector
        x3 = x2 + self.L3 * cos(theta1) * cos(theta2 + theta3)
        y3 = y2 + self.L3 * sin(theta1) * cos(theta2 + theta3)
        z3 = z2 + self.L3 * sin(theta2 + theta3)
        p3 = np.array([x3, y3, z3])
        
        return np.array([p0, p1, p2, p3])

    def draw_angle_arc(self, center, v1, v2, radius=20):
        """Draw an arc to visualize angle between two vectors in 3D"""
        # Project vectors onto the plane perpendicular to the view direction
        # For simplicity, we'll use the x-z plane (side view)
        v1_proj = np.array([v1[0], v1[2]])
        v2_proj = np.array([v2[0], v2[2]])
        
        # Normalize vectors
        v1_proj = v1_proj / np.linalg.norm(v1_proj)
        v2_proj = v2_proj / np.linalg.norm(v2_proj)
        
        # Calculate angle
        angle = np.arccos(np.clip(np.dot(v1_proj, v2_proj), -1.0, 1.0))
        
        # Create points for an arc
        n_points = 20
        theta = np.linspace(0, angle, n_points)
        
        # Rotation matrix to align arc with first vector
        c = v1_proj[0]
        s = v1_proj[1]
        R = np.array([[c, -s], [s, c]])
        
        # Generate arc points
        arc_points = radius * np.array([np.cos(theta), np.sin(theta)])
        arc_points = np.dot(R, arc_points)
        
        # Convert back to 3D coordinates
        arc_3d = np.zeros((3, n_points))
        arc_3d[0] = center[0] + arc_points[0]  # X coordinates
        arc_3d[1] = center[1]                  # Y coordinates (constant)
        arc_3d[2] = center[2] + arc_points[1]  # Z coordinates
        
        return arc_3d, angle

    def update(self, val):
        """Update the visualization"""
        # Get target position from sliders
        self.target = np.array([self.slider_x.val, self.slider_y.val, self.slider_z.val])
        
        # Calculate joint angles
        angles = self.inverse_kinematics(self.target)
        
        if angles is not None:
            # Calculate joint positions
            points = self.forward_kinematics(angles)
            
            # Update the visualization
            self.leg_line.set_data_3d(points[:,0], points[:,1], points[:,2])
            self.target_point.set_data_3d([self.target[0]], [self.target[1]], [self.target[2]])
            
            # Clear previous angle visualizations
            for line in self.angle_arcs:
                if line:
                    line.remove()
            self.angle_arcs = []
            
            # Calculate vectors between joints
            v1 = points[1] - points[0]  # First segment vector
            v2 = points[2] - points[1]  # Second segment vector
            v3 = points[3] - points[2]  # Third segment vector
            
            # Draw arcs for each joint angle
            arc1_points, angle1 = self.draw_angle_arc(points[1], v1, v2)
            arc2_points, angle2 = self.draw_angle_arc(points[2], v2, v3)
            
            # Plot the arcs as 3D lines
            arc1 = self.ax.plot(arc1_points[0], arc1_points[1], arc1_points[2], 
                               'r-', alpha=0.5, linewidth=2)[0]
            arc2 = self.ax.plot(arc2_points[0], arc2_points[1], arc2_points[2], 
                               'r-', alpha=0.5, linewidth=2)[0]
            self.angle_arcs = [arc1, arc2]
            
            # Update angle text
            angle_text = f'θ1: {math.degrees(angles[0]):.1f}°\n' \
                        f'θ2: {math.degrees(angles[1]):.1f}°\n' \
                        f'θ3: {math.degrees(angles[2]):.1f}°'
            self.angle_text.set_text(angle_text)
        
        self.fig.canvas.draw_idle()

