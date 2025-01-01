import numpy as np
import platform
from serial_communication import SerialCommunicator

class HexapodController:
    def __init__(self):
        # Default leg lengths for each leg group
        self.leg_lengths = {
            'left_front': {'S1': 32.25, 'S2': 44, 'S3': 69.5},
            'left_center': {'S1': 32.25, 'S2': 44, 'S3': 69.5},
            'left_back': {'S1': 32.25, 'S2': 44, 'S3': 69.5},
            'right_front': {'S1': 32.25, 'S2': 44, 'S3': 69.5},
            'right_center': {'S1': 32.25, 'S2': 44, 'S3': 69.5},
            'right_back': {'S1': 32.25, 'S2': 44, 'S3': 69.5}
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
        
        # Initialize offsets for each leg group
        self.offsets = {
            'left_front': [0, 0, 0],
            'left_center': [0, 0, 0],
            'left_back': [0, 0, 0],
            'right_front': [0, 0, 0],
            'right_center': [0, 0, 0],
            'right_back': [0, 0, 0]
        }

    def set_leg_lengths(self, leg_group, lengths):
        """Set lengths for a specific leg.
        
        Args:
            leg_group (str): The leg group to set lengths for
            lengths (list): List of [S1, S2, S3] lengths
        """
        try:
            if leg_group in self.leg_lengths:
                self.leg_lengths[leg_group]['S1'] = lengths[0]
                self.leg_lengths[leg_group]['S2'] = lengths[1]
                self.leg_lengths[leg_group]['S3'] = lengths[2]
                print(f"Set lengths for {leg_group}: S1={lengths[0]}, S2={lengths[1]}, S3={lengths[2]}")
            else:
                print(f"Invalid leg group: {leg_group}")
        except Exception as e:
            print(f"Error setting leg lengths: {e}")

    def set_all_leg_lengths(self, lengths):
        """Set the same lengths for all legs.
        
        Args:
            lengths (list): List of [S1, S2, S3] lengths
        """
        try:
            for leg_group in self.leg_lengths:
                self.set_leg_lengths(leg_group, lengths)
        except Exception as e:
            print(f"Error setting all leg lengths: {e}")

    def calculate_inverse_kinematics(self, target, leg_group, is_right_side=False):
        """Calculate joint angles for given target point."""
        try:
            x, y, z = target
            L1 = self.leg_lengths[leg_group]['S1']
            L2 = self.leg_lengths[leg_group]['S2']
            L3 = self.leg_lengths[leg_group]['S3']
            
            # Adjust calculations for right side legs
            if is_right_side:
                y = -y  # Mirror Y coordinates for right side

            theta1 = np.arctan2(y, x)
            r = np.sqrt(x**2 + y**2)
            d = np.sqrt((r - L1)**2 + z**2)

            if d > L2 + L3:
                print(f"Target out of reach for {leg_group}: distance {d:.2f} exceeds maximum reach {L2 + L3:.2f}")
                return None

            # Check if the target is reachable
            cos_alpha = (L2**2 + d**2 - L3**2) / (2 * L2 * d)
            if abs(cos_alpha) > 1:
                print(f"Target unreachable for {leg_group}: cos_alpha = {cos_alpha}")
                return None
            alpha = np.arccos(cos_alpha)
            
            beta = np.arctan2(z, r - L1)
            theta2 = beta + alpha

            cos_gamma = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
            if abs(cos_gamma) > 1:
                print(f"Target unreachable for {leg_group}: cos_gamma = {cos_gamma}")
                return None
            gamma = np.arccos(cos_gamma)
            theta3 = gamma - np.pi

            # Convert angles to degrees
            angles = np.degrees([theta1, theta2, theta3])
            
            # Check for NaN values
            if np.any(np.isnan(angles)):
                print(f"Error: NaN values in angle calculations for {leg_group}")
                return None
                
            print(f"{leg_group} theta values: θ1={angles[0]:.2f}°, θ2={angles[1]:.2f}°, θ3={angles[2]:.2f}°")
            return angles
        except Exception as e:
            print(f"IK calculation error for {leg_group}: {e}")
            return None

    def add_angle_offsets(self, leg_group, offsets):
        """Add offsets to a specific leg group's angles."""
        try:
            if leg_group in self.offsets:
                self.offsets[leg_group] = offsets
            else:
                print(f"Invalid leg group: {leg_group}")
        except Exception as e:
            print(f"Error adding angle offsets: {e}")

    def move_leg(self, leg_group, target):
        """Calculate angles for a specific leg target position."""
        try:
            if leg_group not in self.motor_groups:
                print(f"Invalid leg group: {leg_group}")
                return None

            is_right_side = leg_group.startswith('right_')
            angles = self.calculate_inverse_kinematics(target, leg_group, is_right_side)
            
            if angles is not None and not np.any(np.isnan(angles)):
                # Add offsets to angles
                adjusted_angles = angles + np.array(self.offsets[leg_group])
                return adjusted_angles
            return None
            
        except Exception as e:
            print(f"Error moving leg: {e}")
            return None

    def move_all_legs(self, targets):
        """Calculate angles for all leg target positions.
        
        Args:
            targets: Dictionary with leg group names as keys and target positions as values
                    e.g., {'left_front': [x, y, z], 'right_front': [x, y, z], ...}
        """
        results = {}
        try:
            for leg_group, target in targets.items():
                print(f"\nMoving {leg_group}:")
                angles = self.move_leg(leg_group, target)
                if angles is not None:
                    results[leg_group] = angles
            return results
        except Exception as e:
            print(f"Error moving all legs: {e}")
            return None

