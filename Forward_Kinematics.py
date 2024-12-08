import numpy as np

def dh_matrix(a, alpha, d, theta):
    """
    Create a Denavit-Hartenberg transformation matrix
    
    Parameters:
    a: link length (mm)
    alpha: link twist angle (degrees)
    d: link offset (mm)
    theta: joint angle (degrees)
    
    Returns:
    4x4 NumPy transformation matrix
    """
    # Convert angles to radians
    alpha_rad = np.deg2rad(alpha)
    theta_rad = np.deg2rad(theta)
    
    # Compute trigonometric values
    cos_theta = np.cos(theta_rad)
    sin_theta = np.sin(theta_rad)
    cos_alpha = np.cos(alpha_rad)
    sin_alpha = np.sin(alpha_rad)
    
    # Create the transformation matrix
    A = np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])
    
    return A

# DH Parameters from the table
# Link 1
a1, alpha1, d1 = 0, 90, 96
# Link 2
a2, alpha2, d2 = -104, 0, 0
# Link 3
a3, alpha3, d3 = -118, 0, 0
# Link 4
a4, alpha4, d4 = 0, -90, -9
# Link 5
a5, alpha5, d5 = 0, 0, 137

# Example usage with sample joint angles
def forward_kinematics(theta1, theta2, theta3, theta4, theta5):
    """
    Compute forward kinematics transformation
    
    Parameters:
    theta1-theta5: Joint angles in degrees
    
    Returns:
    Transformation matrix from base to end effector
    """
    # Adjust joint angles based on DH parameter table
    theta1_adjusted = theta1 + 90
    theta2_adjusted = theta2 - 225
    theta3_adjusted = theta3 - 90
    theta4_adjusted = theta4
    theta5_adjusted = theta5 + 90
    
    # Compute individual link transformation matrices
    A1 = dh_matrix(a1, alpha1, d1, theta1_adjusted)
    A2 = dh_matrix(a2, alpha2, d2, theta2_adjusted)
    A3 = dh_matrix(a3, alpha3, d3, theta3_adjusted)
    A4 = dh_matrix(a4, alpha4, d4, theta4_adjusted)
    A5 = dh_matrix(a5, alpha5, d5, theta5_adjusted)
    
    # Compute total transformation matrix
    T = A1 @ A2 @ A3 @ A4 @ A5
    
    return T

# Demonstration
# Zero position
zero_pose = forward_kinematics(0, 0, 0, 0, 0)
print("Transformation matrix at zero position:")
print(zero_pose)