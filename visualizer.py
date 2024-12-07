import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np
import time

# Define the links using Denavit-Hartenberg parameters
L1 = RevoluteDH(d=96, a=0, alpha=np.pi/2)  # Changed d parameter to 96
L2 = RevoluteDH(d=0, a=-104, alpha=0)
L3 = RevoluteDH(d=0, a=-64, alpha=0)
L4 = RevoluteDH(d=-9, a=0, alpha=-np.pi/2)
L5 = RevoluteDH(d=137, a=0, alpha=0)

# Create the SerialLink robot
KR = DHRobot([L1, L2, L3, L4, L5], name='robot')

# Function to calculate servo angles and joint transformations
def calculate_servo_angles(H_matrix):
    # Initial guess for IK
    q0 = [0, 0, 0, 0, 0]
    
    # Mask to indicate which DOFs to consider
    mask = [1, 1, 1, 1, 1, 0]
    
    # Perform Inverse Kinematics
    q_sol = KR.ikine_LM(H_matrix, q0=q0, mask=mask)
    
    # Check if a solution was found
    if not q_sol.success:
        raise ValueError("Inverse Kinematics solution could not be found.")
    
    # Convert joint angles to degrees and round to 2 decimal places
    q_sol_degrees = np.round(np.degrees(q_sol.q), 0)  # Rounded to nearest integer
    
    # Convert joint angles to servo pulse widths (microseconds)
    servo_angles = q_sol_degrees * 2000 / 180 + 500

    # Calculate specific joint transformations for servo driving
    joint1_angle = 1500 + servo_angles[0] - 500
    joint2_angle = servo_angles[1] + 1800 + 500
    joint3_angle = - servo_angles[2] + 1900 - 500
    joint4_angle = servo_angles[3] + 1600 - 1500
    joint5_angle = 1500 + servo_angles[4] - 500

    # Return results
    return q_sol_degrees, servo_angles, joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle

# Example usage
if __name__ == "__main__":
    # Define the transformation matrix (H) as a predefined matrix
    H_matrix = SE3(np.array([
        [0, 0, 1, 201],
        [0, 1, 0, 9],
        [-1, 0, 0, 220],
        [0, 0, 0, 1]
    ]))  # Replace this matrix with the desired end-effector pose

    # Calculate the servo angles based on the input H matrix
    q_sol_degrees, servo_angles, joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle = calculate_servo_angles(H_matrix)
    
    # Display the results without decimals and with commas
    print("\nResults:")
    print(f"Joint Angles (Degrees): {', '.join(map(str, q_sol_degrees.astype(int)))}")
    print(f"Servo Angles (Microseconds): {', '.join(map(str, servo_angles.astype(int)))}")
    print(f"Joint 1 Servo: {int(joint1_angle)} µs")
    print(f"Joint 2 Servo: {int(joint2_angle)} µs")
    print(f"Joint 3 Servo: {int(joint3_angle)} µs")
    print(f"Joint 4 Servo: {int(joint4_angle)} µs")
    print(f"Joint 5 Servo: {int(joint5_angle)} µs")
    
    final = str(int(joint1_angle)) + ',' + str(int(joint2_angle)) + ',' + str(int(joint3_angle)) + ',' + str(int(joint4_angle)) + ',' + str(int(joint5_angle)) + ',2200'
    
    print(final)
    
    # Plot initial position (using q0 = [0, 0, 0, 0, 0])
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    # Define joint configuration as a row vector
    q = [0, -np.pi/2, -np.pi/2, np.pi/2, 0]
    
    # Plot initial position
    KR.plot(q, block=True)
    axes[0].set_title("Initial Position")


    # Plot final position
    KR.plot(np.radians(q_sol_degrees), block=True)
    axes[1].set_title("Final Position (After IK)")

    plt.tight_layout()
    plt.show()