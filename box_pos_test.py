from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np
import requests

# Define the links using Denavit-Hartenberg parameters
L1 = RevoluteDH(d=96, a=0, alpha=np.pi/2)  # Changed d parameter to 96
L2 = RevoluteDH(d=0, a=-104, alpha=0)
L3 = RevoluteDH(d=0, a=-64, alpha=0)
L4 = RevoluteDH(d=-9, a=0, alpha=-np.pi/2)
L5 = RevoluteDH(d=137, a=0, alpha=0)

KR = DHRobot([L1, L2, L3, L4, L5], name='robot')

# Mask to consider only 5 DOFs
mask = [1, 1, 1, 1, 1, 0]
np.random.seed(42)  # For reproducibility

# Output file for storing solutions
output_file = "ik_solutions.txt"

# Infinite loop to find valid IK solutions
attempt_count = 0

with open(output_file, "w") as file:
    file.write("Valid IK Solutions:\n")  # Header for the file

while True:
    attempt_count += 1
    
    # Generate random x and y coordinates for the Bluebox position
    random_x = np.random.uniform(-300, 300)  # Adjust range as needed
    random_y = np.random.uniform(-300, 300)  # Adjust range as needed
    random_z = np.random.uniform(0, 300)  # Adjust range as needed
    
    # Update BlueboxPos matrix
    BlueboxPos = np.array([
        [np.cos(np.pi/2), 0, np.sin(np.pi/2), random_x],
        [0, 1, 0, random_y],
        [-np.sin(np.pi/2), 0, np.cos(np.pi/2), 50],
        [0, 0, 0, 1]
    ])
    target_location = SE3(BlueboxPos)

    print(f"Attempt {attempt_count}: Trying to solve IK for BlueboxPos at x={random_x:.2f}, y={random_y:.2f}...")
    
    # Generate a random initial guess within the joint range
    q0 = np.random.uniform(-2 * np.pi, 2 * np.pi, size=5)
    
    # Solve the IK using Levenberg-Marquardt (or any preferred method)
    q_sol = KR.ikine_LM(target_location, q0=q0, mask=mask)
    
    if q_sol.success:
        # Convert joint angles to degrees and servo pulse widths
        joint_angles_deg = np.round(np.degrees(q_sol.q), 2)
        servo_angles = joint_angles_deg * 2000 / 180 + 500
        
        # Calculate specific joint transformations for servo driving
        joint1_angle = 1500 + servo_angles[0] - 500
        joint2_angle = servo_angles[1] + 1800 + 500
        joint3_angle = - servo_angles[2] + 1900 - 500
        joint4_angle = servo_angles[3] + 1250 - 1500
        joint5_angle = 1500 + servo_angles[4] - 500

        joint_angles_list = [
            joint1_angle,
            joint2_angle,
            joint3_angle,
            joint4_angle,
            joint5_angle,
        ]

        # Ensure all servo angles are within [500, 2500]
        if np.all((np.array(joint_angles_list) >= 500) & (np.array(joint_angles_list) <= 2500)):
            print(f"Valid solution found on attempt {attempt_count}!")

            # Prepare the servo position string
            final = (
                f"{int(joint1_angle)},{int(joint2_angle)},{int(joint3_angle)},"
                f"{int(joint4_angle)},{int(joint5_angle)},1900"  # 1900 for release action
            )

            # Save the solution to the text file
            with open(output_file, "a") as file:
                file.write(f"Blue Matrix {BlueboxPos}:\n")
                file.write(f"Attempt {attempt_count}:\n")
                file.write(f"Joint Angles (Degrees): {joint_angles_deg.tolist()}\n")
                file.write(f"Servo Angles (Pulse Widths): {joint_angles_list}\n")
                file.write(f"Servo Command: {final}\n")
                file.write(f"Target Position: x={random_x:.2f}, y={random_y:.2f}\n\n")

            # Log to console
            print(f"Solution saved to {output_file}")
        else:
            pass
            # print("Solution found but joint angles are out of range.")
    else:
        pass
        # print("No valid solution found. Retrying...")