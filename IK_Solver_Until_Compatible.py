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

# Create the SerialLink robot
KR = DHRobot([L1, L2, L3, L4, L5], name='robot')

H_matrix = SE3(np.array([
    [0, 0, 1, 230],
    [0, 1, 0, 9],
    [-1, 0, 0, 200],
    [0, 0, 0, 1]
]))  # Replace this matrix with the desired end-effector pose

# Randomly generate initial guesses and solve IK
solutions = []
mask = [1, 1, 1, 1, 1, 0]  # Consider 5 DOFs
np.random.seed(42)  # Set random seed for reproducibility
valid_solutions_count = 0  # Counter for valid solutions

while valid_solutions_count < 10000:  # Stop when 10 valid solutions are found
    # Generate a random initial guess within typical joint range (-π to π)
    q0 = np.random.uniform(-2*np.pi, 2*np.pi, size=5)
    # q0 = [0, 0, 0, 0, 0]
    
    # Solve IK
    q_sol = KR.ikine_LM(H_matrix, q0=q0, mask=mask)
    
    if q_sol.success:  # Check if a valid solution was found
        # Convert solution to degrees and servo angles
        joint_angles_deg = np.round(np.degrees(q_sol.q), 2)
        # Convert joint angles to servo pulse widths (microseconds)
        servo_angles = joint_angles_deg * 2000 / 180 + 500

        # Calculate specific joint transformations for servo driving
        joint1_angle = 1500 + servo_angles[0] - 500
        joint2_angle = servo_angles[1] + 1800 + 500
        joint3_angle = - servo_angles[2] + 1900 - 500
        joint4_angle = servo_angles[3] + 1600 - 1500
        joint5_angle = 1500 + servo_angles[4] - 500

        joint_angles_list = [
            joint1_angle,
            joint2_angle,
            joint3_angle,
            joint4_angle,
            joint5_angle,
        ]
        
        print("\nJoint Angles (Degrees):")
        print(joint_angles_list)
        
        # Check if all servo angles are within the range [500, 2500]
        if np.all((np.array(joint_angles_list) >= 500) & (np.array(joint_angles_list) <= 2500)):
            solutions.append(q_sol.q)
            valid_solutions_count += 1  # Increment valid solutions counter
            break

# Convert solutions to degrees and servo ranges
solutions_degrees = np.round(np.degrees(solutions), 2)
print("\nValid Inverse Kinematics Solutions (Joint Angles in Degrees):")
print(solutions_degrees)

# Map joint angles to servo ranges (pulse width in microseconds)
servo_angles_list = []
for solution in solutions_degrees:
    servo_angles = solution * 2000 / 180 + 500
    servo_angles_list.append(servo_angles)

print("\nServo Angles (Pulse Width in Microseconds):")
for i, servo_angles in enumerate(servo_angles_list):
    print(f"Solution {i + 1}: {servo_angles}")

# Calculate specific joint angles based on given transformations
final_joint_angles_list = []
for servo_angles in servo_angles_list:
    # Calculate specific joint transformations for servo driving
    joint1_angle = 1500 + servo_angles[0] - 500
    joint2_angle = servo_angles[1] + 1800 + 500
    joint3_angle = - servo_angles[2] + 1900 - 500
    joint4_angle = servo_angles[3] + 1600 - 1500
    joint5_angle = 1500 + servo_angles[4] - 500

    # Store the calculated joint angles
    final_joint_angles_list.append([
        joint1_angle,
        joint2_angle,
        joint3_angle,
        joint4_angle,
        joint5_angle,
    ])

# Add gripper action (grab or release)
gripper_action = "grab"  # Change this value to "release" for releasing the gripper

# Display all calculated joint angle combinations and send API requests
print("\nCalculated Joint Angles for All Solutions:")
for i, joint_angles in enumerate(final_joint_angles_list):
    print(f"Solution {i + 1}: {joint_angles}")
    
    # Prepare the final servo position string
    final = str(int(joint1_angle)) + ',' + str(int(joint2_angle)) + ',' + str(int(joint3_angle)) + ',' + str(int(joint4_angle)) + ',' + str(int(joint5_angle))

    # Gripper control logic based on the gripper action
    if gripper_action == "grab":
        final += ",2500"  # Send 2500 for gripper grab action
    elif gripper_action == "release":
        final += ",1900"  # Send 1900 for gripper release action
    
    print(final)
    
    # Send the API request
    url = f"http://192.168.1.104/set_positions?pos={final}"
    response = requests.get(url)
    
    # Print the response from the API (for debugging purposes)
    print(f"API Response: {response.status_code}")
    print(f"Response Content: {response.text}")