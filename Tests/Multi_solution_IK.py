from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np

# Define the links using Denavit-Hartenberg parameters
L1 = RevoluteDH(d=96, a=0, alpha=np.pi/2)  # Changed d parameter to 96
L2 = RevoluteDH(d=0, a=-104, alpha=0)
L3 = RevoluteDH(d=0, a=-118, alpha=0)
L4 = RevoluteDH(d=-9, a=0, alpha=-np.pi/2)
L5 = RevoluteDH(d=137, a=0, alpha=0)

# Create the SerialLink robot
KR = DHRobot([L1, L2, L3, L4, L5], name='robot')

# Define joint configuration as a row vector
q = [0, -np.pi/2, -np.pi/2, np.pi/2, 0]

# Forward Kinematics
qf = [0, -np.pi/2, -np.pi/2, np.pi/2, 0]
TR = KR.fkine(qf)
print("Forward Kinematics (End-Effector Transformation Matrix):")
print(TR)

# Inverse Kinematics
q0_list = [  # Initial guesses for IK
    [0, 0, 0, 0, 0],
    [np.pi/4, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4],
    [-np.pi/4, np.pi/4, -np.pi/4, np.pi/4, -np.pi/4],
    [np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2],
    [-np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2],
    [np.pi, 0, np.pi, 0, np.pi],
    [0, -np.pi, 0, np.pi, 0],
    [-np.pi, 0, -np.pi, 0, -np.pi],
    [np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2],
    [-np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2]
]

solutions = []
mask = [1, 1, 1, 1, 1, 0]  # Consider 5 DOFs

for q0 in q0_list:
    q_sol = KR.ikine_LM(TR, q0=q0, mask=mask)  # Solve IK
    if q_sol.success:  # Check if a valid solution was found
        solutions.append(q_sol.q)

# Remove duplicates (if any)
unique_solutions = np.unique(solutions, axis=0)

print("\n10 Possible Inverse Kinematics Solutions (Joint Angles):")
for i, sol in enumerate(unique_solutions[:10]):  # Limit to 10 solutions
    print(f"Solution {i + 1}: {sol}")
    
# Convert radians to degrees and round to 2 decimal places
solutions_degrees = np.round(np.degrees(unique_solutions), 2)
print("\nInverse Kinematics Solutions (Joint Angles in Degrees):")
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
joint_angles_list = []
for servo_angles in servo_angles_list:
    joint1_angle = 1500 - servo_angles[0] + 500
    joint2_angle = servo_angles[1] + 1800 + 500
    joint3_angle = servo_angles[2] + 2500 + 500
    joint4_angle = servo_angles[3] + 1600 - 1500
    joint5_angle = 1500 - servo_angles[4] + 500

    # Store the calculated joint angles
    joint_angles_list.append([
        joint1_angle,
        joint2_angle,
        joint3_angle,
        joint4_angle,
        joint5_angle,
    ])

# Display all calculated joint angle combinations
print("\nCalculated Joint Angles for All Solutions:")
for i, joint_angles in enumerate(joint_angles_list):
    print(f"Solution {i + 1}: {joint_angles}")