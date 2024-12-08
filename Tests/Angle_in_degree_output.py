from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np

# Define the links using Denavit-Hartenberg parameters
L1 = RevoluteDH(d=96, a=0, alpha=np.pi/2)  # Changed d parameter to 96
L2 = RevoluteDH(d=0, a=-104, alpha=0)
L3 = RevoluteDH(d=0, a=-64, alpha=0)
L4 = RevoluteDH(d=-9, a=0, alpha=-np.pi/2)
L5 = RevoluteDH(d=137, a=0, alpha=0)

# Create the SerialLink robot
KR = DHRobot([L1, L2, L3, L4, L5], name='robot')

# Define joint configuration as a row vector
q = [0, -np.pi/2, -np.pi/2, np.pi/2, 0]

# Plot the robot
KR.plot(q, block=True)

# Forward Kinematics
qf = [0, -np.pi/2, -np.pi/2, np.pi/2, 0]
TR = KR.fkine(qf)
print("Forward Kinematics (End-Effector Transformation Matrix):")
print(TR)

# Inverse Kinematics
q0 = [0, 0, 0, 0, 0]  # Initial guess for IK
# Mask to indicate which DOFs to consider (similar to MATLAB's 'mask' argument)
mask = [1, 1, 1, 1, 1, 0]
TR_target = TR  # Target transformation matrix
q_sol = KR.ikine_LM(TR_target, q0=q0, mask=mask)

print("Inverse Kinematics Solution (Joint Angles):")
print(q_sol.q)

# Print Forward Kinematics result
print("Forward Kinematics (End-Effector Transformation Matrix):")
print(TR)

# Perform Inverse Kinematics
q_sol = KR.ikine_LM(TR_target, q0=q0, mask=mask)

# Convert joint angles to degrees and round off to 2 decimal places
q_sol_degrees = np.round(np.degrees(q_sol.q), 2)

# Print IK solution in degrees
print("Inverse Kinematics Solution (Joint Angles in Degrees, Rounded to 2 Decimals):")
print(q_sol_degrees)

# Convert joint angles for specific servo ranges
servo_angles = q_sol_degrees * 2000 / 180 + 500
print("Servo Angles (Pulse Width in Microseconds):")
print(servo_angles)

# Calculate specific joint angles based on given transformations
joint1_angle = 1500 - servo_angles[0] +500
joint2_angle = servo_angles[1] + 1800 +500
joint3_angle = servo_angles[2] + 2500 +500
joint4_angle = servo_angles[3] + 1600 -1500
joint5_angle = 1500 - servo_angles[4] + 500

# Display the calculated joint angles
print("\nCalculated Joint Angles:")
print(f"Joint 1 Angle: {joint1_angle}°")
print(f"Joint 2 Angle: {joint2_angle}°")
print(f"Joint 3 Angle: {joint3_angle}°")
print(f"Joint 4 Angle: {joint4_angle}°")
print(f"Joint 5 Angle: {joint5_angle}°")