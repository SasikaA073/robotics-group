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

GreenboxPos = np.array([
        [0, 0, 1, 250],   # Location 1
        [0, 1, 0, 0],
        [-1, 0, 0, 50],
        [0, 0, 0, 1]
    ])

BlueboxPos = np.array([
        [0, 0, 1, 250],   # Location 1
        [0, 1, 0, -100],
        [-1, 0, 0, 50],
        [0, 0, 0, 1]
    ])

RedboxPos = np.array([
        [0, 0, 1, 150],   # Location 1
        [0, 1, 0, -200],
        [-1, 0, 0, 60],
        [0, 0, 0, 1]
    ])

firstDestinationPos = np.array([
        [0, 0, 1, 200],   # Location 1
        [0, 1, 0, 150],
        [-1, 0, 0, 50],
        [0, 0, 0, 1]
    ])

secondDestinationPos = np.array([
        [0, 0, 1, 250],   # Location 1
        [0, 1, 0, 100],
        [-1, 0, 0, 50],
        [0, 0, 0, 1]
    ])

thirdDestinationPos = np.array([
        [0, 0, 1, 150],   # Location 4
        [0, 1, 0, 200],
        [-1, 0, 0, 50],
        [0, 0, 0, 1]
    ])

freePos = np.array([
        [0, 0, 1, 250],   # Location 4
        [0, 1, 0, 0],
        [-1, 0, 0, 40],
        [0, 0, 0, 1]
    ])

pointsChosen = ["Green", "Second"]

if pointsChosen[0] == "Green":
    boxPos = GreenboxPos
elif pointsChosen[0] == "Blue":
    boxPos = BlueboxPos
elif pointsChosen[0] == "Red":
    boxPos = RedboxPos
    
if pointsChosen[1] == "First":
    destinationPos = firstDestinationPos
elif pointsChosen[1] == "Second":
    destinationPos = secondDestinationPos
elif pointsChosen[1] == "Third":
    destinationPos = thirdDestinationPos

boxMovedPos = boxPos.copy()
destinationMovedPos = destinationPos.copy()

boxMovedPos[0:3, 3] += np.array([0, 0, 105])  # Add 105 to the Z-component
destinationMovedPos[0:3, 3] += np.array([0, 0, 105])  # Add 105 to the Z-component

# Define the target locations and corresponding arm actions
locations = [
    SE3(boxMovedPos),
    SE3(boxPos),
    SE3(boxMovedPos),
    SE3(destinationMovedPos),
    SE3(destinationPos),
    SE3(destinationMovedPos),
    SE3(freePos),
]
 
locations = [locations[0], locations[1], locations[1], locations[2], locations[3], locations[4], locations[4], locations[5], locations[6]]

arm_actions = ["release","release", "grab", "grab", "grab", "grab","release","release","release"]  # Define different arm actions for each location

# Mask to consider only 5 DOFs
mask = [1, 1, 1, 1, 1, 0]
np.random.seed(42)  # Set random seed for reproducibility

# Loop through each location and perform actions
for i, (H_matrix, action) in enumerate(zip(locations, arm_actions)):
    print(f"\nProcessing Location {i + 1} with Action: {action}")
    
    # Solve IK for the current location
    solutions = []
    valid_solutions_count = 0  # Counter for valid solutions
    
    while valid_solutions_count < 100:  # Stop when 10 valid solutions are found
        # Generate a random initial guess within typical joint range (-π to π)
        q0 = np.random.uniform(-2*np.pi, 2*np.pi, size=5)
        
        # Solve IK
        q_sol = KR.ikine_LM(H_matrix, q0=q0, mask=mask)
        
        if q_sol.success:  # Check if a valid solution was found
            # Convert solution to degrees and servo angles
            joint_angles_deg = np.round(np.degrees(q_sol.q), 2)
            # Convert joint angles to servo pulse widths (microseconds)
            servo_angles = joint_angles_deg * 2000 / 180 + 500

            # Calculate specific joint transformations for servo driving
            joint1_angle = 1570 + servo_angles[0] - 500
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
            
            # Check if all servo angles are within the range [500, 2500]
            if np.all((np.array(joint_angles_list) >= 500) & (np.array(joint_angles_list) <= 2500)):
                solutions.append(q_sol.q)
                valid_solutions_count += 1  # Increment valid solutions counter
                break
    
    # Convert solutions to degrees and servo ranges
    solutions_degrees = np.round(np.degrees(solutions), 2)
    
    # Map joint angles to servo ranges (pulse width in microseconds)
    servo_angles_list = []
    for solution in solutions_degrees:
        servo_angles = solution * 2000 / 180 + 500
        servo_angles_list.append(servo_angles)
    
    # Calculate specific joint angles based on given transformations
    final_joint_angles_list = []
    for servo_angles in servo_angles_list:
        # Calculate specific joint transformations for servo driving
        joint1_angle = 1560 + servo_angles[0] - 500
        joint2_angle = servo_angles[1] + 1800 + 500
        joint3_angle = - servo_angles[2] + 1900 - 500
        joint4_angle = servo_angles[3] + 1250 - 1500
        joint5_angle = 1500 + servo_angles[4] - 500

        # Store the calculated joint angles
        final_joint_angles_list.append([
            joint1_angle,
            joint2_angle,
            joint3_angle,
            joint4_angle,
            joint5_angle,
        ])
    
    # Display all calculated joint angle combinations and send API requests
    print("\nCalculated Joint Angles for Solution(s):")
    for joint_angles in final_joint_angles_list:
        print(f"Joint Angles: {joint_angles}")
        
        # Prepare the final servo position string
        final = str(int(joint1_angle)) + ',' + str(int(joint2_angle)) + ',' + str(int(joint3_angle)) + ',' + str(int(joint4_angle)) + ',' + str(int(joint5_angle))

        # Gripper control logic based on the action
        if action == "grab":
            final += ",2400"  # Send 2500 for gripper grab action
        elif action == "release":
            final += ",1900"  # Send 1900 for gripper release action
        
        # Send the API request
        print(f"Sending position: {final}")
        url = f"http://192.168.1.104/set_positions?pos={final}"
        response = requests.get(url)
        
        # Print the response from the API (for debugging purposes)
        print(f"API Response: {response.status_code}")
        print(f"Response Content: {response.text}")