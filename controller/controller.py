# Here’s the controller code,(Both solver and UI)(Replace the IP address)

from flask import Flask, render_template, request
import numpy as np
import requests
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

# Initialize Flask app
app = Flask(__name__)

# Define the links using Denavit-Hartenberg parameters
L1 = RevoluteDH(d=96, a=0, alpha=np.pi/2)
L2 = RevoluteDH(d=0, a=-104, alpha=0)
L3 = RevoluteDH(d=0, a=-64, alpha=0)
L4 = RevoluteDH(d=-9, a=0, alpha=-np.pi/2)
L5 = RevoluteDH(d=137, a=0, alpha=0)

# Create the SerialLink robot
KR = DHRobot([L1, L2, L3, L4, L5], name='robot')

# Define positions for the boxes and destinations
GreenboxPos = np.array([
    [0, 0, 1, 250],   # Location 1
    [0, 1, 0, 0],
    [-1, 0, 0, 50],
    [0, 0, 0, 1]
])

BlueboxPos = np.array([
    [0, 0, 1, 250],
    [0, 1, 0, -100],
    [-1, 0, 0, 50],
    [0, 0, 0, 1]
])

RedboxPos = np.array([
    [0, 0, 1, 150],
    [0, 1, 0, -200],
    [-1, 0, 0, 60],
    [0, 0, 0, 1]
])

# Additional destination positions
firstDestinationPos = np.array([
    [0, 0, 1, 200],
    [0, 1, 0, 150],
    [-1, 0, 0, 50],
    [0, 0, 0, 1]
])

secondDestinationPos = np.array([
    [0, 0, 1, 250],
    [0, 1, 0, 100],
    [-1, 0, 0, 50],
    [0, 0, 0, 1]
])

thirdDestinationPos = np.array([
    [0, 0, 1, 150],
    [0, 1, 0, 200],
    [-1, 0, 0, 50],
    [0, 0, 0, 1]
])

freePos = np.array([
    [0, 0, 1, 250],
    [0, 1, 0, 0],
    [-1, 0, 0, 40],
    [0, 0, 0, 1]
])

# Route to render the HTML page
@app.route('/')
def index():
    return render_template('index.html')

# Route to handle the arm movement
@app.route('/move', methods=['POST'])
def move_arm():
    # Get the selected values from the HTML form
    cube_color = request.form['color']
    destination = request.form['destination']

    # Map the color to the corresponding box position
    if cube_color == "Green":
        box_pos = GreenboxPos
    elif cube_color == "Blue":
        box_pos = BlueboxPos
    elif cube_color == "Red":
        box_pos = RedboxPos

    # Map the destination to the corresponding destination position
    if destination == "First":
        destination_pos = firstDestinationPos
    elif destination == "Second":
        destination_pos = secondDestinationPos
    elif destination == "Third":
        destination_pos = thirdDestinationPos

    # Adjust the Z-component for both box and destination
    boxMovedPos = box_pos.copy()
    destinationMovedPos = destination_pos.copy()
    boxMovedPos[0:3, 3] += np.array([0, 0, 105])  # Add 105 to the Z-component
    destinationMovedPos[0:3, 3] += np.array([0, 0, 105])  # Add 105 to the Z-component

    # Define the target locations and corresponding arm actions
    locations = [
        SE3(boxMovedPos),
        SE3(box_pos),
        SE3(boxMovedPos),
        SE3(destinationMovedPos),
        SE3(destination_pos),
        SE3(destinationMovedPos),
        SE3(freePos),
    ]

    locations = [locations[0], locations[1], locations[1], locations[2], locations[3], locations[4], locations[4], locations[5], locations[6]]
    arm_actions = ["release", "release", "grab", "grab", "grab", "grab", "release", "release", "release"]

    # Mask to consider only 5 DOFs
    mask = [1, 1, 1, 1, 1, 0]
    np.random.seed(42)

    for i, (H_matrix, action) in enumerate(zip(locations, arm_actions)):
        print(f"\nProcessing Location {i + 1} with Action: {action}")

        solutions = []
        valid_solutions_count = 0

        while valid_solutions_count < 100:
            q0 = np.random.uniform(-2*np.pi, 2*np.pi, size=5)
            q_sol = KR.ikine_LM(H_matrix, q0=q0, mask=mask)

            if q_sol.success:
                joint_angles_deg = np.round(np.degrees(q_sol.q), 2)
                servo_angles = joint_angles_deg * 2000 / 180 + 500

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

                if np.all((np.array(joint_angles_list) >= 500) & (np.array(joint_angles_list) <= 2500)):
                    solutions.append(q_sol.q)
                    valid_solutions_count += 1
                    break

        solutions_degrees = np.round(np.degrees(solutions), 2)
        servo_angles_list = []
        for solution in solutions_degrees:
            servo_angles = solution * 2000 / 180 + 500
            servo_angles_list.append(servo_angles)

        final_joint_angles_list = []
        for servo_angles in servo_angles_list:
            joint1_angle = 1560 + servo_angles[0] - 500
            joint2_angle = servo_angles[1] + 1800 + 500
            joint3_angle = - servo_angles[2] + 1900 - 500
            joint4_angle = servo_angles[3] + 1250 - 1500
            joint5_angle = 1500 + servo_angles[4] - 500

            final_joint_angles_list.append([
                joint1_angle,
                joint2_angle,
                joint3_angle,
                joint4_angle,
                joint5_angle,
            ])

        for joint_angles in final_joint_angles_list:
            final = str(int(joint1_angle)) + ',' + str(int(joint2_angle)) + ',' + str(int(joint3_angle)) + ',' + str(int(joint4_angle)) + ',' + str(int(joint5_angle))

            if action == "grab":
                final += ",2500"
            elif action == "release":
                final += ",1900"

            print(f"Sending position: {final}")
            url = f"http://192.168.1.104/set_positions?pos={final}"
            response = requests.get(url)
            if response.status_code == 200:
                print(f"Success: Arm moved to {action} position.")
            else:
                print(f"Error: Failed to move arm. Status Code: {response.status_code}")

    return render_template('index.html', success=True)

# Correct
if __name__ == '__main__':
    app.run(debug=True)