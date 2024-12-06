import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet and set up the environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Load the Kuka robot and environment objects
robot_id = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")[0]
p.loadURDF("tray/tray.urdf", [.5, 0, 0], [0, 0, 1, 0])
p.loadURDF("block.urdf", [0.4, 0, 0.01], [0, 0, 0, 1])

# Get the number of joints in the Kuka robot
num_joints = p.getNumJoints(robot_id)
print(f"Kuka Robot has {num_joints} joints.")

# Create sliders for joint control
joint_sliders = []
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint_index)
    joint_name = joint_info[1].decode("utf-8")
    slider = p.addUserDebugParameter(
        joint_name, 
        joint_info[8],  # Lower limit
        joint_info[9],  # Upper limit
        0.0             # Initial value
    )
    joint_sliders.append(slider)

# Function to control the gripper
def control_gripper(gripper_opening_angle):
    """Control the Kuka gripper's opening."""
    # Gripper joint indices in Kuka URDF
    gripper_joints = [7, 8]  # Adjust these indices based on your URDF

    for joint_index in gripper_joints:
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=gripper_opening_angle,
            force=50  # Maximum force
        )

# Main simulation loop
while True:
    # Read the slider values and set joint positions
    for joint_index in range(num_joints):
        target_angle = p.readUserDebugParameter(joint_sliders[joint_index])
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=200  # Maximum torque/force
        )

    # Example: Use a fixed gripper position (open or close)
    control_gripper(gripper_opening_angle=1.)  # 0.04 radians for open position

    # Step the simulation
    p.stepSimulation()
    time.sleep(1 / 240)
