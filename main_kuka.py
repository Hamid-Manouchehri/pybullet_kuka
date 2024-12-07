import pybullet as p
import pybullet_data
import time
import numpy as np
from attrdict import AttrDict

# Connect to PyBullet and set up the environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera( # Set the camera view
    cameraDistance=1.0,    # Zoom level (2 meters away)
    cameraYaw=75,          # 45 degrees rotation around the vertical axis
    cameraPitch=-40,       # Tilt 30 degrees downward
    cameraTargetPosition=[0, 0, 0]  # Focus on the origin (0, 0, 0)
)

# Load the KUKA robot and environment objects
kuka_id = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")[0]
p.loadURDF("tray/tray.urdf", [0.6, 0, 0], [0, 0, 1, 0])
cuboid_blue_id = p.loadURDF("block.urdf", [0.45, 0, 0.01], [0, 0, 0, 1])
eff_index = 7  # TODO

# Initialize the joints dictionary
joints = AttrDict()

# Populate the joints dictionary with information about each joint
for joint_index in range(p.getNumJoints(kuka_id)):
    joint_info = p.getJointInfo(kuka_id, joint_index)
    joint_name = joint_info[1].decode("utf-8")
    joints[joint_name] = AttrDict({
        "id": joint_info[0],
        "lowerLimit": joint_info[8],
        "upperLimit": joint_info[9],
        "maxForce": joint_info[10],
        "maxVelocity": joint_info[11],
    })

def calculate_ik(position, orientation):
        quaternion = p.getQuaternionFromEuler(orientation)
        # print(quaternion)
        # quaternion = (0,1,0,1)
        lower_limits = [-np.pi]*6
        upper_limits = [np.pi]*6
        joint_ranges = [2*np.pi]*6
        rest_poses = [(-0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0)] # rest pose of our ur5 robot

        joint_angles = p.calculateInverseKinematics(
            kuka_id, eff_index, position, quaternion, 
            jointDamping=[0.01]*11 , upperLimits=upper_limits, 
            lowerLimits=lower_limits, jointRanges=joint_ranges, 
            restPoses=rest_poses
        )
        return joint_angles

# Function to set joint angles
def set_joint_angles(joint_angles):
    control_joints = ["J0", "J1", "J2", "J3", "J4", "J5", "J6"]
    poses = []
    indexes = []
    forces = []

    for i, name in enumerate(control_joints):
        joint = joints[name]
        poses.append(joint_angles[i])
        indexes.append(joint.id)
        forces.append(joint.maxForce)

    p.setJointMotorControlArray(
        kuka_id, indexes,
        p.POSITION_CONTROL,
        targetPositions=poses,
        targetVelocities=[0] * len(poses),
        positionGains=[0.05] * len(poses),
        forces=forces
    )

def position_path(t, t_max, start_pos, end_pos):

    return start_pos + (end_pos - start_pos) * (t / t_max)



def orientation_path(t, t_max, start_orient, end_orient):
    """ orientation is in Euler. """

    return start_orient + (end_orient - start_orient) * (t / t_max)


def get_current_pose():
    linkstate = p.getLinkState(kuka_id, eff_index, computeForwardKinematics=True)
    position, orientation = linkstate[0], linkstate[1]
    return (position, orientation)


def get_joint_angles():
    "revolute joints: kuka + gripper"
    j = p.getJointStates(kuka_id, [0, 1, 2, 3, 4, 5, 6, 8, 10, 11, 13])
    joints = [i[0] for i in j]
    return joints


def execute_task_space_trajectory(time_step, duration, start_pos, final_pos, start_orientation, final_orientation):

    t = 0
    position_tolerance = 0.005
    trailDuration = 15
    prevPose=[0,0,0]
    prevPose1=[0,0,0]
    hasPrevPose = 0

    while True:

        position_1 = position_path(t, duration, start_pos, final_pos)
        orientation_1 = orientation_path(t, duration, start_orientation, final_orientation)
        
        all_joint_angles = calculate_ik(position_1, orientation_1)
        ur5_joint_angles = all_joint_angles[:7]  # Use the first 6 joints for UR5

        set_joint_angles(ur5_joint_angles)

        ls = p.getLinkState(kuka_id, eff_index)	
        if (hasPrevPose):
            p.addUserDebugLine(prevPose, position_1, [0,0,0.3], 1, trailDuration)
            p.addUserDebugLine(prevPose1, ls[4], [1,0,0], 1, trailDuration)
        prevPose=position_1
        prevPose1=ls[4]
        hasPrevPose = 1	

        t += time_step

        if np.linalg.norm(np.array(get_current_pose()[0]) - np.array(final_pos)) < position_tolerance:
            print("End-effector reached the target position.")
            print(f"Current position: {get_current_pose()[0]}, Target position: {final_orientation}")
            print("\n\n\ntrajectory completed!\n\n\n")
            control_gripper(gripper_opening_angle=0.1)  # 0.: close
            break

        p.stepSimulation()


def execute_gripper(base_tip_joints_angle):

    joint_angle_threshold = 0.01  # TODO

    control_gripper(base_tip_joints_angle)

    while True:

        left_base_tip_joint = get_joint_angles()[8]
        right_base_tip_joint = get_joint_angles()[10]

        if np.linalg.norm(abs(np.array([left_base_tip_joint, right_base_tip_joint])) - 
        abs(np.array([base_tip_joints_angle, base_tip_joints_angle]))) < joint_angle_threshold:
            print("\ngripper's fingers are set to desired position!\n")
            break

        p.stepSimulation()



def wait_for_reaching_joint_angles(desired_joint_angles, tolerance=0.01):
    while True:
        current_joint_angles = np.array(get_joint_angles()[:7])
        error = np.abs(current_joint_angles - desired_joint_angles)
        
        if np.all(error < tolerance):
            print("\nRobot reached desired joint angles.\n")
            break
        
        # print(f"Error: {error}")  # Optional: Debugging information
        p.stepSimulation()
        time.sleep(1 / 240)



# Function to control the gripper
def control_gripper(gripper_opening_angle):

    poses = []
    indexes = []
    forces = []

    gripper_joints = ["base_left_finger_joint", "base_right_finger_joint", "right_base_tip_joint", "left_base_tip_joint"]
    joint_angles = np.array([0., 0., gripper_opening_angle, -gripper_opening_angle])

    for i, name in enumerate(gripper_joints):
        joint = joints[name]
        # print("\n\n\n\n\n", i, "    ", joint)
        poses.append(joint_angles[i])
        indexes.append(joint.id)
        forces.append(joint.maxForce)

    # print("joint angles: ", joint_angles, "indixes: ", indexes)

    p.setJointMotorControlArray(
        kuka_id,indexes,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_angles,
        targetVelocities=[0]*4,
        positionGains=[0.05]*4,
        forces=[50.]*4
    )


def is_object_grasped(gripper_id, object_id):
    
    contact_points = p.getContactPoints(bodyA=gripper_id, bodyB=object_id)
    return len(contact_points) > 0


time_step = .1  # TODO

start_pos = np.array([0., 0., 1.305])  # home position, up-right
second_pos = np.array([0.41, 0., 0.48])  # on top of the object with distance in z-axis
third_pos = np.array([0.41, 0., 0.27])  # right on top of the object

start_orientation = np.array([0., 0., 0.])
second_orientation = np.array([np.pi, 0., np.pi/2])
third_orientation = np.array([np.pi, 0., np.pi/2])


def main():

    des_kuka_joint_angle = np.array([0., 0.114, 0., -1.895, 0., 1.13, np.pi/2])  # on top of the object with distance in z-axis
    # des_kuka_joint_angle = np.array([0., np.pi/6, 0., -2.094, 0., np.pi/6, np.pi/2])  # right on top of the object 

    set_joint_angles(des_kuka_joint_angle)
    wait_for_reaching_joint_angles(des_kuka_joint_angle)
    execute_gripper(.5)  # 0.: close

    duration = 20
    execute_task_space_trajectory(time_step, duration, second_pos, third_pos, third_orientation, third_orientation)
    execute_gripper(0.1)  # 0.: close
    execute_task_space_trajectory(time_step, duration, third_pos, second_pos, third_orientation, third_orientation)


    while True:
        # print(get_joint_angles()[:7])  # Kuka joint angles
        p.stepSimulation()
        time.sleep(1 / 240)



if __name__ == '__main__':
    main()