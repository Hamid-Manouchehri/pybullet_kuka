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
    cameraDistance=2.0,    # Zoom level (2 meters away)
    cameraYaw=45,          # 45 degrees rotation around the vertical axis
    cameraPitch=-30,       # Tilt 30 degrees downward
    cameraTargetPosition=[0, 0, 0]  # Focus on the origin (0, 0, 0)
)

# Load the KUKA robot and environment objects
kuka_id = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")[0]
p.loadURDF("tray/tray.urdf", [0.6, 0, 0], [0, 0, 1, 0])
p.loadURDF("block.urdf", [0.45, 0, 0.01], [0, 0, 0, 1])
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


def execute_trajectory(time_step, duration, start_pos, final_pos, start_orientation, final_orientation):

    t = 0
    position_tolerance = 0.01
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
            break

        p.stepSimulation()



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

    p.setJointMotorControlArray(
        kuka_id,indexes,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_angles,
        targetVelocities=[0]*4,
        positionGains=[0.05]*4,
        forces=[50.]*4
    )


time_step = .1  # TODO

start_pos = np.array([0., 0., 1.305])
second_pos = np.array([0.3, 0., 1.])

start_orientation = np.array([0., 0., 0.])
second_orientation = np.array([0., np.pi, 0.])


def main():

    duration = 10
    # execute_trajectory(time_step, duration, start_pos, second_pos, start_orientation, second_orientation)

    set_joint_angles([0., np.pi/6, 0., -np.pi, 0., np.pi/6, np.pi/2])
    control_gripper(gripper_opening_angle=1.)  # 0.: close

    # while True:
    #     # Example: Move to a specific set of joint angles

    #     joints_angles = calculate_ik()

    #     set_joint_angles([0., 1., 0., 1., 0., 0., 0.])

    #     # Control the gripper
    #     control_gripper(gripper_opening_angle=0.)  # Adjust to open/close gripper

    #     # Step the simulation
    #     p.stepSimulation()
    #     time.sleep(1 / 240)

    # print(p.getEulerFromQuaternion(get_current_pose()[1]))

    while True:
        p.stepSimulation()
        time.sleep(1 / 240)

if __name__ == '__main__':
    main()