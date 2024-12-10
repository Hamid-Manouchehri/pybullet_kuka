import pybullet as p
import pybullet_data
import time
import numpy as np
from attrdict import AttrDict

# Connect to PyBullet and set up the environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Hides the PyBullet GUI

p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera( # Set the camera view
    cameraDistance=1.0,    # Zoom level (2 meters away)
    cameraYaw=75,          # 45 degrees rotation around the vertical axis
    cameraPitch=-40,       # Tilt 30 degrees downward
    cameraTargetPosition=[0, 0, 0]  # Focus on the origin (0, 0, 0)
)

# Load the KUKA robot and environment objects
planeId = p.loadURDF("plane.urdf")
cuboid_green_id = p.loadURDF("block.urdf", [0.44, 0, 0.01], [0, 0, 0, 1])
kuka_id = p.loadURDF("kuka_iiwa/kuka_with_prismatic_gripper.urdf")

eff_index = 7

numJoints = p.getNumJoints(kuka_id)
print("joint ID:\n\n\n", numJoints)

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
    control_joints = ["lbr_iiwa_joint_1", "lbr_iiwa_joint_2", "lbr_iiwa_joint_3",
                      "lbr_iiwa_joint_4", "lbr_iiwa_joint_5", "lbr_iiwa_joint_6",
                      "lbr_iiwa_joint_7"]
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
    return (position, p.getEulerFromQuaternion(orientation))


def get_joint_angles(kuka_or_gripper=None):
    "revolute joints: kuka + gripper"
    j = p.getJointStates(kuka_id, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
    joints = [i[0] for i in j]

    if kuka_or_gripper == 'kuka':
        return joints[:7]  # [0, 1, 2, 3, 4, 5, 6]
    
    elif kuka_or_gripper == 'gripper':
       return joints[8:]  # [8, 9]

    else:
        return joints


def execute_task_space_trajectory(time_step, duration, start_pos, final_pos):

    t = 0
    position_tolerance = 0.005
    trailDuration = 15
    prevPose=[0,0,0]
    prevPose1=[0,0,0]
    hasPrevPose = 0

    while True:

        position_1 = position_path(t, duration, start_pos[0], final_pos[0])
        orientation_1 = orientation_path(t, duration, start_pos[1], start_pos[1])

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

        if np.linalg.norm(np.array(get_current_pose()[0]) - np.array(final_pos[0])) < position_tolerance:
            print("End-effector reached the target position.")
            print(f"Current position: {get_current_pose()[0]}, Target position: {final_pos[1]}")
            print("\n\n\ntrajectory completed!\n\n\n")
            break

        print("obj tilt angle: ", get_object_state(cuboid_green_id)[1])  # object tilt angle (rad)

        p.stepSimulation()


def execute_gripper(time_step, init_pos, fin_pos, duration=1):
    """
    Smoothly open or close the gripper by interpolating the gripper opening angle.

    Args:
        time_step (float): Time increment for each simulation step.
        duration (float): Total duration for the gripper motion (in seconds).
        init_pos (float): Initial gripper opening distance.
        fin_pos (float): Final gripper opening distance.
    """
    t = 0
    position_tolerance = 0.006
    gripper_joints = ["left_finger_sliding_joint", "right_finger_sliding_joint"]

    while True:
        # Interpolate the gripper angle
        interpolated_dist = init_pos + (fin_pos - init_pos) * (t / duration)
        # fingers_pos = np.array([-interpolated_dist, interpolated_dist])
        fingers_pos = np.array([-fin_pos, fin_pos])

        # Set joint angles for the gripper
        poses = []
        indexes = []
        forces = []

        for i, name in enumerate(gripper_joints):
            joint = joints[name]
            poses.append(fingers_pos[i])
            indexes.append(joint.id)
            forces.append(joint.maxForce)

        p.setJointMotorControlArray(
            kuka_id, indexes,
            controlMode=p.POSITION_CONTROL,
            targetPositions=fingers_pos,
            targetVelocities=[0.]*2,
            positionGains=[0.05]*2,
            forces=[50.]*2
        )

        if np.linalg.norm(np.array(get_joint_angles("gripper")) - fingers_pos) < position_tolerance:
            print("End-effector reached the target position.")
            print(f"Current position: {get_current_pose()[0]}, Target position: {fin_pos}")
            print("\n\n\ntrajectory completed!\n\n\n")
            break

        # Step the simulation and increment time
        p.stepSimulation()
        t += time_step
        time.sleep(time_step)

    print("Gripper trajectory completed.")


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


def get_object_state(object_id):

    position, orientation = p.getBasePositionAndOrientation(object_id)
    orientation_euler = p.getEulerFromQuaternion(orientation)
    linear_velocity, angular_velocity = p.getBaseVelocity(object_id)

    # print(f"Position: {position}")
    # print(f"Orientation (Quaternion): {orientation}")
    # print(f"Orientation (Euler): {orientation_euler}")
    # print(f"Linear Velocity: {linear_velocity}")
    # print(f"Angular Velocity: {angular_velocity}")

    # return {
    #     "position": position,
    #     "orientation_quaternion": orientation,
    #     "orientation_euler": orientation_euler,
    #     "linear_velocity": linear_velocity,
    #     "angular_velocity": angular_velocity,
    # }

    return orientation_euler


time_step = .1  # TODO

## generalized position of end-effector: position + orientation (Euler)
start_pos = np.array([[0., 0., 1.305], [0., 0., 0.]])  # home position, up-right
second_pos = np.array([[0.41, 0., 0.48], [-np.pi/2, 0., -np.pi/2]])  # on top of the object with distance in z-axis
third_pos = np.array([[0.41, 0., 0.14], [-np.pi/2, 0., -np.pi/2]])  # right on top of the object
fourth_pos = np.array([[0.41, 0., 0.24], [-np.pi/2, 0., -np.pi/2]])  # right on top of the object


def main():

    des_kuka_joint_angle = np.array([0., 0.114, 0., -1.895, 0., 1.13, 0.])  # on top of the object with distance in z-axis
    # des_kuka_joint_angle = np.array([0., np.pi/6, 0., -2.094, 0., np.pi/6, np.pi/2])  # right on top of the object 

    set_joint_angles(des_kuka_joint_angle)
    wait_for_reaching_joint_angles(des_kuka_joint_angle)
    execute_gripper(time_step, init_pos=0., fin_pos=.01, duration=1)


    duration = 20
    execute_task_space_trajectory(time_step, duration, second_pos, third_pos)
    execute_gripper(time_step, init_pos=0.01, fin_pos=-.0029, duration=1)
    execute_task_space_trajectory(time_step, duration, third_pos, fourth_pos)


    while True:
        # print(get_joint_angles("gripper"))
        # print(get_current_pose())
        # print(get_object_state(cuboid_green_id)[1])  # object tilt angle (rad)

        p.stepSimulation()
        time.sleep(1 / 240)



if __name__ == '__main__':
    main()