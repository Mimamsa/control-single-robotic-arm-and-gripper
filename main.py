"""
"""
import os
import math 
import numpy as np
import time
import pybullet as p
import random
from datetime import datetime
import pybullet_data
from collections import namedtuple
from scipy.spatial.transform import Rotation as R
import yaml
import click
import cv2

from KeystrokeCounter import KeystrokeCounter, Key, KeyCode
from CameraSim import CameraSim

# ROBOT_URDF_PATH = "./data/urdf/ur5e-hande.urdf"
ROBOT_URDF_PATH = "./data/urdf/tm5-900-hande.urdf"
TABLE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "table/table.urdf")
PLANE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
TRAY_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "tray", "tray.urdf")
DOMINO_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "domino", "domino.urdf")
LEGO_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "lego", "lego.urdf")


class RobotArmSim():
  
    def __init__(self):
        p.connect(p.GUI)
        p.setRealTimeSimulation(True)
        p.setGravity(0, 0, -9.8)
        
        self.end_effector_index = 9
        self.load_scene()
        self.robot = self.load_robot()
        self.num_joints = p.getNumJoints(self.robot)
        
        # for ur5e + hand-e
        # self.control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "hande_left_finger_joint", "hande_right_finger_joint"]
        # for tm5-900 + hand-e
        self.control_joints = ["shoulder_1_joint", "shoulder_2_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "hande_left_finger_joint", "hande_right_finger_joint"]
        self.joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        self.joint_info = namedtuple("jointInfo", ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity", "controllable"])

        self.joints = dict()
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = self.joint_type_list[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = True if jointName in self.control_joints else False
            info = self.joint_info(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
            # initialize velocity & force for revolute joints
            if info.type == "REVOLUTE":
                p.setJointMotorControl2(self.robot, info.id, p.VELOCITY_CONTROL, targetVelocity=0, force=10)
            self.joints[info.name] = info

    def __del__(self):
        p.disconnect()

    def load_scene(self):
        """Load scene """
        plane = p.loadURDF(PLANE_URDF_PATH, [0, 0, -0.6300], [0, 0, 0, 1])
        table = p.loadURDF(TABLE_URDF_PATH, [0.5, 0, -0.6300], [0, 0, 0, 1])
        # tray = p.loadURDF(TRAY_URDF_PATH, [0.5, 0, 0], [0, 0, 0, 1])
        domino = p.loadURDF(DOMINO_URDF_PATH, [0.6, 0, 0], [0, 0, 0, 1])
        lego = p.loadURDF(LEGO_URDF_PATH, [0.65, 0, 0], [0, 0, 0, 1])

    def load_robot(self):
        """Load robotic arm """
        # flags = p.URDF_USE_SELF_COLLISION
        flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE
        robot = p.loadURDF(ROBOT_URDF_PATH, [0, 0, 0], [0, 0, 0, 1], flags=flags)
        return robot

    def set_joint_values(self, joint_values):
        """ Set joints values.
        Args
            joint_values (list[float, 8]): Controllable joints values of the robotic arm.
        """
        poses = []
        indexes = []
        forces = []

        for i, name in enumerate(self.control_joints):
            joint = self.joints[name]
            poses.append(joint_values[i])
            indexes.append(joint.id)
            forces.append(joint.maxForce)

        p.setJointMotorControlArray(
            self.robot, indexes,
            p.POSITION_CONTROL,
            targetPositions=joint_values,
            targetVelocities=[0]*len(poses),
            positionGains=[0.04]*len(poses), forces=forces
        )

    def get_joint_values(self):
        """Get joints values. """
        j = p.getJointStates(self.robot, [1,2,3,4,5,6,10,11])
        joints = [i[0] for i in j]
        return joints

    def check_collisions(self):
        """Check collisions """
        collisions = p.getContactPoints()
        if len(collisions) > 10:
            print("[Collision detected!] {}".format(datetime.now()))
            return True
        return False

    def calculate_ik(self, position, orientation, gripper_offsets):
        """Calculate inverse-kinematics for controllable joints.
        Args
            position (list[float, 3]): Catesian position of end effector.
            orientation (list[float, 3]): Catesian oriention of end effector.
            gripper_offsets (list[float, 2]): Offsets of gripper joints.
        Returns
            (list[,8]): Controllable joint values.
        """
        quaternion = p.getQuaternionFromEuler(orientation)
        lower_limits = [-math.pi]*6 + [0.]*2
        upper_limits = [math.pi]*6 + [0.025]*2
        joint_ranges = [2*math.pi]*6 + [0.025]*2
        rest_poses = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0] + [0.]*2
        jointDamping = [0.01]*6 + [0.01]*2  # 

        joint_values = p.calculateInverseKinematics(
            self.robot, self.end_effector_index, position, quaternion, 
            jointDamping=jointDamping, upperLimits=upper_limits, 
            lowerLimits=lower_limits, jointRanges=joint_ranges, 
            restPoses=rest_poses
        )
        joint_values = list(joint_values)
        joint_values[-2:] = gripper_offsets

        return joint_values

    def add_gui_sliders(self):
        """Add GUI sliders. """
        self.sliders = []
        self.sliders.append(p.addUserDebugParameter("X", 0.3, 0.8, 0.4))
        self.sliders.append(p.addUserDebugParameter("Y", -0.5, 0.5, 0))
        self.sliders.append(p.addUserDebugParameter("Z", 0, 1, 0.4))
        self.sliders.append(p.addUserDebugParameter("Rx", -math.pi, math.pi, 0))
        self.sliders.append(p.addUserDebugParameter("Ry", -math.pi, math.pi, 0))
        self.sliders.append(p.addUserDebugParameter("Rz", -math.pi, math.pi, 0))
        self.sliders.append(p.addUserDebugParameter("gripper_offset", 0, 0.025, 0))

    def read_gui_sliders(self):
        """Read values from GUI sliders. """
        x = p.readUserDebugParameter(self.sliders[0])
        y = p.readUserDebugParameter(self.sliders[1])
        z = p.readUserDebugParameter(self.sliders[2])
        Rx = p.readUserDebugParameter(self.sliders[3])
        Ry = p.readUserDebugParameter(self.sliders[4])
        Rz = p.readUserDebugParameter(self.sliders[5])
        gx = p.readUserDebugParameter(self.sliders[6])
        return [x, y, z, Rx, Ry, Rz, gx]

    def get_current_pose(self, id):
        """Get current pose of the link
        Returns
            position: Catesian position of center of mass.
            orientation: Catesian oriention of center of mass, in quaternion [x,y,z,w].
        """
        linkstate = p.getLinkState(self.robot, id, computeForwardKinematics=True)
        position, orientation = linkstate[0], linkstate[1]
        return (position, orientation)

    def get_ee_pose(self):
        """Get current end effector pose. """
        return self.get_current_pose(self.end_effector_index)


@click.command()
@click.option('-ca', '--camera_attached', is_flag=True, default=False, help='On-hand camera attached or not')
def main(camera_attached):
    """Entry function """

    with open('config/camera.yaml', 'r') as f:
        cfg = yaml.safe_load(f)

    sim = RobotArmSim()
    sim.add_gui_sliders()

    if camera_attached:
        camera = CameraSim(**cfg['camera'])
        cap_count = 0

    with KeystrokeCounter() as key_counter:
        while True:
            if camera_attached:
                # On-hand camera
                pos, quat = sim.get_current_pose(9)
                rmat = R.from_quat(quat).as_matrix()
                eye_position = pos + rmat @ np.array([0, 0.01, 0.04])
                target_position = pos + rmat @ (np.array([0, 0.01, 0.04])*2)
                up_vector = rmat @ np.array([0, 1, 0])
                camera.set_pose(eye_position, target_position, up_vector)
                rgb, depth, mask = camera.get_camera_image()

            x, y, z, Rx, Ry, Rz, Gx = sim.read_gui_sliders()
            joint_values = sim.calculate_ik([x, y, z], [Rx, Ry, Rz], [Gx, Gx])
            joint_values = list(joint_values)

            sim.set_joint_values(joint_values)
            sim.check_collisions()

            press_events = key_counter.get_press_events()
            for key_stroke in press_events:
                if key_stroke == KeyCode(char='q'):
                    print('Exit.')
                    return
                elif key_stroke == Key.space:
                    key_counter.clear()
                    if camera_attached:
                        saved_path = 'capture-{}.jpg'.format(str(cap_count))
                        cap_count += 1
                        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                        cv2.imwrite(saved_path, rgb)
                        print('Image saved: {}'.format(saved_path))
                    else:
                        print('Camera is not attached.')

if __name__ == "__main__":
    main()