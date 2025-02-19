import pybullet as p
import numpy as np
from typing import Tuple, List, Dict, Any
from dataclasses import dataclass
from enum import Enum
import pprint
from numpy import ndarray

class JointType(int, Enum):
    REVOLUTE = 0
    PRISMATIC = 1
    SPHERICAL = 2
    PLANAR = 3
    FIXED = 4
    POINT2POINT = 5
    HINGE = 6
    SLIDER = 7
    UNIVERSAL = 8
    GEAR = 9
    CONE_TWIST = 10
    D6 = 11
    MAX_JOINT_TYPE = 12

@dataclass
class JointState:
    position: float
    velocity: float
    reaction_forces: Tuple[float, float, float]
    applied_torque: float
@dataclass
class LinkState:
    world_pos: Tuple[float, float, float]
    world_orientation: Tuple[float, float, float, float]
    local_inertial_pos: Tuple[float, float, float]
    local_inertial_orn: Tuple[float, float, float]
    world_frame_pos: Tuple[float, float, float]
    world_link_frame_orn: Tuple[float, float, float]
    world_link_linear_velocity: Tuple[float, float, float]
    world_link_angular_velocity: Tuple[float, float, float]


@dataclass
class JointInfo:
    joint_index: int
    joint_name: str
    joint_type: JointType

    q_index: int #
    u_index: int
    flags: int
    joint_damping: float # specified in URDF
    joint_friction: float # specified in URDF

    joint_lower_limit: float
    joint_upper_limit: float
    joint_max_force: float
    joint_max_velocity: float
    link_name: str
    joint_axis: Tuple[float, float, float]
    parent_frame_pos: Tuple[float, float, float]
    parent_frame_orn: Tuple[float, float, float, float]
    parent_index: int

class Robot:
    def __init__(self, robot_id: int, physics_client: int):
        self.id: int = robot_id
        self.num_joints: int = p.getNumJoints(robot_id, physicsClientId=physics_client)
        self.physics_client:int  = physics_client
        self.joints: Dict[str, JointInfo] = {}
        self.control_mode: int = p.POSITION_CONTROL
        self._init_joints()

    def _init_joints(self)->None:
       for i in range(self.num_joints):
           info = p.getJointInfo(self.id, i, physicsClientId=self.physics_client)
           joint_name = info[1].decode('utf-8')
           self.joints[joint_name] = JointInfo(
               joint_index=info[0],
               joint_name=joint_name,
               joint_type=JointType(info[2]),
               q_index=info[3],
               u_index=info[4],
               flags=info[5],
               joint_damping=info[6],
               joint_friction=info[7],
               joint_lower_limit=info[8],
               joint_upper_limit=info[9],
               joint_max_force=info[10],
               joint_max_velocity=info[11],
               link_name=info[12].decode('utf-8'),
               joint_axis=info[13],
               parent_frame_pos=info[14],
               parent_frame_orn=info[15],
               parent_index=info[16]
           )
    def get_base_pose(self)->Tuple[np.ndarray, np.ndarray]:
        """ Return both position and orientation of the robot"""
        pos, orn = p.getBasePositionAndOrientation(self.id, physicsClientId=self.physics_client)
        return np.array(pos), np.array(orn)

    def get_joint_state(self, joint_name: str)-> JointState:
        """ Return the state of the joint"""
        joint_index = self.joints[joint_name].joint_index
        state = p.getJointState(self.id, joint_index, physicsClientId=self.physics_client)
        return JointState(position=state[0], velocity=state[1], reaction_forces=state[2], applied_torque=state[3])

    def set_joint_position(self, joint_name: str, position: float)->None:
        """ Set the position of the joint"""
        joint_index = self.joints[joint_name].joint_index
        p.setJointMotorControl2(self.id, joint_index, controlMode=self.control_mode, targetPosition=position, physicsClientId=self.physics_client)
    def reset_joint_state(self, joint_name: str, position: float)->None:
        """ Reset the position of the joint"""
        joint_index = self.joints[joint_name].joint_index
        p.resetJointState(self.id, joint_index, targetValue=position, targetVelocity=0, physicsClientId=self.physics_client)
    def get_link_state(self, link_name: str)-> LinkState:
        """ Return both position and orientation of the link"""
        link_index = self.joints[link_name].joint_index
        state = p.getLinkState(self.id, link_index, physicsClientId=self.physics_client)
        return LinkState(
            world_pos=state[0],
            world_orientation=state[1],
            local_inertial_pos=state[2],
            local_inertial_orn=p.getEulerFromQuaternion(state[3]),
            world_frame_pos=state[4],
            world_link_frame_orn=p.getEulerFromQuaternion(state[5]),
            world_link_linear_velocity=state[6],
            world_link_angular_velocity=state[7]
        )

    def get_joint_names(self)->List[str]:
        """ Return the names of the joints"""
        return list(self.joints.keys())

    def get_link_names(self)->List[str]:
        """ Return the names of the links"""
        return [joint.link_name for joint in self.joints.values()]

    def print_joint_info(self)->None:
        pprint.PrettyPrinter(indent=4).pprint(self.joints)

    def get_com(self) -> np.ndarray:
        """
        Calculate the center of mass of the robot in world coordinates
            R = SUM(mi*ri)/M
        """
        total_mass = 0
        weighted_pos = np.zeros(3)

        # get base mass and position of base
        info = p.getDynamicsInfo(self.id, -1, physicsClientId=self.physics_client)
        base_mass = info[0]
        base_pos, base_orn = p.getBasePositionAndOrientation(self.id, physicsClientId=self.physics_client)

        total_mass += base_mass
        weighted_pos += np.array(base_pos) * base_mass

        # loop through all links
        for joint in self.joints.values():
            info = p.getDynamicsInfo(self.id, joint.joint_index, physicsClientId=self.physics_client)
            link_mass = info[0]

            # Get world position of link's COM
            link_state = p.getLinkState(self.id, joint.joint_index, physicsClientId=self.physics_client)
            link_com_world_pos = link_state[0]  # World position of center of mass

            total_mass += link_mass
            weighted_pos += np.array(link_com_world_pos) * link_mass

        # calculate the center of mass
        com = weighted_pos / total_mass if total_mass > 0 else np.zeros(3)
        return com













