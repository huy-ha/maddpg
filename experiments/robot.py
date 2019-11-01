import pybullet as p
import numpy as np
import time


class Robot:
    def __init__(self, timeStep, pos=None, maxVelocity=5, maxForce=10000, rot=None):
        if pos is None:
            pos = np.array([0, 0, 0])
        if rot is None:
            rot = np.array([0, 0, 0])
        self._timeStep = timeStep
        self._maxVelocity = maxVelocity
        self._maxForce = maxForce
        self._rot = rot
        self._pos = pos

        #################################
        #########SET UP PYBULLET#########
        #################################
        # Add UR5 robot and mount (visual only)
        self._robot_body_id = p.loadURDF(
            "assets/ur5/ur5.urdf",
            np.add(self._pos, [0, 0, 0.4]),
            p.getQuaternionFromEuler(self._rot),
            # p.URDF_USE_SELF_COLLISION,
            # p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        )

        self._mount_body_id = p.loadURDF(
            "assets/ur5/mount.urdf",
            np.add(self._pos, [0, 0, 0.2]),
            p.getQuaternionFromEuler([0, 0, 0]))
        # for _ in range(200):
        #     p.stepSimulation()
        #     time.sleep(0.02)

        # Get revolute joint indices of robot (skip fixed joints)
        robot_joint_info = [p.getJointInfo(self._robot_body_id, i) for i in range(
            p.getNumJoints(self._robot_body_id))]
        self._robot_joint_indices = [
            x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        # for joint_id_1 in self._robot_joint_indices:
        #     for joint_id_2 in self._robot_joint_indices:
        #         if joint_id_1 != joint_id_2:
        #             p.setCollisionFilterPair(
        #                 self._robot_body_id, self._robot_body_id, joint_id_1, joint_id_2, 1)
        self._robot_joint_lower_limits = [
            x[8] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._robot_joint_upper_limits = [
            x[9] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._joint_epsilon = 0.01

        self._robot_home_joint_config = [-np.pi, -np.pi/2,
                                         np.pi/2, -np.pi/2,
                                         -np.pi/2, 0]
        # self._robot_home_joint_config = [0, 0, 0, 0, 0, 0]

        self._palm_body_id = p.loadURDF("assets/palm/palm.urdf",
                                        # p.URDF_USE_SELF_COLLISION,
                                        # p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
                                        )

        p.changeDynamics(self._palm_body_id,
                         -1,
                         lateralFriction=1.0,
                         spinningFriction=1.0,
                         rollingFriction=0.0001,
                         frictionAnchor=True)

        self.reset()

        self._robot_tool_joint_idx = 9
        self._robot_tool_tip_joint_idx = 10
        self._robot_tool_offset = [-0.03, 0, -0.01]
        # TODO how to do a fixed joint
        p.createConstraint(self._robot_body_id,
                           self._robot_tool_joint_idx,
                           self._palm_body_id,
                           -1,
                           jointType=p.JOINT_FIXED,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=[0, 0, 0],
                           childFramePosition=self._robot_tool_offset,
                           childFrameOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0]))
        for _ in range(100):
            p.stepSimulation()

    def getConfigs(self):
        # MaxVelocity, TimeStep,Joints
        return self._maxVelocity, self._timeStep, 6

    def applyAction(self, action):
        current_joint_state = [p.getJointState(self._robot_body_id, i)[
            0] for i in self._robot_joint_indices]
        target_joint_state = current_joint_state + action
        atupper = target_joint_state >= self._robot_joint_upper_limits
        atlower = target_joint_state <= self._robot_joint_lower_limits
        p.setJointMotorControlArray(self._robot_body_id,
                                    self._robot_joint_indices,
                                    p.POSITION_CONTROL,
                                    target_joint_state,
                                    positionGains=0.005*np.ones(
                                        len(self._robot_joint_indices)))
        return not any(atlower) or any(atupper)

    def getJoints(self):
        return [p.getJointState(self._robot_body_id, i)[
            0] for i in self._robot_joint_indices]

    def reset(self):
        # Move robot to home joint configuration
        for joint_id, joint_pos in zip(self._robot_joint_indices, self._robot_home_joint_config):
            p.resetJointState(self._robot_body_id, joint_id, joint_pos)
        # TODO do actual matrix multiplication and remove hard coded palm offsets
        if self._robot_body_id == 2:
            p.resetBasePositionAndOrientation(self._palm_body_id,
                                              np.add(self._pos, [
                                                     -0.5, -0.1, 0.8]),
                                              p.getQuaternionFromEuler([0, np.pi/2, 0]))
            # p.resetBasePositionAndOrientation(self._palm_body_id,
            #                                   np.add(self._pos, [
            #                                          0.8, 0.25, 0.4]),
            #                                   p.getQuaternionFromEuler([0, 0, np.pi/2]))
        elif self._robot_body_id == 5:
            p.resetBasePositionAndOrientation(self._palm_body_id,
                                              np.add(self._pos, [
                                                     0.5, 0.1, 0.8]),
                                              p.getQuaternionFromEuler([0, np.pi/2, 0]))
            # p.resetBasePositionAndOrientation(self._palm_body_id,
            #                                   np.add(self._pos, [-0.8, -0.25, 0.4]), p.getQuaternionFromEuler([0, 0, np.pi/2]))
