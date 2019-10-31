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
        self._pos = pos

        #################################
        #########SET UP PYBULLET#########
        #################################
        # Add UR5 robot and mount (visual only)
        self._robot_body_id = p.loadURDF(
            "assets/ur5/ur5.urdf",
            np.add(self._pos, [0, 0, 0.4]),
            p.getQuaternionFromEuler(rot),
            # p.URDF_USE_SELF_COLLISION,
            # p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        )
        self._mount_body_id = p.loadURDF(
            "assets/ur5/mount.urdf",
            np.add(self._pos, [0, 0, 0.2]),
            p.getQuaternionFromEuler([0, 0, 0]))

        # Get revolute joint indices of robot (skip fixed joints)
        robot_joint_info = [p.getJointInfo(self._robot_body_id, i) for i in range(
            p.getNumJoints(self._robot_body_id))]
        self._robot_joint_indices = [
            x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._joint_epsilon = 0.01

        self._robot_home_joint_config = [-np.pi, -np.pi/2,
                                         np.pi/2, -np.pi/2,
                                         -np.pi/2, 0]

        self.reset()
        # Attach gripper to UR5 robot
        self._palm_body_id = p.loadURDF("assets/palm/palm.urdf",
                                        # p.URDF_USE_SELF_COLLISION,
                                        # p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
                                        )

        p.resetBasePositionAndOrientation(self._palm_body_id,
                                          np.add(self._pos, [0.5, 0.1, 0.8]),
                                          p.getQuaternionFromEuler([0, np.pi/2, 0]))
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
        self._tool_tip_to_ee_joint = [0, 0, 0.17]

        # Define Denavit-Hartenberg parameters for UR5
        self._ur5_kinematics_d = np.array(
            [0.089159, 0., 0., 0.10915, 0.09465, 0.0823])
        self._ur5_kinematics_a = np.array([0., -0.42500, -0.39225, 0., 0., 0.])

        # Set friction coefficients for gripper fingers
        for i in range(p.getNumJoints(self._palm_body_id)):
            p.changeDynamics(self._palm_body_id,
                             i,
                             lateralFriction=1.0,
                             spinningFriction=1.0,
                             rollingFriction=0.0001,
                             frictionAnchor=True)

        # Start thread to handle additional gripper constraints (gripper joint mimic behavior)
        # set one joint as the open/close motor joint (other joints should mimic)
        self._gripper_motor_joint_idx = 1

    def getConfigs(self):
        # MaxVelocity, TimeStep,Joints
        return self._maxVelocity, self._timeStep, 6

    # Control joint positions/velocities to enforce additional hard contraints on gripper behavior
    def step_constraints(self):
        gripper_joint_positions = np.array(
            [p.getJointState(self._palm_body_id, i)[0]
                for i in range(p.getNumJoints(self._palm_body_id))])
        p.setJointMotorControlArray(self._palm_body_id,
                                    [6, 3, 8, 5, 10],
                                    p.POSITION_CONTROL,
                                    [
                                        gripper_joint_positions[1],
                                        -gripper_joint_positions[1],
                                        -gripper_joint_positions[1],
                                        gripper_joint_positions[1],
                                        gripper_joint_positions[1]
                                    ],
                                    positionGains=np.ones(5))

    def applyAction(self, action):
        current_joint_state = [p.getJointState(self._robot_body_id, i)[
            0] for i in self._robot_joint_indices]
        target_joint_state = current_joint_state + action
        p.setJointMotorControlArray(self._robot_body_id,
                                    self._robot_joint_indices,
                                    p.POSITION_CONTROL,
                                    target_joint_state,
                                    positionGains=0.005*np.ones(
                                        len(self._robot_joint_indices)))
        # self.step_constraints()

    def getJoints(self):
        return [p.getJointState(self._robot_body_id, i)[
            0] for i in self._robot_joint_indices]

    def reset(self):
        # Move robot to home joint configuration
        p.setJointMotorControlArray(self._robot_body_id,
                                    self._robot_joint_indices,
                                    p.POSITION_CONTROL,
                                    self._robot_home_joint_config,
                                    positionGains=np.ones(
                                        len(self._robot_joint_indices)))

        actual_joint_state = [p.getJointState(self._robot_body_id, x)[
            0] for x in self._robot_joint_indices]
        timeout_t0 = time.time()
        # and (time.time()-timeout_t0) < timeout:
        while not all([np.abs(actual_joint_state[i]-self._robot_home_joint_config[i]) < self._joint_epsilon for i in range(6)]):
            p.stepSimulation()
            if time.time()-timeout_t0 > 5:
                print(
                    '[TossingBot] Timeout: robot motion exceeded 5 seconds. Skipping.')
                p.setJointMotorControlArray(self._robot_body_id,
                                            self._robot_joint_indices, p.POSITION_CONTROL,
                                            self._robot_home_joint_config,
                                            positionGains=np.ones(len(self._robot_joint_indices)))
                break
            actual_joint_state = [p.getJointState(self._robot_body_id, x)[
                0] for x in self._robot_joint_indices]
            time.sleep(0.01)
