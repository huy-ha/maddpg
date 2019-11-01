from robot import Robot
import pybullet as p
import pybullet_data
import numpy as np
import time

cid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
plane_id = p.loadURDF("plane.urdf")
boxId = p.loadURDF(
    "assets/objects/cube/cube.urdf",
    [0, 0, 1])
urdf_filename = "assets/ur5/ur5.urdf"
urdf_filename = "assets/ur5/ur5-rainbow.urdf"
robot_id = p.loadURDF(
    urdf_filename,
    [0.8, 0, 0.4],
    p.getQuaternionFromEuler([0, 0, np.pi]),
    p.URDF_USE_SELF_COLLISION,
    p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
)
for _ in range(10):
    p.stepSimulation()
    time.sleep(0.02)
robot_joint_ids = [
    x[0] for x in [p.getJointInfo(robot_id, i) for i in range(
        p.getNumJoints(robot_id))] if x[2] == p.JOINT_REVOLUTE]
robot_home_joint_config = [-np.pi, -np.pi/2,
                           np.pi/2, -np.pi/2,
                           -np.pi/2, 0]

# for joint_id, joint_pos in zip(robot_joint_ids, robot_home_joint_config):
#     print("setting joint {} to {}".format(joint_id, joint_pos))
#     p.resetJointState(robot_id, joint_id, joint_pos)
#     print("final joint state of {} : {} ".format(
#         joint_id, p.getJointState(robot_id, joint_id)[0]))

p.setJointMotorControlArray(robot_id,
                            robot_joint_ids,
                            p.POSITION_CONTROL,
                            robot_home_joint_config,
                            positionGains=np.ones(
                                len(robot_joint_ids)))
while True:
    p.stepSimulation()
    time.sleep(0.01)
