#!/usr/bin/env python

# Copyright (c) 2018 Andy Zeng

import socket
import struct
import time
import numpy as np
import os
import sys
import math
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
from datetime import datetime

import gym
from gym import spaces
from gym.utils import seeding
from robot import Robot


class PyBulletSim(gym.Env):
    def __init__(self, gui=True, timeStep=0.01, NAgents=2, maxEpisodeLength=1000):
        self._timeStep = timeStep
        self._p = p
        self._gui = gui
        if gui:
            cid = p.connect(p.SHARED_MEMORY)  # What does this do???
            if (cid < 0):
                cid = p.connect(p.GUI)  # What does this do???
        else:
            p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self._timeStep)
        p.setRealTimeSimulation(0)

        self._plane_id = p.loadURDF("plane.urdf")
        self._boxId = p.loadURDF("assets/objects/cube/cube.urdf")
        # how many steps to stepSimulation before punishing illegal actions
        self._actionSteps = 50
        self.NAgents = NAgents

        #################################
        ##########SET UP ROBOT###########
        #################################
        positions = [[0.8, 0, 0], [-0.8, 0, 0], [0, 0.8, 0], [0, -0.8, 0]]
        rotations = [[0, 0, np.pi], [0, 0, 0], [0, 1, 0], [0, -1, 0]]
        self.robots = []
        for i in range(self.NAgents):
            self.robots.append(
                Robot(self._timeStep, pos=positions[i], rot=rotations[i]))
        maxVelocity, timeStep, Njoints = self.robots[0].getConfigs()

        #################################
        #######SET UP ACTION SPACE#######
        #################################
        action_dim = Njoints
        action_high = np.array(
            [maxVelocity*timeStep*self._actionSteps] * action_dim)
        self.action_space = np.array(
            [spaces.Box(-action_high, action_high) for _ in range(self.NAgents)])

        #################################
        ####SET UP OBSERVATION SPACE#####
        #################################
        self._observation = []
        # Each agents have 6 joints and box position and poses
        # observation dimension for each agent
        observation_dim = Njoints * self.NAgents + 7
        observation_high = np.array([1] * observation_dim)
        self.observation_space = np.array(
            [spaces.Box(-observation_high, observation_high) for _ in range(self.NAgents)])

        self.viewer = None  # TODO what is this for
        #################################
        #####OTHER OPENAI GYM STUFF######
        #################################
        self._max_episode_steps = maxEpisodeLength
        self._current_episode_step = 0
        ### TEMP FOR DEBUGGING ###

        self.seed()  # TODO
        self.reset()

    def reset(self):
        self.terminate_episode = False
        for robot in self.robots:
            robot.reset()
        for _ in range(50):
            p.stepSimulation()

        self._current_episode_step = 0
        self.resetBox()
        p.stepSimulation()
        self._observation = self._getObservation()
        return self._observation

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _getObservation(self):
        robotJoints = []
        for robot in self.robots:
            robotJoints.append(robot.getJoints())
        observation = []
        boxPos, boxRot = p.getBasePositionAndOrientation(self._boxId)
        boxPos = np.array(boxPos)
        boxRot = np.array(boxRot)
        boxObservation = np.concatenate((boxPos, boxRot))
        for i in range(len(self.robots)):
            robotObservation = np.array([])

            # Relative box position and rotation
            boxObservation = np.concatenate(
                (boxPos - self.robots[i]._pos, boxRot))
            robotObservation = np.concatenate(
                (robotObservation, boxObservation))
            # Self Joints
            robotObservation = np.concatenate(
                (robotObservation, robotJoints[i]))
            # Other's joints
            for j in range(len(self.robots)):
                if j != i:
                    robotObservation = np.concatenate((
                        robotObservation, robotJoints[j]))
            observation.append(robotObservation)
        return np.array(observation)

    def step(self, actions):
        assert len(actions) == self.NAgents, "Wrong Action Dimensions"
        self._current_episode_step += 1
        rewards = np.zeros(len(self.robots))
        # Compute target joint state
        target_joint_states = [robot.getJoints() + action
                               for robot, action in zip(self.robots, actions)]
        # Set Robot's target joint state
        for i, robot in enumerate(self.robots):
            robot.setTargetJointState(target_joint_states[i])

        for _ in range(self._actionSteps):
            p.stepSimulation()
            rewards += self._getReward()

        # Punish agent if suggested illegal action
        for i, robot in enumerate(self.robots):
            actual_joint_state = robot.getJoints()
            if not all([np.abs(actual_joint_state[joint_id]-target_joint_states[i][joint_id]) < 0.01 for joint_id in range(6)]):
                rewards[i] -= 1
        done = self.terminate_episode or self._current_episode_step >= self._max_episode_steps
        if not done and self._gui:
            self._observation = self._getObservation()
        return self._observation, rewards, done, {}

    def _getReward(self):
        rewards = np.zeros(len(self.robots))
        for i, robot in enumerate(self.robots):
            touchedBox = p.getContactPoints(
                self._boxId, robot._palm_body_id) != ()
            # touchedGround = p.getContactPoints(
            #     self._plane_id, robot._palm_body_id) != ()
            # touchedPalmSelf = p.getContactPoints(
            #     robot._robot_body_id, robot._palm_body_id) != ()
            # touchedSelf = p.getContactPoints(
            #     robot._robot_body_id, robot._robot_body_id) != ()
            if touchedBox:
                rewards[i] += 1
                # print("[{}] {} touched box!".format(
                #     datetime.now().strftime("%H:%M:%S"), i))
            # if touchedPalmSelf:
            #     rewards[i] -= 1
            # if touchedSelf:
            #     rewards[i] -= 1
            #     print("[{}] {} self collision!".format(
            #         int(round(time.time() * 1000)) % 100000, i))
            # if touchedGround:
            #     rewards[i] -= 1
        return rewards

    def resetBox(self):
        random_orientation = [
            np.random.randint(2)*np.pi/2,
            np.random.randint(2)*np.pi/2,
            np.random.random_sample()*2*np.pi-np.pi]

        p.resetBasePositionAndOrientation(
            self._boxId, [0, 1*(np.random.random_sample()-0.5), 0.3], p.getQuaternionFromEuler(random_orientation))
        for _ in range(10):
            p.stepSimulation()
