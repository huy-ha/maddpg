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

import gym
from gym import spaces
from gym.utils import seeding
from robot import Robot


class PyBulletSim(gym.Env):
    def __init__(self, gui=True, timeStep=0.01, NAgents=2):
        self._timeStep = timeStep
        #################################
        #########SET UP PYBULLET#########
        #################################
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

        #################################
        ######TEMP MULTI-AGENT SETUP#####
        #################################
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
        maxVelocity, timeStep, joints = self.robots[0].getConfigs()

        #################################
        #######SET UP ACTION SPACE#######
        #################################
        action_dim = joints * self.NAgents
        action_high = np.array([maxVelocity*timeStep*10] * action_dim)
        self.action_space = []
        for _ in range(self.NAgents):
            self.action_space.append(spaces.Box(-action_high,
                                                action_high))
        self.action_space = np.array(self.action_space)

        #################################
        ####SET UP OBSERVATION SPACE#####
        #################################
        self._observation = []
        # Each agents have 6 joints and box position and poses
        observation_dim = (joints * self.NAgents + 7) * self.NAgents
        observation_high = np.array([1] * observation_dim)
        self.observation_space = []
        for _ in range(self.NAgents):
            self.observation_space.append(spaces.Box(-observation_high,
                                                     observation_high))
        self.observation_space = np.array(self.observation_space)
        self.viewer = None  # TODO what is this for
        #################################
        #####OTHER OPENAI GYM STUFF######
        #################################
        self._max_episode_steps = 1000
        self._current_episode_step = 0
        ### TEMP FOR DEBUGGING ###

        self.seed()  # TODO
        self.reset()

    def reset(self):
        self.terminate_episode = False
        for robot in self.robots:
            robot.reset()
        self._current_episode_step = 0
        self.resetBox()
        p.stepSimulation()
        self._observation = self._getObservation()
        reward = self._getReward()
        return self._observation, reward, False, {}

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # TODO check
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
            # Box position and rotation
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
        assert len(actions) != self.NAgents, "Wrong Action Dimensions"
        self._current_episode_step += 1
        for i, robot in enumerate(self.robots):
            robot.applyAction(actions[i])
        reward = np.zeros(len(self.robots))
        for _ in range(10):
            p.stepSimulation()
            reward += self._getReward()
        done = self.terminate_episode or\
            self._current_episode_step >= self._max_episode_steps
        if not done and self._gui:
            self._observation = self._getObservation()
        return self._observation, reward, done, {}

    def _getReward(self):
        reward = []
        for robot in self.robots:
            if p.getContactPoints(self._boxId, robot._palm_body_id) != ():
                reward.append(100)
                self.terminate_episode = True
            else:
                reward.append(-0.01)
        return reward

    def resetBox(self):
        random_orientation = [
            np.random.randint(2)*np.pi/2,
            np.random.randint(2)*np.pi/2,
            np.random.random_sample()*2*np.pi-np.pi]

        p.resetBasePositionAndOrientation(
            self._boxId, [0, 1*(np.random.random_sample()-0.5), 0.3], p.getQuaternionFromEuler(random_orientation))
        for _ in range(50):
            p.stepSimulation()