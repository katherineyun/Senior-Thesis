####################################################################
#Author: Peixin Chang
#This file is used for initializing YARP, Gazebo
#An OpenAI gym environment is the derived class for this
####################################################################

import gym
from libs.yarp_icub import yarp
from libs.yarp_icub import icub

import matplotlib.pyplot as plt
import sys
import time
import os
import numpy as np
#import matplotlib.pyplot as plt
from env.Gazebo.iCubManipulator import iCubG
from env.Gazebo.GazeboClient import GazeboClient
#from baselines import logger
#import cv2
import pyximport
pyximport.install()

from env.Gazebo.CExtension.imageGrabber.cImageGrabber import imageGrabber

import scipy.misc


import signal
import atexit


class icubEnv(gym.Env):
	"""Superclass for all Gazebo environments.
	"""
	metadata = {'render.modes': ['human']}


	def __init__(self):
		# START OF ROBOT CONFIGURATIONS
		robot_name = 'iCub0'
		obs_dim = (80, 120, 3)
		action_dim = 4
		state_dim = 18

		# change these two flags to output desired kind of observations
		motor = True
		img = False

		activationPart=[False, True, True, True, False, True]
		kwargs = {
			# robot configuration
			'activationPart': activationPart,
			#TODO: find out motor velocity limitation

			'useSparseReward': False

		}

		self.robot = iCubG(robot_name=robot_name, action_dim=action_dim,obs_dim=obs_dim, state_dim=state_dim,
						   motor=motor, img=img, **kwargs)

		# START OF ENV CONFIGURATIONS
		fullPath=os.getcwd()

		GazeboControlProperty = {
			'path2tables': fullPath+"/env/Gazebo/GazeboModel/tables/",
			 'path2items': fullPath+"/env/Gazebo/GazeboModel/assets/",
			 'path2world': fullPath+"/env/Gazebo/GazeboModel/iCub/worlds/icub_fixed.world",
			'path2image': fullPath + "/env/Gazebo/GazeboModel/images/",
			'tablex': 0.41, 'tabley': 0, 'tablez': 0.9,
			'activationPart': activationPart,
			'robot': self.robot,
			 'diff_height': False, 'height_range': 0.1,
			 'render': True,

			 }
		#initialize Gazebo Client, which will be shared in classes in iCubManipulator.py and icub_env
		self.G = GazeboClient(**GazeboControlProperty)
		self.GazeboStarted=0 # a flag indicating if the simulator has launched so that we won't launch it again

		self.done = 0
		self.reward = 0
		self.terminated = False
		self.envStepCounter = 0
		self.episodeCounter=0
		self.actionRepeat=1

		self.maxSteps = 200
		self.noTargetIndex = (0, 8)
		self.chapter = 0


		self.useSparseReward = kwargs['useSparseReward']

		self.episodeReward = 0.0

		self.lift = 0
		self.graspDuration = 0

		# register callback for signal handling
		for sig in (signal.SIGABRT, signal.SIGINT, signal.SIGTERM):
			signal.signal(sig, self.signal_handler)

		# register callback for exit()
		atexit.register(self.exit_handler)


	def signal_handler(self, signal, frame):
		self.G.clean_up()
		time.sleep(1)
		print("Signal Handled")
		sys.exit(0)

	def exit_handler(self):
		self.G.clean_up()
		time.sleep(1)
		print('Exit Handled')
		sys.exit(0)


	def reset(self):
		# After gym.make, we call env.reset(). This function is then called.

		# starts Gazebo only once
		if self.GazeboStarted==0:
			self.GazeboStarted=1
			# turn on Gazebo, YARP
			self.G.simulator_off()  # shut down existed Gazebo
			self.G.simulator_on()
			self.G.environment_setting() # set physics
			self.robot.G=self.G



		# every reset will reset these training related values
		self.done = 0
		self.reward = 0
		self.terminated = False
		self.envStepCounter = 0

		s = self.robot.reset() # load model here and do robot_specific reset and calc_state

		self.episodeCounter = self.episodeCounter + 1



		if self.robot.img and self.episodeCounter % 50 ==0:
			print("saving images for this episode")


		return
	def actConverter(self,target_coor):
		"""this function converts coordinates generated from cv face_recognizer to iCub's world coordinate. Here we want the iCub's
		finger to point towards this direction"""

		action = target_coor

		return action

	def step(self, action):

		self.resume()

		for i in range(self.actionRepeat):  # it is 1 by default, used for frame skip
			self.robot.applyAction(action)
			time.sleep(0.02)
			self.done = self._termination()
			if self.done:
				break
			self.envStepCounter = self.envStepCounter + 1

		self.pause()

		state = self.robot.calc_state()  # get low dimensional state
		return


	def pause(self):
		self.G.simulationControl("p", 1)  # pause the simulation
		return
	def resume(self):
		self.G.simulationControl("p", 0)  # unpause the simulation
		return

	def cameraReader(self):
		# Create numpy array to receive the image and the YARP image wrapped around it

		a = imageGrabber(640*480*3, self.robot.RCameraPort)
		a = np.reshape(a, (480,640,3))


		# scipy.misc.imsave("out"+str(self.envStepCounter)+".png",a)
		# self.envStepCounter += 1


		return a


	def close(self):
		self.G.clean_up()



	def _termination(self):

		if self.envStepCounter >= self.maxSteps - 1:
			return True

		return False


