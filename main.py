import gym
from libs.yarp_icub import yarp
import numpy as np
import time
import matplotlib.pyplot as plt
from env.Gazebo.icub_env import icubEnv
from env.Gazebo.speech_rec import speechRec
from env.Gazebo.yichen_image import faceRec
import cv2


def main(env_id):
	#fixed location for now


	if env_id is 'icubEnv':


		#initialize environment
		# env = gym.make('icub-v0')
		speech = speechRec()
		recognizer, microphone = speech.init()
		word = speech.startGame(recognizer, microphone)
		if word == 'Rachel':
			target = 'jennifer_anniston'
		if word == 'Joey':
			target = 'matt_leBlanc'
		if word == 'Phoebe':
			target = 'lisa_kudrow'
		if word == 'Monica':
			target = 'courteney_cox'
		if word == 'Chandler':
			target = 'matthew_perry'
		if word == 'Ross':
			target = 'david_schwimmer'


		encodings = "env/Gazebo/friends.pickle"
		#image = env.cameraReader()
		image = cv2.imread('env/Gazebo/00000054.jpg')
		face = faceRec(encodings, image, target)


		# action = [0.41, -0.1, 0.96]
		# for i_episode in range(20):
		# 	observ = env.reset()
		#
		# 	for t in range(200):
		#
		# 		env.step(action)
		#
		# 		## plotting camera view
		# 		# a = env.cameraReader()
		# 		# img = plt.imshow(a)
		# 		# plt.show()
		#
		# env.close()


if __name__ == "__main__":
	environment='icubEnv'
	main(environment)

