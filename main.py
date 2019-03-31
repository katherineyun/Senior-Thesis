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


		# ##initialize environment
		env = gym.make('icub-v0')
		# speech = speechRec()
		# recognizer, microphone = speech.init()
		# word = speech.startGame(recognizer, microphone)
		# if(word == False):
		# 	return
		# char_names = ["Rachel", "Joey", "Phoebe", "Catherine", "Monica","Chandler", "Ross"]
		# art_names = ['jennifer_anniston', 'matt_leBlanc', 'lisa_kudrow', 'unknown', 'courteney_cox', 'matthew_perry', 'david_schwimmer']
		#
		# for i in range(len(char_names)):
		# 	if(word == char_names[i]):
		# 		target = art_names[i]
		# 		break

		target = 'jennifer_anniston'
		encodings = "env/Gazebo/friends.pickle"
		image = cv2.imread('env/Gazebo/00000054.jpg')


		#icub_coor = actConverter(target_coor)

		action = [0.41, -0.1, 0.96]
		#for i_episode in range(20):
		observ = env.reset()
		time.sleep(3)



		image = env.cameraReader()
		time.sleep(3)
		face = faceRec(encodings, image, target)
		target_coor = face[0][1]

		for t in range(200):
			env.step(action)
		env.close()

	return target_coor

if __name__ == "__main__":
	environment='icubEnv'
	target_coor = main(environment)

	print(target_coor)