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

		##initialize environment
		env = gym.make('icub-v0')

		##recognize speech
		speech = speechRec()
		recognizer, microphone = speech.init()
		word = speech.startGame(recognizer, microphone)
		if(word == False):
			return
		char_names = ["Rachel", "Joey", "Phoebe", "Catherine", "Monica","Chandler", "Ross"]
		art_names = ['jennifer_anniston', 'matt_leBlanc', 'lisa_kudrow', 'unknown', 'courteney_cox', 'matthew_perry', 'david_schwimmer']

		for i in range(len(char_names)):
			if(word == char_names[i]):
				target = art_names[i]
				break

		##decide target and face recognition
		#target = 'courteney_cox'
		encodings = "env/Gazebo/friends.pickle"
		#image = cv2.imread('env/Gazebo/00000054.jpg')

		#target_coor = [-0.41, 0.1, 0.5]

		env.reset()
		time.sleep(2)

		image = env.cameraReader()
		time.sleep(2)

		face = faceRec(encodings, image, target)
		target_coor = face[0][1]
		action = env.actConverter(target_coor)

		for t in range(300):
			env.step(action)

		time.sleep(3)
		env.close()


	return "Simulation Ended"

if __name__ == "__main__":
	environment='icubEnv'
	signal = main(environment)
	print(signal)