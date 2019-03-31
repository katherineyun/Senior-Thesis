# USAGE
# python recognize_faces_image.py --encodings encodings.pickle --image examples/example_01.png

# import the necessary packages
import face_recognition
import argparse
import pickle
import cv2
import os, sys
import time
from matplotlib import pyplot as plt



def faceRec(encodings, image, person):
	print("[INFO] loading encodings...")
	data = pickle.loads(open(encodings, "rb").read())
	rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	print("[INFO] recognizing faces...")
	boxes = face_recognition.face_locations(rgb,
		model='hog')
	encodings = face_recognition.face_encodings(rgb, boxes)
	# initialize the list of names for each face detected
	names = []
	# loop over the facial embeddings
	for encoding in encodings:
		# attempt to match each face in the input image to our known
		# encodings
		matches = face_recognition.compare_faces(data["encodings"],
			encoding)
		name = "Unknown"
		# check to see if we have found a match
		if True in matches:
			# find the indexes of all matched faces then initialize a
			# dictionary to count the total number of times each face
			# was matched
			matchedIdxs = [i for (i, b) in enumerate(matches) if b]
			counts = {}

			# loop over the matched indexes and maintain a count for
			# each recognized face face
			for i in matchedIdxs:
				name = data["names"][i]
				counts[name] = counts.get(name, 0) + 1

			# determine the recognized face with the largest number of
			# votes (note: in the event of an unlikely tie Python will
			# select first entry in the dictionary)
			name = max(counts, key=counts.get)

		# update the list of names
		names.append(name)
	print(image.shape[:2])
	face_list = []
	for ((top, right, bottom, left), name) in zip(boxes, names):
		# draw the predicted face name on the image
		if name == person:
			cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
			y = top - 15 if top - 15 > 15 else top + 15
			cv2.putText(image, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
						0.75, (0, 255, 0), 2)
			face_list.append((name, ((top+bottom)/2, (left+right)/2)))

	print(face_list)
	cv2.namedWindow("output", cv2.WINDOW_NORMAL)
	imS = cv2.resize(image, (1080, 1960))
	cv2.imshow("Image",imS)
	while True:
		key = cv2.waitKey(1) & 0xff
		if key == 13:
			break
	return face_list




