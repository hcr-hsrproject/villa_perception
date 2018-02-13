import face_recognition
import sys
from os import listdir
from os.path import isfile, join
import numpy as np

def main():
	threshold = 0.6
	known_folder = sys.argv[1]
	known_imgs = [f for f in listdir(known_folder) if isfile(join(known_folder, f))]

	unknown_folder = sys.argv[2]
	unknown_imgs = [f for f in listdir(unknown_folder) if isfile(join(unknown_folder, f))]

	known_img_names = []
	known_encodings = []

        print len(known_img_names)
	for img_name in known_imgs:
		img = face_recognition.load_image_file(known_folder+'/'+img_name)
		known_encodings.append(face_recognition.face_encodings(img)[0])
		dot_idx = img_name.rfind('.')
		name = img_name[0:dot_idx]
		known_img_names.append(name)

	for img_name in unknown_imgs:
		img = face_recognition.load_image_file(unknown_folder+'/'+img_name)
		unknown_encoding = face_recognition.face_encodings(img)[0]
		distances = face_recognition.face_distance(known_encodings, unknown_encoding)
		if(min(distances)<threshold):
			match_idx = np.argmin(distances)
			print(img_name+': '+known_img_names[match_idx])
		else:
			print(img_name+': unknown')

main()
	
