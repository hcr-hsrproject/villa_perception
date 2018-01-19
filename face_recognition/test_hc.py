import face_recognition
import sys
from os import listdir
from os.path import isfile, join
import numpy as np

def main():

    img_name ="sample.JPG"    
    img = face_recognition.load_image_file(img_name)
    face_locations= face_recognition.face_locations(img)
    print face_locations
    # if(min(distances)<threshold):
	# match_idx = np.argmin(distances)
	# print(img_name+': '+known_img_names[match_idx])
    # else:
        # print(img_name+': unknown')

main()
	
