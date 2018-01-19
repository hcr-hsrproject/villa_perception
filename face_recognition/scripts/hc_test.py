import face_recognition
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from rude_carnie.model import select_model, get_checkpoint
from rude_carnie import paths
from rude_carnie.detect import face_detection_model



def crop_face(image_data):
	image_batch = image_data
	files = []
	face_detect = face_detection_model('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')
	face_files = face_detect.run(image_data)
	#print(face_files)
	return face_files


# Load the jpg file into a numpy array
image = face_recognition.load_image_file("ut_student2.jpg")
cropped_face=crop_face(image)

print len(cropped_face)

# # Find all the faces in the image
# face_locations = face_recognition.face_locations(image)

# print("I found {} face(s) in this photograph.".format(len(face_locations)))

# for face_location in face_locations:

#     # Print the location of each face in this image
#     top, right, bottom, left = face_location
#     print("A face is located at pixel location Top: {}, Left: {}, Bottom: {}, Right: {}".format(top, left, bottom, right))

#     # You can access the actual face itself like this:
#     face_image = image[top:bottom, left:right]
#     pil_image = image.fromarray(face_image)
# pil_image.show()
