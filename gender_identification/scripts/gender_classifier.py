import numpy as np
import rospy
import tensorflow as tf

from rude_carnie.model import select_model, get_checkpoint
from rude_carnie import paths
from rude_carnie.detect import face_detection_model

GENDER_LIST =['M','F']

class GenderClassifier:
    def __init__(self):
        self.RESIZE_FINAL = 227
        self.label_list = GENDER_LIST
        self.nlabels = len(self.label_list)

        model_fn = select_model("inception")
        self.images = tf.placeholder(tf.float32, [None, self.RESIZE_FINAL, self.RESIZE_FINAL, 3])
        logits = model_fn(self.nlabels, self.images, 1, False)
        self.softmax_output = tf.nn.softmax(logits)


        checkpoint_path = paths.get_gender_checkpoint_path()
        model_checkpoint_path, _ = get_checkpoint(checkpoint_path, None,
                                                                      "checkpoint")

        saver = tf.train.Saver()
        self.sess = tf.Session()
        init_op = tf.global_variables_initializer()
        self.sess.run(init_op)

        saver.restore(self.sess, model_checkpoint_path)

    def crop_face(self, image_data):
        image_batch = image_data
        files = []
        face_detect = face_detection_model('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')
        face_files = face_detect.run(image_data)
        #print(face_files)
        if(len(face_files)==1):
            return face_files[0]
        else:
            return image_batch


    def classify(self, image_data):
        image_batch = image_data
        batch_results = self.sess.run(self.softmax_output, feed_dict={self.images: image_batch[np.newaxis, :, :, :]})
        output = batch_results[0]
        batch_sz = batch_results.shape[0]
        for i in range(1, batch_sz):
            output = output + batch_results[i]

        output /= batch_sz
        best = np.argmax(output)
        best_choice = (self.label_list[best], output[best])

        return best_choice
