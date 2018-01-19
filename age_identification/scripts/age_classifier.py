from rude_carnie import model, paths
import tensorflow as tf
import numpy as np
from rude_carnie.detect import face_detection_model

AGE_LIST = ['(0, 2)', '(4, 6)', '(8, 12)', '(15, 20)', '(25, 32)', '(38, 43)', '(48, 53)', '(60, 100)']


class AgeClassifier:
    def __init__(self):
        self.RESIZE_FINAL = 227
        self.label_list = AGE_LIST
        self.nlabels = len(self.label_list)

        model_fn = model.select_model('inception')
        self.images = tf.placeholder(tf.float32, [None, self.RESIZE_FINAL, self.RESIZE_FINAL, 3])
        logits = model_fn(self.nlabels, self.images, 1, False)
        self.softmax_output = tf.nn.softmax(logits)


        checkpoint_path = paths.get_age_checkpoint_path()

        model_checkpoint_path, self.global_step = model.get_checkpoint(checkpoint_path, None,
                                                                      'checkpoint')

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

