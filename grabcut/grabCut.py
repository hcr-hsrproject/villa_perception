#!/usr/bin/env python

import argparse
import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
from grabcut_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError


np.set_printoptions(threshold='nan')


# def run_grabcut(img, rect):
#     """ Returns a numpy image array where the background has been 
#         replaced with black pixels
#     Args:
#         img (np.array): numpy array representing the image
#         rect (x,y,w,h): bounding rectangle which surrounds the foreground 
#                         of the image
#     """
#     mask = np.zeros(img.shape[:2], np.uint8)
#     bg_model = np.zeros((1,65), np.float64)
#     fg_model = np.zeros((1,65), np.float64)
#     cv2.grabCut(img, mask, rect, bg_model, fg_model, 5, cv2.GC_INIT_WITH_RECT)
#     mask2 = np.where((mask == 2) | (mask == 0), 0, 1).asType('uint8')
#     img = img * mask2[:,:,np.newaxis]
#     plt.imshow(img)
#     plt.colorbar()
#     plt.show()
#     return img


def get_foreground_pixels(img, rect):
    """ Returns all of the pixels in an image that are in the foreground
    Args:
        img (np.array): numpy array representing the image
        rect (x,y,w,h): bounding rectangle which surrounds the foreground 
                        of the image
    """
    mask = np.zeros(img.shape[:2], np.uint8)
    bg_model = np.zeros((1,65), np.float64)
    fg_model = np.zeros((1,65), np.float64)
    cv2.grabCut(img, mask, rect, bg_model, fg_model, 5, cv2.GC_INIT_WITH_RECT)
    flattened = mask.flatten()
    mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    img_rgb = img.flatten()
    img = img * mask2[:,:,np.newaxis]
    #  SHOW IMAGE
    # cv2.imshow("picture",img)
    # cv2.waitKey(100)
    background_coords = np.where((flattened == 1) | (flattened == 3))
    r = []
    g = []
    b = []
    r.extend(len(background_coords[0]))
    g.extend(len(background_coords[0]))
    b.extend(len(background_coords[0]))
    for i in range(len(background_coords)):
        r.extend((img_rgb[i])[0])
        g.extend((img_rgb[i])[1])
        b.extend((img_rgb[i])[2])
    # return (img , background_coords)
    ListBG = [len(background_coords[0])]
    ListBG.extend(list(background_coords[0]))
    return (ListBG , r , g , b)

def get_background_pixels(img, rect):
    """ Returns all of the pixels in an image that are in the background
    Args:
        img (np.array): numpy array representing the image
        rect (x,y,w,h): bounding rectangle which surrounds the foreground 
                        of the image
    """
    mask = np.zeros(img.shape[:2], np.uint8)
    bg_model = np.zeros((1,65), np.float64)
    fg_model = np.zeros((1,65), np.float64)
    cv2.grabCut(img, mask, rect, bg_model, fg_model, 5, cv2.GC_INIT_WITH_RECT)
    flattened = mask.flatten()
    background_coords = np.where((flattened == 0) | (flattened == 2))
    return background_coords

# Server
def handle_img(req):
    bridge = CvBridge()
    size = len(req.rect_input)/4;
    print "Rect : " 
    print  req.rect_input
    print req.img_input.encoding[0] 
    infos = []
    indices = []
    r = []
    g = []
    b = []
    if size > 0:
        print "Number of objects :"
        print  size

        img = bridge.imgmsg_to_cv2(req.img_input, desired_encoding="bgr8")
        mask = np.zeros(img.shape[:2], np.uint8)

        for i in range(size):
            try :
                infos = get_foreground_pixels(img,req.rect_input [(i*4):(i*4+4)])
                indices.extend(infos[0])
                r.extend(infos[1])
                g.extend(infos[2])
                b.extend(infos[3])
                
                # indices.extend( len ( get_foreground_pixels(img,req.rect_input[(i*4):(i*4+4)] ) ) );
                # indices.extend([])
                # cv2.imshow("picture",imgp)
                # cv2.waitKey(1000)
            except CvBridgeError, e:
              print e
              return []
 
    return srv_picture_to_indicesResponse(np.asarray(indices),np.asarray(r),np.asarray(g),np.asarray(b))


def grabcut_server():
    rospy.init_node('image_server')
    s = rospy.Service("grabcut_node_server", srv_picture_to_indices, handle_img)
    print " "
    print "gracut_node_server initialized ..."
    print "     Ready to grab the cut."
    print " "
    rospy.spin()

if __name__ == '__main__':
    i = 1
    while i < 2 :
      grabcut_server()
