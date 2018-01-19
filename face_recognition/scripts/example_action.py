#! /usr/bin/env python
from __future__ import print_function
import rospy

import actionlib

import villa_3d_object_extract.msg
from villa_3d_object_extract.srv import BoundingBoxesForClasses
from geometry_msgs.msg import PoseStamped


class FindObjectAction:
    def __init__(self, name):
        self.detector = rospy.ServiceProxy("extract_objects", BoundingBoxesForClasses)
        self.detector.wait_for_service(10)
        self._as = actionlib.SimpleActionServer(name, villa_3d_object_extract.msg.FindObjectAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.last_image = None
      
    def execute_cb(self, goal):
        result = villa_3d_object_extract.msg.FindObjectResult()
        success = False
        i = 0
        while i < 2:
            # TODO: Move head left for try 2 then right for try 3
            extractions = self.detector([goal.object_class])
            if extractions.class_names > 0:
                success = True
                break
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted')
                self._as.set_preempted()
                break

        if success:
            result.bbox_height = extractions.bbox_height[0]
            result.bbox_width = extractions.bbox_width[0]
            result.bbox_length = extractions.bbox_length[0]
            stamped_pose = PoseStamped()
            stamped_pose.pose = extractions.bbox_pose[0]
            stamped_pose.header = extractions.header
            result.bbox_pose = stamped_pose
            rospy.loginfo('%s: Succeeded')
            self._as.set_succeeded(result)
        
        
if __name__ == '__main__':
    rospy.init_node('find_object')
    server = FindObjectAction(rospy.get_name())
    print("Action ready")
rospy.spin()