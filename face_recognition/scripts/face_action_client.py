#! /usr/bin/env python
from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import face_recognition_ros.msg
# from face_recognition_ros.msg import FaceRecogActionGoal

def face_action_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor.
	# goal_publisher =rospy.Publisher('/face_recog_action/goal', FaceRecogActionGoal, queue_size=10)
	client = actionlib.SimpleActionClient('face_recog_action', face_recognition_ros.msg.FaceRecogAction)
	# Waits until the action server has started up and started
	# listening for goals.
	print("wait for server")
	client.wait_for_server()
	# print("wait for server")
	# Creates a goal to send to the action server.
	goal = face_recognition_ros.msg.FaceRecogGoal(start=True)
	# goal.goal.start=True
	# goal_publisher.publish(goal)
	# client.wait_for_server()
	print("wait for server")
	# Sends the goal to the action server.
	client.send_goal(goal)
	print("goal sent")
	# Waits for the server to finish performing the action.
	client.wait_for_result()
	# ROS_INFO("Start action")

	# Prints out the result of executing the action
	return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('face_recog_action_client')
		result = face_action_client()
		# print("Result:", ', '.join([str(n) for n in result.sequence]))
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)