from sensor_msgs.msg import PointCloud2
from villa_perception_msgs.srv import AddPointCloud
import rospy
import sys

pt_cloud = None

def callback(msg):
    global pt_cloud
    pt_cloud = msg

if __name__ == '__main__':
    topic = sys.argv[1]
    srv = sys.argv[2]

    rospy.init_node('acc_test', anonymous=True)
    rospy.Subscriber(topic, PointCloud2, callback)

    rospy.wait_for_service(srv)
    srv = rospy.ServiceProxy(srv, AddPointCloud)

    while not rospy.is_shutdown():
        raw_input("Press Enter to add point cloud...")
        srv(pt_cloud)

