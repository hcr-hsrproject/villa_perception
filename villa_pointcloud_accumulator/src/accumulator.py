from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
from villa_perception_msgs.msg import PointCloud2Array
from villa_perception_msgs.srv import AddPointCloud
from threading import Lock
import rospy
import tf


class PointCloudAccumulator(object):
    def __init__(self, node_name='accumulator', srv_name='add_point_cloud', pub_name='villa_merged_point_cloud', rate=10):
        rospy.init_node(node_name, anonymous=True)

        self.srv = rospy.Service(srv_name, AddPointCloud, self.add)
        self.pub = rospy.Publisher(pub_name, PointCloud2, queue_size=10)
        self.rate = rospy.Rate(rate)

        self.lock = Lock()
        self.tf = tf.TransformListener()

        self.frame = '/map'
        self.pc_array = PointCloud2Array()
        self.pc_merged = None

    def add(self, req):
        #pt_cloud = self.tf.transformPointCloud('/map', req.pt_cloud)
        pos, rot = self.tf.lookupTransform(self.frame, req.pt_cloud.header.frame_id, req.pt_cloud.header.stamp)
        transform = self.getTransform(req.pt_cloud.header, pos, rot)
        pt_cloud = do_transform_cloud(req.pt_cloud, transform)
        with self.lock:
            self.pc_array.pt_clouds.append(pt_cloud)
            self.merge(pt_cloud)
        return True

    def merge(self, pc):
        if self.pc_merged == None:
            self.pc_merged = pc
            self.pc_merged.row_step *= self.pc_merged.height
            self.pc_merged.width *= self.pc_merged.height
            self.pc_merged.height = 1
        else:
            self.pc_merged.header = pc.header
            self.pc_merged.width += pc.height*pc.width
            self.pc_merged.row_step += pc.row_step*pc.height
            self.pc_merged.data += pc.data

    def getTransform(self, header, pos, rot):
        transform = TransformStamped()

        transform.header = header
        transform.child_frame_id = self.frame

        transform.transform.translation.x = pos[0]
        transform.transform.translation.y = pos[1]
        transform.transform.translation.z = pos[2]
    
        transform.transform.rotation.x = rot[0]
        transform.transform.rotation.y = rot[1]
        transform.transform.rotation.z = rot[2]
        transform.transform.rotation.w = rot[3]
       
        return transform 

    def run(self):
        while not rospy.is_shutdown():
            if self.pc_merged != None:
                with self.lock:
                    self.pub.publish(self.pc_merged)
                self.rate.sleep()

if __name__ == '__main__':
    acc = PointCloudAccumulator()
    acc.run()

