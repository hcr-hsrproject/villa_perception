
#include <features_classifier.h>
#include <Cluster3DBoundingBoxDistanceCompare.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include "villa_3d_object_extract/srv_picture_to_indices.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int8.h>

#include <tmc_yolo2_ros/Detections.h>
#include <tmc_yolo2_ros/Detection.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define KINECT_HEIGHT 1//0.7366 //m

#define CUTOFF_NUM 2 // Number of closest bounding boxes to consider

#define BOX_HEIGHT_TOL 0.1
#define BOX_WIDTH_TOL 0.1
#define BOX_DEPTH_TOL 0.1

#define BOX_HEIGHT_MAX_TOL 2.434
#define BOX_WIDTH_MAX_TOL 2.5
#define BOX_DEPTH_MAX_TOL 2.5

#define PURPLE_COLOR 45

#define	MAX_HEIGHT_TOL 2.434 //m Maximum height of person (8ft) to consider
    std::vector<int> rect_temp (8);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "villa_3D_object_bounding_boxes");

	Features_check bbh_obj;


  	// Create Subsribers
	bbh_obj.yolo_detectedObjects_sub = bbh_obj.node.subscribe<tmc_yolo2_ros::Detections>("yolo2_node/detections", 10, boost::bind(&Features_check::yolo_detected_obj_callback, &bbh_obj, _1));	
	bbh_obj.image_sub = bbh_obj.node.subscribe<sensor_msgs::Image>("/hsrb/head_rgbd_sensor/rgb/image_rect_color", 1000, boost::bind(&Features_check::ImageCallback, &bbh_obj, _1));

	ros::spin();
	return 0;
}
