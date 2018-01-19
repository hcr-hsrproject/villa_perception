#include <BoundingBoxDObject.h>
#include <Cluster3DBoundingBoxDistanceCompare.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "villa_3D_object_bounding_boxes");

	Bounding_Box_dobject bbh_obj;

  	// Create Subsribers
	bbh_obj.yolo_detectedObjects_sub = bbh_obj.node.subscribe<tmc_yolo2_ros::Detections>("yolo2_node/detections", 10, boost::bind(&Bounding_Box_dobject::yolo_detected_obj_callback, &bbh_obj, _1));	
    bbh_obj.registered_cloud_sub = bbh_obj.node.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, boost::bind(&Bounding_Box_dobject::cloud_callback, &bbh_obj, _1));

	bbh_obj.prism_voxels_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/prism_voxels", 0 );
	bbh_obj.cluster_voxels_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/cluster_voxels", 0 );
	bbh_obj.cluster_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/cluster_boxes", 0 );
	bbh_obj.dobject_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/dobject_boxes_3D", 0 );



	bbh_obj.dobject_points_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/dobject_voxels", 0 );

	bbh_obj.number_of_detected_dobjects_pub = bbh_obj.node.advertise<std_msgs::Int8>("detection/number_of_detected_dobjects", 0);



/*	  message_filters::Subscriber<geometry_msgs::TransformStamped> sub1_;
	  message_filters::Subscriber<geometry_msgs::TransformStamped> sub2_;
	  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicy;
	  message_filters::Synchronizer<MySyncPolicy> sync_;

    sub1_(nh_, "topic1", 10),
    sub2_(nh_, "topic2", 10),
    sync_(MySyncPolicy(10),  sub1_, sub2_)
  
	sync_.registerCallback(boost::bind(&ExampleClass::Callback, this, _1, _2));	  
*/


	//Subscribe to detected objects
	//Subscribe to images
	//Subsribe to points




	ros::spin();
  return 0;
}
