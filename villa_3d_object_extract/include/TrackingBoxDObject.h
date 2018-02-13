#ifndef TRACKING_BOX_D_OBJECT_H
#define TRACKING_BOX_D_OBJECT_H

#include <TrackingBoxPersonDesc.h>
#include <Mean3DTrackingBox.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>

#include <tmc_yolo2_ros/Detections.h>
#include <tmc_yolo2_ros/Detection.h>

//#include "grabcut_msgs/srv_picture_to_indices.h" // Adding GrabCut Service

class Tracking_Box_dobject{
public:
	ros::NodeHandle 		  node;
 	
 	// GrabCut Variables
 	sensor_msgs::Image 		  img;
	std::vector<int> 		  rect;
	std::vector<int>		  list_indices;
	//grabcut_msgs::srv_picture_to_indices srv;

	bool 					  received_image;
	//

	ros::Publisher 			  prism_voxels_pub;
	ros::Publisher 	          Objects_points_pub;
	ros::Publisher 			  cluster_voxels_pub;	

	ros::Publisher            cluster_boxes_array_pub; //bounding box for candidate dobjects
	ros::Publisher 			  dobject_boxes_array_pub; //bounding box for extracted dobject

	ros::Publisher 			  dobject_points_pub; // publish point cloud of dobject points;
	ros::Publisher 			  voxelized_pub; // publish point cloud of dobject points;	
	ros::Publisher 			  number_of_detected_dobjects_pub; //give number of detected dobjects



	ros::Subscriber			  detectedObjects_sub;
	ros::Subscriber			  yolo_detectedObjects_sub;	
	ros::Subscriber           registered_cloud_sub;
	ros::Subscriber           image_sub;
	
	// Client
	ros::ServiceClient        client;

	std::vector<TrackingBox_Person_Desc> boxes;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	std::vector< pcl::PointCloud<pcl::PointXYZRGB> > tracking_box_points;
	pcl::PointCloud<pcl::PointXYZRGB> bounding_objects_points;	

    std::vector<Mean3DTrackingBox> vector_of_candidate_means; 
    std::vector<tmc_yolo2_ros::Detection> vector_of_candidate_ids;
    int SyncId = 1;
	std::vector<int> known_ids;
	std::vector<int> NewClass;


    std::vector<Mean3DTrackingBox> vector_of_known_means; 
    std::vector<tmc_yolo2_ros::Detection> vector_of_known_ids; 


    std::map<int, std::vector<int> > colormap;

	void yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg);	
  	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
	
  	void dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg);

	void extract_means_dobject_boxes();

	void tracker();
	void publish_tracked();

	Tracking_Box_dobject(); // Constructor
	~Tracking_Box_dobject();  // Destructor


private:
  	bool init_vars();

};

#endif
