#ifndef BOUNDING_BOX_D_OBJECT_H
#define BOUNDING_BOX_D_OBJECT_H

#include <BoundingBoxPersonDesc.h>
#include <Cluster3DBoundingBox.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>

#include <tmc_yolo2_ros/Detections.h>
#include <tmc_yolo2_ros/Detection.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//#include "grabcut_msgs/srv_picture_to_indices.h" // Adding GrabCut Service
class Bounding_Box_dobject_nicolas{
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

	std::vector<BoundingBox_Person_Desc> boxes;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	std::vector< pcl::PointCloud<pcl::PointXYZRGB> > bounding_box_points;
	std::vector< pcl::PointCloud<pcl::PointXYZ> > voxelized_bounding_box_points;	
	pcl::PointCloud<pcl::PointXYZRGB> bounding_objects_points;	


	std::vector< std::vector<pcl::PointIndices> > bounding_boxes_clusters; // each element is a bounding box which contains a vector of cluster indices
	std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_clusters_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster

	std::vector<Cluster3D_BoundingBox> dobject_3D_boxes;


	visualization_msgs::MarkerArray cluster_boxes_array; 
	visualization_msgs::MarkerArray dobject_boxes_array;


    std::map<int, std::vector<int> > colormap;

	void yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg);	
  	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
	
  	void dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg);
  	void voxelize_points();
  	void publish_objects_points();

	void extract_clusters();

	void extract_candidate_dobject_boxes();

	void extract_dobject_boxes();

	void publish_dn_points();
	void publish_clusters();
	void publish_clusters_boxes();
	void publish_dobject_3D_boxes();
	void publish_dobject_points();	

	void delete_previous_markers();

	visualization_msgs::Marker createBoundingBoxMarker(const std::string &array_namespace,
													   const Cluster3D_BoundingBox &box, 
												  	   const int &marker_index, const int &apply_color);

	Bounding_Box_dobject_nicolas(); // Constructor
	~Bounding_Box_dobject_nicolas();  // Destructor


private:
  	bool init_vars();

};

#endif
