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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <tmc_yolo2_ros/Detections.h>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480
#define VOXEL_SIZE 0.01 //1cm voxels
#define VEL_CRITERION 0.001
#define CLUSTER_DIST_TOLERANCE 0.15
#define HUMAN_BOXES_NAMESPACE "human_boxes"
#define CLUSTER_BOXES_NAMESPACE "cluster_3D_boxes"
#define dobject_BOXES_NAMESPACE "dobject_3D_boxes"

#define KINECT_HEIGHT 1//0.7366 //m

#define CUTOFF_NUM 2 // Number of closest bounding boxes to consider

#define BOX_HEIGHT_TOL 0.1
#define BOX_WIDTH_TOL 0.1
#define BOX_DEPTH_TOL 0.1

#define BOX_HEIGHT_MAX_TOL 2.434
#define BOX_WIDTH_MAX_TOL 2.5
#define BOX_DEPTH_MAX_TOL 2.5

#define DEPTH_MAX 3.5

#define PURPLE_COLOR 45
#define GREEN_COLOR 0


#define	MAX_HEIGHT_TOL 2.434 //m Maximum height of person (8ft) to consider


class Cluster3D_BoundingBox{
public:
	float mean_x;
	float mean_y;
	float mean_z;

	float box_x_center; float box_y_center; float box_z_center;

	float x_min; float x_max;
	float y_min; float y_max;
	float z_min; float z_max;	

	pcl::PointIndices voxel_indices;
	int voxel_box_index;

	float box_x_size();
	float box_y_size();	
	float box_z_size();
	float distance_from_origin() const;

	Cluster3D_BoundingBox();
	Cluster3D_BoundingBox(const float _x_min, const float _x_max, 
						  const float _y_min, const float _y_max, 
						  const float _z_min, const float _z_max,
						  const float _mean_x, const float _mean_y, const float _mean_z,
						  const int _voxel_box_index, const pcl::PointIndices &_voxel_indices);
	~Cluster3D_BoundingBox();	

};

class BoundingBox_Person_Desc
{
public:
  float tl_x;
  float tl_y;
  float width;
  float height;  

  BoundingBox_Person_Desc(float _tl_x, float _tl_y, 
  						  float _width, float _height); // Constructor
  ~BoundingBox_Person_Desc(); // Destructor
};
BoundingBox_Person_Desc::BoundingBox_Person_Desc(float _tl_x, float _tl_y, float _width, float _height):
							tl_x(_tl_x), 
							tl_y(_tl_y),  
							width(_width), 
							height(_height){}

BoundingBox_Person_Desc::~BoundingBox_Person_Desc(){}

class Bounding_Box_dobject{
public:
	ros::NodeHandle 		  node;

	ros::Publisher 			  prism_voxels_pub;
	ros::Publisher 			  cluster_voxels_pub;	

	ros::Publisher            cluster_boxes_array_pub; //bounding box for candidate dobjects
	ros::Publisher 			  dobject_boxes_array_pub; //bounding box for extracted dobject
	ros::Publisher 			  human_boxes_array_pub; //bounding box for extracted dobject

	ros::Publisher 			  dobject_points_pub; // publish point cloud of dobject points;
	ros::Publisher 			  voxelized_pub; // publish point cloud of dobject points;	
	ros::Publisher 			  number_of_detected_dobjects_pub; //give number of detected dobjects
	ros::Publisher 			  human_box_pub;
	ros::Publisher 			  People_pos_pub;
	ros::Publisher 			  One_People_pos_pub;



	ros::Publisher 			  People_coordinate_pub;		//MK
	tf::TransformListener 	  listener;
	tf::StampedTransform 	  transform_base_head;			//MK

	ros::Subscriber			  detectedObjects_sub;
	ros::Subscriber			  yolo_detectedObjects_sub;	
	ros::Subscriber           registered_cloud_sub;
	ros::Subscriber 		   global_pos_sub;
	ros::Subscriber 		   Joint_states_sub;

	std::vector<BoundingBox_Person_Desc> boxes;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	std::vector< pcl::PointCloud<pcl::PointXYZRGB> > bounding_box_points;
	std::vector< pcl::PointCloud<pcl::PointXYZ> > voxelized_bounding_box_points;	

	std::vector< std::vector<pcl::PointIndices> > bounding_boxes_clusters; // each element is a bounding box which contains a vector of cluster indices
	// std::vector< std::vector<pcl::PointIndices> > bounding_boxes_human_clusters; // each element is a bounding box which contains a vector of cluster indices
	std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_clusters_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster
	std::vector<Cluster3D_BoundingBox>  bounding_box_clusters_humans; // each element is a bounding box which contains a vector of 3d boxes for the cluster

	std::vector<Cluster3D_BoundingBox> dobject_3D_boxes;


	visualization_msgs::MarkerArray cluster_boxes_array; 
	visualization_msgs::MarkerArray dobject_boxes_array;
	visualization_msgs::MarkerArray human_markers_array;

	// people_msgs::PositionMeasurement Position_array;

	std::vector<people_msgs::PositionMeasurement> people;
	
	///////////////      mk   //////////////////
	visualization_msgs::MarkerArray human_boxes_array;
	std::vector< std::vector<double> > box_pos_vec;
	int  box_pos_size;
	void publish_human_boxes();
	void publish_human_boxes_delayed();
	std::vector< std::vector<pcl::PointIndices> > bounding_boxes_humans; // each element is a bounding box which contains a vector of cluster indices
	std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_humans_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster
	/////////////////////////////////////////////
	bool IsHeadMoving;
	std::vector<double> Head_vel;
	std::vector<double> global_pose;
	bool isMoving;

	std::map<int, std::vector<int> > colormap;


	void yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg);	
	// void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
  	void dn_extract_points();
  	void dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg);
  	void voxelize_points();
	void extract_clusters();

	void extract_candidate_dobject_boxes();
	void extract_human_boxes();
	void extract_dobject_boxes();

	void publish_dn_points();
	void publish_clusters();
	void publish_clusters_boxes();
	void publish_dobject_3D_boxes();
	void publish_dobject_points();	

	double getDistance(double _x, double _y, double _t);
	bool   ISRobotMoving(double _x, double _y, double _t);


	void delete_previous_markers();
	void delete_previous_human_markers();
	
	void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
	void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	visualization_msgs::Marker createBoundingBoxMarker(const std::string &array_namespace,
													   const Cluster3D_BoundingBox &box, 
												  	   const int &marker_index, const int &apply_color);

	visualization_msgs::Marker createHumanMarker(const std::string &array_namespace,
													   const Cluster3D_BoundingBox &box, 
												  	   const int &marker_index, const int &apply_color);

	visualization_msgs::Marker createHumanMarker(const std::string &array_namespace,
																	   const double x, const double y, const double z,
																  	   const int &marker_index);

	void createHumanPositionMarker(const std::string &array_namespace,
																	   const Cluster3D_BoundingBox &box, 
																  	   const int &marker_index, const int &apply_color);

	Bounding_Box_dobject(); // Constructor
	~Bounding_Box_dobject();  // Destructor

private:
  	bool init_vars();

};



