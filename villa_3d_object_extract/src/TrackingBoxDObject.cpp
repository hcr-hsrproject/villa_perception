#include <TrackingBoxDObject.h>
#include <Mean3DTrackingBox.h>
#include <string>
#include <iostream>
#include <ros/package.h>
#include <vector>
#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480

#define VOXEL_SIZE 0.01 //1cm voxels

#define CLUSTER_DIST_TOLERANCE 0.2

#define DEPTH_MAX 10.0

#define CLUSTER_BOXES_NAMESPACE "cluster_3D_boxes"
#define dobject_BOXES_NAMESPACE "dobject_3D_boxes"

#define GREEN_COLOR 0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb_points (new pcl::PointCloud<pcl::PointXYZRGB>);	


Tracking_Box_dobject::Tracking_Box_dobject(): cloud(new pcl::PointCloud<pcl::PointXYZRGB>){
	init_vars();
}

bool Tracking_Box_dobject::init_vars(){
	received_image = false;
	std::cout << "Initialize received_image bool to:" << received_image << std::endl;

    std::string path = ros::package::getPath("villa_3d_object_extract");
    std::string filename = path + "/src/colors.txt";
    std::cout << filename << std::endl;
    FILE *fp = fopen(filename.c_str(),"rb"); 
    if(fp==NULL) {
        printf("Error trying to read file.\n");
    }

    size_t buffer_size = 80;
    char cmd[buffer_size]; // Character buffer
    int total_color_nums;

    // Get total number of color_nums
    int success = fscanf(fp, "%d", &total_color_nums); // This integer will be on the first line

    for (int i = 0; i < total_color_nums; i++){
      int color_num; int r; int g; int b;
      std::vector<int> color;
      int line_read_success = fscanf(fp, "%d %d %d %d", &color_num, &r, &g, &b);  
      color.push_back(r); color.push_back(g); color.push_back(b); 
      colormap[color_num] = color;

      printf("color_num: %d r: %d g: %d b: %d \n", color_num, r, g, b);

    }

    if (ferror(fp) != 0 || fclose(fp) != 0){
        return false;
    }

    printf(".....Successfully Loaded Colormap Parameters.\n");
    return true; 
}

Tracking_Box_dobject::~Tracking_Box_dobject(){}

void Tracking_Box_dobject::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	srv.request.img_input = (*msg);
/*	for (int i = 0; i < msg->encoding.size(); i++)
		{std::cout<< msg->encoding[i];}*/
	//received_image = true;
	// std::cout << "received_image bool to:" << received_image << std::endl;	
	}

void Tracking_Box_dobject::yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg){
if (SyncId == 1)
{
	SyncId = 0;
	std::vector<tmc_yolo2_ros::Detection> objects;
	objects = msg->detections;	
	vector_of_candidate_ids = msg->detections;	

	ROS_INFO("Hello world! I detected something");
	std::cout << "Number of detected objects: " << objects.size() << std::endl;

	int number_of_objects = objects.size();
	if (number_of_objects > 0)
	{
		srv.request.rect_input.clear();
		boxes.clear();
		std::vector<int> rect_temp (4);

		for(size_t i = 0; i < objects.size(); i++)
		{
			/*        std::cout << "Object " << i+1 << "    Class Name:" << objects[i].class_name << std::endl;                                
			*/        //Rectangle 
			int left  = (objects[i].x - objects[i].width/2.0);
			int right = (objects[i].x + objects[i].width/2.0);
			int top   = (objects[i].y - objects[i].height/2.0);
			int bot   = (objects[i].y + objects[i].height/2.0);
			if (right > CAMERA_PIXEL_WIDTH-1)  right = CAMERA_PIXEL_WIDTH-1;
			if (bot > CAMERA_PIXEL_HEIGHT-1)    bot = CAMERA_PIXEL_HEIGHT-1;
			int tl_x = left < 0 ? 0 : left;
			int tl_y = top < 0 ? 0 : top;
			int width_bound = right - tl_x;
			int height_bound = bot - tl_y;
			rect_temp[0] = tl_x;
			rect_temp[1] = tl_y;
			rect_temp[2] = width_bound;
			rect_temp[3] = height_bound;
			srv.request.rect_input.insert( srv.request.rect_input.end(), rect_temp.begin(), rect_temp.end() );
			boxes.push_back(TrackingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound ) );
		}

	/*std::cout<< "NUMBER OF BOXES :  "<< boxes.size()<<std::endl;
	*/
	std_msgs::Int8 num_msg;
	/*    if(dobject_boxes_array.markers.size() >= boxes.size() ){;
	num_msg.data = boxes.size();
	number_of_detected_dobjects_pub.publish(num_msg);
	}*/
	}
}     
}

void Tracking_Box_dobject::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){
//std::cout << a << std::endl;
pcl::fromROSMsg(*msg, *cloud);
	if (boxes.size() > 0 && client.call(srv))
	{
		list_indices = srv.response.indices_output;
		dn_extract_points(msg); 	// Extract points from each box
		extract_means_dobject_boxes(); // Get the means of each box
		if (vector_of_candidate_ids.size()==vector_of_candidate_means.size())
		{
			tracker();
			publish_tracked();
		}
		else {std::cout<< "ID vs MEAN : "<<vector_of_candidate_ids.size()<<"  "<<vector_of_candidate_means.size()<<std::endl;
}
	}
// 		// extract_dobject_boxes(); // Extract dobject boxes only;

// 	}
}

void Tracking_Box_dobject::dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg){
if (boxes.size() > 0)
{

	tracking_box_points.clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr prism_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	prism_points->header = (*cloud).header;

	int ADD = 0;
	for (int b_i = 0; b_i < boxes.size(); b_i++){		

	// For each box, extract pointcloud bounded by the darknet classifier. store pointcloud to a vector
		pcl::PointXYZRGB pt_color;
		for (int ind = ADD + 1; ind < ADD + list_indices[ADD]+1; ind++)
		{
			if (!pcl::isFinite(cloud->points[list_indices[ind]])){
				continue;
			}
	    	pt_color.x = cloud->points[list_indices[ind]].x;
	      	pt_color.y = cloud->points[list_indices[ind]].y;
	      	pt_color.z = cloud->points[list_indices[ind]].z;
	        pt_color.rgb = cloud->points[list_indices[ind]].rgb;
	        // prism_points->points.push_back(pt_color);
	        prism_points->points.push_back(pt_color);

		}
		ADD = ADD + 1 + list_indices[ADD];
		tracking_box_points.push_back(*prism_points);

	}
}
} 



void Tracking_Box_dobject::extract_means_dobject_boxes(){
if (SyncId == 0)
{
	SyncId = 1;
	vector_of_candidate_means.clear();
	std::cout<< "Tracking vector of boxes size "<<tracking_box_points.size()<<std::endl;
	for(size_t i = 0; i < tracking_box_points.size() ; i++)
	{
		int total_pts = 0;
	    float total_x = 0;
	    float total_y = 0;
	    float total_z = 0;		
		for(size_t j = 0; j < tracking_box_points[i].size() ; j++)
	    {
	    	float pt_x = tracking_box_points[i].points[j].x;
	      	float pt_y = tracking_box_points[i].points[j].y;
	      	float pt_z = tracking_box_points[i].points[j].z;

    		total_x += pt_x; total_y += pt_y; total_z += pt_z;
    		total_pts++;  
		} // Finished with iterating through the points

		if (total_pts > 0)
		{
			float mean_x = total_x / ( (float) total_pts);
			float mean_y = total_y / ( (float) total_pts);
			float mean_z = total_z / ( (float) total_pts);	
		
			vector_of_candidate_means.push_back(Mean3DTrackingBox(mean_x, mean_y, mean_z));
		}
		else 
		{
			std::cout<< " ERROR "<<std::endl;
			std::cout<< vector_of_candidate_ids[i].class_name << " will be erased " <<std::endl;
		    std::vector<tmc_yolo2_ros::Detection> vector_of_corrected_ids;
		    vector_of_corrected_ids = vector_of_candidate_ids;
			vector_of_candidate_ids.clear();
			for (int ind = 0; ind < vector_of_corrected_ids.size(); ind++)
			{
				if (ind != i)
				{
					vector_of_candidate_ids.push_back(vector_of_corrected_ids[ind]);
				}
			}
		}
	} 
	}
}

void Tracking_Box_dobject::tracker(){
char buffer [50];
std::vector<tmc_yolo2_ros::Detection> vector_of_temp_candidate_ids = vector_of_candidate_ids; 
std::vector<Mean3DTrackingBox> vector_of_temp_known_means = vector_of_known_means;
std::vector<int> Found_ID(vector_of_candidate_ids.size(),1) ;
int number_of_classes = 0;
NewClass.clear();

if ( vector_of_known_means.size() == 0) 
{ 
	vector_of_known_means = vector_of_candidate_means; 
	vector_of_known_ids = vector_of_candidate_ids;
	NewClass.resize(vector_of_candidate_ids.size(),0);
}
else 
{

for (int ind = 0; ind < vector_of_candidate_ids.size();ind++)
{	
	for (int test = ind + 1; test < vector_of_candidate_ids.size(); test++)
	{
		if ( (vector_of_candidate_ids[ind].class_name == vector_of_candidate_ids[test].class_name) && 
		 	  (vector_of_temp_candidate_ids[ind].class_name!="eterwqfw") )
		{
				Found_ID[ind] = Found_ID[ind] + 1; 
				vector_of_temp_candidate_ids[test].class_name = "eterwqfw";
		}
	}
	if ( vector_of_temp_candidate_ids[ind].class_name != "eterwqfw" )
	{
	std::cout<< vector_of_candidate_ids[ind].class_name << " was seen  "<<
	 Found_ID[ind]  <<" times "<<std::endl;
	}
}

NewClass.resize(vector_of_candidate_ids.size(),1);
float Delta = 0;

for (int i = 0; i < vector_of_candidate_means.size(); i++)
{
	float DeltaMin = 100;
/*	std::cout<< " Computing Deltas for " << vector_of_candidate_ids[i].class_name << std::endl;
*/
		for (int j = 0; j < vector_of_known_means.size(); j++ )
		{
			if ( vector_of_candidate_ids[i].class_name == vector_of_known_ids[j].class_name)
			{
			Delta = sqrt(pow(vector_of_candidate_means[i].mean_x - vector_of_known_means[j].mean_x,2) + 
						 pow(vector_of_candidate_means[i].mean_y - vector_of_known_means[j].mean_y,2) +
						 pow(vector_of_candidate_means[i].mean_z - vector_of_known_means[j].mean_z,2));
			std::cout<< " Delta  = " << Delta << std::endl;
			if ( Delta < 0.4)
			{	
				if (Delta < DeltaMin) 
				{	
				DeltaMin = Delta;
				vector_of_temp_known_means[j] = vector_of_candidate_means[i];
				NewClass[i] = 0;

				}	
			}
			}
	}
	if (NewClass[i] == 0)
	{
		std::cout<< " Object " << vector_of_known_ids[i].class_name << " is recognized !" << std::endl;}
	}
	/*	for (int index = 0; index < vector_of_known_means*/
	vector_of_known_means = vector_of_temp_known_means;

	for (int check = 0 ; check < vector_of_candidate_ids.size(); check++)
	{
		if (NewClass[check] == 1) 
		{
			std::cout<< " Object " << vector_of_candidate_ids[check].class_name << " is discovered ! "<< std::endl;
			vector_of_known_means.push_back(vector_of_candidate_means[check]);
			vector_of_known_ids.push_back(vector_of_candidate_ids[check]);
		}
	}
}

}

void Tracking_Box_dobject::publish_tracked()
{
pcl::PointXYZRGB pt;
xyzrgb_points->header = (*cloud).header;
for (int i = 0; i < tracking_box_points.size(); i++)
{
	if (NewClass[i] == 1)
	{
		std::cout<< " Object " << vector_of_candidate_ids[i].class_name << " is published ! "<< std::endl;
		for (int j = 0; j < tracking_box_points[i].points.size();j++)
		{
	    	pt.x = tracking_box_points[i].points[j].x;
	      	pt.y = tracking_box_points[i].points[j].y;
	      	pt.z = tracking_box_points[i].points[j].z;
	      	pt.rgb = tracking_box_points[i].points[j].rgb;
	      	xyzrgb_points->points.push_back(pt);			
		}
	}
}
Objects_points_pub.publish(xyzrgb_points);
}



