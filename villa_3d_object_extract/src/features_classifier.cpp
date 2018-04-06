#include <features_classifier.h>
#include <Cluster3DBoundingBoxDistanceCompare.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include "villa_3d_object_extract/srv_picture_to_indices.h"

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480

#define VOXEL_SIZE 0.01 //1cm voxels

#define CLUSTER_DIST_TOLERANCE 0.2

#define DEPTH_MAX 10.0

#define CLUSTER_BOXES_NAMESPACE "cluster_3D_boxes"
#define dobject_BOXES_NAMESPACE "dobject_3D_boxes"

#define GREEN_COLOR 0


Features_check::Features_check() {
	init_vars();
}

bool Features_check::init_vars(){
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


Features_check::~Features_check(){}

void Features_check::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	srv.request.img_input = (*msg);
	received_image = true;
    ROS_INFO("Getting the picture");
    std::cout << boxes.size()<<std::endl;
	if (boxes.size() > 0 && client.call(srv)){
		list_indices = srv.response.indices_output;
        r = srv.response.r_list;
        g = srv.response.g_list;
        b = srv.response.b_list;

    	ROS_INFO("read rgb data from picture");
		rgb_read(msg); 	// Extract and check pixels from each box.
	// std::cout << "received_image bool to:" << received_image << std::endl;	
	}
    else {ROS_INFO("could not call grabcut service");}
}

void Features_check::yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg){
	std::vector<tmc_yolo2_ros::Detection> objects;
	objects = msg->detections;	
	ROS_INFO("Hello world! I detected something");
	std::cout << "Number of detected objects: " << objects.size() << std::endl;

    int number_of_objects = objects.size();
    if (number_of_objects > 0)
    {
    std::vector<int> rect_temp (4);
    srv.request.rect_input.clear();
    boxes.clear();
    for(size_t i = 0; i < objects.size(); i++){
        std::cout << "Object " << i+1 << "    Class Name:" << objects[i].class_name << std::endl;
        // std::cout << "    Class Name:" << objects[i].class_name << std::endl;
        // std::cout << "    Confidence:" << objects[i].confidence << std::endl;
        // std::cout << "      Center x:" << objects[i].x << std::endl;  
        // std::cout << "      Center y:" << objects[i].y << std::endl;
        // std::cout << "        Height:" << objects[i].height << std::endl;
        // std::cout << "         Width:" << objects[i].width << std::endl;                                          

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

        // std::cout << "    TL_x index:" << tl_x << std::endl;
        // std::cout << "    TL_y index:" << tl_y << std::endl;        
        // std::cout << "   width_bound:" << width_bound << std::endl; 
        // std::cout << "  height_bound:" << width_bound << std::endl; 

        rect_temp[0] = tl_x;
        rect_temp[1] = tl_y;
        rect_temp[2] = width_bound;
        rect_temp[3] = height_bound;

        srv.request.rect_input.insert( srv.request.rect_input.end(), rect_temp.begin(), rect_temp.end() );


        boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound ) );

    }


    std::cout << "Number of objects detected: " << boxes.size() <<std::endl;
/*    std_msgs::Int8 num_msg;
    if(dobject_boxes_array.markers.size() >= boxes.size() ){;
        num_msg.data = boxes.size();
        number_of_detected_dobjects_pub.publish(num_msg);
    }*/
    }

/*std::cout<< srv.request.rect_input.size()<<std::endl;
/*std::cout<<" get rect from yolo -> completed "<<std::endl;
*/}

void Features_check::rgb_read(const sensor_msgs::ImageConstPtr& msg){

	if (boxes.size() > 0){
	    bool detect = false;
		// bounding_box_points.clear();
		bounding_objects_points.clear();
		int ADD = 0;
		std::cout<< boxes.size() << " objects are seen"<<std::endl;
		bounding_objects_points.header = (*cloud).header;
		float ref_r = 255;
		float ref_g = 255;
		float ref_b = 255;
		// For each box, extract pointcloud bounded by the darknet classifier. store pointcloud to a vector
		for (int b_i = 0; b_i < boxes.size(); b_i++){		
			float red = 0;
			float green = 0;
			float blue = 0;
			for (int ind = ADD + 1; ind < ADD + list_indices[ADD]+1; ind++)
			{
				red = red + r[list_indices[ind]];
			  	green = green + g[list_indices[ind]];
			  	blue = blue + b[list_indices[ind]];
			}
			red = red / list_indices[ADD];
			green = green / list_indices[ADD];
			blue = blue / list_indices[ADD];
			ROS_INFO(" R G B :  %.3f   ,    %.3f   ,    %.3f  ", red,green,blue);
			if (ref_r - red < 5 && ref_g- green < 5 && ref_b - blue < 5 )
			{
				detect = true;
				std::cout << "Ref colour is being detected" << std::endl;
			}
			ADD = ADD + 1 + list_indices[ADD];
		}//End of box for loop
	} 
}
