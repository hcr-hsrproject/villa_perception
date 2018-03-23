#include <human_tmc_cluster_based_yolo_3D_extractor.h>



Cluster3D_BoundingBox::Cluster3D_BoundingBox(){}
Cluster3D_BoundingBox::~Cluster3D_BoundingBox(){}

Cluster3D_BoundingBox::Cluster3D_BoundingBox(const float _x_min, const float _x_max, 
						  const float _y_min, const float _y_max, 
						  const float _z_min, const float _z_max,
  						  const float _mean_x, const float _mean_y, const float _mean_z,
						  const int _voxel_box_index, const pcl::PointIndices &_voxel_indices):
			x_min(_x_min), x_max(_x_max),
			y_min(_y_min), y_max(_y_max),
			z_min(_z_min), z_max(_z_max),
			mean_x(_mean_x), mean_y(_mean_y), mean_z(_mean_z),
			voxel_box_index(_voxel_box_index){

	voxel_indices = _voxel_indices;

	box_x_center = (x_max + x_min)/2.0;
	box_y_center = (y_max + y_min)/2.0;
	box_z_center = (z_max + z_min)/2.0;

}

float Cluster3D_BoundingBox::box_x_size(){
	return std::abs(x_max - x_min);
}
float Cluster3D_BoundingBox::box_y_size(){
	return std::abs(y_max - y_min);
}
float Cluster3D_BoundingBox::box_z_size(){
	return std::abs(z_max - z_min);
}
float Cluster3D_BoundingBox::distance_from_origin() const{
	return sqrt(pow(box_x_center, 2) + pow(box_y_center, 2) + pow(box_z_center, 2));
}

struct Cluster3D_BoundingBox_distance_compare {
	bool operator() (const Cluster3D_BoundingBox &lhs, const Cluster3D_BoundingBox &rhs) const{ 
		float lhs_dist = lhs.distance_from_origin();
		float rhs_dist = rhs.distance_from_origin();		 
		return lhs_dist < rhs_dist;	
	}

} distance_compare_obj;


//std::sort (myvector.begin(), myvector.end(), distance_cost_compare_obj) 


Bounding_Box_dobject::Bounding_Box_dobject(): cloud(new pcl::PointCloud<pcl::PointXYZRGB>){
	init_vars();
}

bool Bounding_Box_dobject::init_vars(){

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

      // printf("color_num: %d r: %d g: %d b: %d \n", color_num, r, g, b);

    }

    if (ferror(fp) != 0 || fclose(fp) != 0){
        return false;
    }

    printf(".....Successfully Loaded Colormap Parameters.\n");
    return true; 

	box_pos_size=10;
    box_pos_vec.resize(box_pos_size);
    for(int i(0);i <box_pos_size;i++)
    	box_pos_vec[i].resize(3);

    global_pose.resize(3,0.0);
    isMoving=false;
    Head_vel.resize(2,0.0);
    IsHeadMoving=false;

}


Bounding_Box_dobject::~Bounding_Box_dobject(){}

void Bounding_Box_dobject::yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg){
	std::vector<tmc_yolo2_ros::Detection> objects;
	objects = msg->detections;	

	//std::cout << "Number of detected objects: " << objects.size() << std::endl;
	int obj_idx=0;
    int number_of_objects = objects.size();
    boxes.clear();
    std::string person_str ("person");

    if(!IsHeadMoving){
    for(size_t i = 0; i < objects.size(); i++){

    	 if(person_str.compare(objects[i].class_name)== 0)			//when detection result is human
	     {	
	        // std::cout << "Object " << i+1 << std::endl;
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

			//boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
        	// ROS_INFO("I detected person : idx : %d", i);
        	boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
        	obj_idx++;
       	}
       	
       	if(person_str.compare(objects[i].class_name)!= 0)
        {	
        	 // ROS_INFO("I detected but it is not person : idx : %d", i);
        	 // std::cout << "    Class Name:" << objects[i].class_name << std::endl;
        	// boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
       	}
        
    }
    
    std::cout << "Number of human detected: " << boxes.size() <<std::endl;
    std_msgs::Int8 num_msg;
    if(dobject_boxes_array.markers.size() >= boxes.size()){
        num_msg.data = boxes.size();
        number_of_detected_dobjects_pub.publish(num_msg);
    }
            
 }
  

}

bool Bounding_Box_dobject::ISRobotMoving(double _x, double _y, double _t)
{

	double temp_dists=getDistance(_x,_y,_t);

	 //std::cout<<"temp_dist : "<<temp_dists<<std::endl;
	if(abs(temp_dists)>0.003){
			
	//	 ROS_INFO("Robot is Moving ");
		return true;
		}
	else
	{
		//ROS_INFO("Robot is not moving");
		return false;
	}
}

double Bounding_Box_dobject::getDistance(double _x, double _y, double _t)
{
	
	//ROS_INFO("get Distance");
	std::vector<double> temp_dist(3,0.0);
	
	temp_dist[0]=_x;
	temp_dist[1]=_y;
	temp_dist[2]=_t;



	double dist=0.0;
	if(global_pose[0]!=NULL)
	{
		for(int i(0);i<3;i++)
			dist+=(global_pose[i]-temp_dist[i])*(global_pose[i]-temp_dist[i]);
		dist=sqrt(dist);

	}	

	return dist;


}

void Bounding_Box_dobject::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){
	pcl::fromROSMsg(*msg, *cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_corners_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Set the same headers as original point cloud for simplicity   
	pt_corners_msg->header = (*cloud).header;

	// 
	// For each box, cluster points using euclidean clustering
	// for each box, extract dobject points using euclidean clustering
	//
	if (boxes.size() > 0){
		 dn_extract_points(msg); 	// Extract points from each box.
		 voxelize_points(); 			// filter points from each box.
		 extract_clusters();			// Cluster the voxels
		// //extract_human_boxes();

		 extract_candidate_dobject_boxes(); // Create bounding boxes on all the clusters
		 extract_dobject_boxes(); 		// Extract dobject boxes only;

		 // publish_dobject_3D_boxes();
		 // publish_dobject_points();
    	 publish_human_boxes();

		
		
	// 	 // publish_dn_points();		// Visualize voxels bounded by dn box
	// 	 // publish_clusters(); 		//Visualize Clusters
	// 	 // publish_clusters_boxes();		
				 
	}
}


void Bounding_Box_dobject::dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg){
	if (boxes.size() > 0){
		bounding_box_points.clear();

		// For each box, extract pointcloud bounded by the darknet classifier. store pointcloud to a vector
		for (int b_i = 0; b_i < boxes.size(); b_i++){		
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr prism_points (new pcl::PointCloud<pcl::PointXYZRGB>);
			prism_points->header = (*cloud).header;

			// Extract top left bounding box index
			int x = boxes[b_i].tl_x;
			int y = boxes[b_i].tl_y; 
			int tl_index = (y*CAMERA_PIXEL_WIDTH) + x;

			// Find points bounded by the dn box.
			int height = (int) boxes[b_i].height;		
			int width = (int) boxes[b_i].width;
			for (int m = 0; m < height; m++){
				for (int n = 0; n < width; n++){
					int point_index = tl_index + (m*CAMERA_PIXEL_WIDTH) + n;
				    // Prepare for copying
				    pcl::PointXYZRGB pt_color;

		    		if (!pcl::isFinite(cloud->points[point_index])){
		    			continue;
		    		}

			    	pt_color.x = cloud->points[point_index].x;
			      	pt_color.y = cloud->points[point_index].y;
			      	pt_color.z = cloud->points[point_index].z;
			        pt_color.rgb = cloud->points[point_index].rgb;

			        prism_points->points.push_back(pt_color);
				}
			}
			// Store vector of bounded point clouds 
			bounding_box_points.push_back(*prism_points);
		}//End of box for loop
	} 
}


void Bounding_Box_dobject::dn_extract_points(){
	if (boxes.size() > 0){
		bounding_box_points.clear();
		std::cout<<"where"<<std::endl;

		// For each box, extract pointcloud bounded by the darknet classifier. store pointcloud to a vector
		for (int b_i = 0; b_i < boxes.size(); b_i++){		
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr prism_points (new pcl::PointCloud<pcl::PointXYZRGB>);
			prism_points->header = (*cloud).header;

			std::cout<<"where1"<<std::endl;
			// Extract top left bounding box index
			int x = boxes[b_i].tl_x;
			int y = boxes[b_i].tl_y; 
			int tl_index = (y*CAMERA_PIXEL_WIDTH) + x;

			// Find points bounded by the dn box.
			int height = (int) boxes[b_i].height;		
			int width = (int) boxes[b_i].width;
			int hw=height*width;
			int pointsize = cloud->points.size();
			for (int m = 0; m < height; m++){
				for (int n = 0; n < width; n++){
					int point_index = tl_index + (m*CAMERA_PIXEL_WIDTH) + n;
				    std::cout<<"point index : " <<point_index<<std::endl;
				    std::cout<<"points size : "<<pointsize<<std::endl;
				    std::cout<<"width*height : "<<hw<<std::endl;
				    // Prepare for copying
				    pcl::PointXYZRGB pt_color;

		    		if (!pcl::isFinite(cloud->points[point_index])){
		    			continue;
		    		}

			    	pt_color.x = cloud->points[point_index].x;
			      	pt_color.y = cloud->points[point_index].y;
			      	pt_color.z = cloud->points[point_index].z;
			        pt_color.rgb = cloud->points[point_index].rgb;

			        prism_points->points.push_back(pt_color);
				}
			}
			// Store vector of bounded point clouds 
			std::cout<<"where2"<<std::endl;
			bounding_box_points.push_back(*prism_points);
		}//End of box for loop
	} 
}

void Bounding_Box_dobject::voxelize_points(){
	voxelized_bounding_box_points.clear();
	for(size_t b_i = 0; b_i < bounding_box_points.size(); b_i++){
		// Copy points to prepare for voxelization
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_prism_points (new pcl::PointCloud<pcl::PointXYZ>);		
		xyz_prism_points->header = (*cloud).header;

		for(size_t pt_i = 0; pt_i < bounding_box_points[b_i].points.size(); pt_i++){
		    pcl::PointXYZ pt_xyz;
	    	pt_xyz.x = bounding_box_points[b_i].points[pt_i].x;
	      	pt_xyz.y = bounding_box_points[b_i].points[pt_i].y;
	      	pt_xyz.z = bounding_box_points[b_i].points[pt_i].z;
	      	xyz_prism_points->points.push_back(pt_xyz);			
		}

		// Voxelize Points
	    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_xyz_prism_points (new pcl::PointCloud<pcl::PointXYZ>);			
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud( xyz_prism_points ); 
		// We set the size of every voxel to be 1x1x1cm (only one point per every cubic centimeter will survive).
		voxel_filter.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
		voxel_filter.filter(*voxelized_xyz_prism_points);

		// Check if voxelization succeeded
		//std::cout <<	 "Box ID" << b_i << " Original Points:" << bounding_box_points[b_i].points.size() << " "
		//		  << "Voxelized Points:" << voxelized_xyz_prism_points->size() << std::endl;

		// Store vector of voxelized bounded point cloud
		voxelized_bounding_box_points.push_back(*voxelized_xyz_prism_points);
	}	
}

void Bounding_Box_dobject::extract_clusters(){
	bounding_boxes_clusters.clear();
	// bounding_boxes_human_clusters.celar();
	for(size_t b_i = 0; b_i < voxelized_bounding_box_points.size(); b_i++){
		// Copy points to prepare for clustering
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_prism_points (new pcl::PointCloud<pcl::PointXYZ>);		
		xyz_prism_points->header = (*cloud).header;

		for(size_t pt_i = 0; pt_i < voxelized_bounding_box_points[b_i].points.size(); pt_i++){
		    pcl::PointXYZ pt_xyz;
	    	pt_xyz.x = voxelized_bounding_box_points[b_i].points[pt_i].x;
	      	pt_xyz.y = voxelized_bounding_box_points[b_i].points[pt_i].y;
	      	pt_xyz.z = voxelized_bounding_box_points[b_i].points[pt_i].z;
	      	xyz_prism_points->points.push_back(pt_xyz);			
		}

	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   		tree->setInputCloud ( xyz_prism_points );

	    std::vector<pcl::PointIndices> cluster_indices; // Eg: cluster_indices[0] contains all the indices that belong to cluster 0

    	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    	ec.setClusterTolerance (CLUSTER_DIST_TOLERANCE); 
    	ec.setMinClusterSize (1);
    	ec.setMaxClusterSize (40000);
    	ec.setSearchMethod (tree);
    	ec.setInputCloud ( xyz_prism_points );
    	ec.extract (cluster_indices);
    	bounding_boxes_clusters.push_back(cluster_indices);
	    //std::cout << "Box id " << b_i << " has " << cluster_indices.size() << " clusters" << std::endl;
    }
//    std::cout << "Number of Bounding Boxes Clusters: " << bounding_boxes_clusters.size() << std::endl;
}

void Bounding_Box_dobject::extract_human_boxes(){
	bounding_box_clusters_humans.clear();
	// For each DN Box, extract the candidate dobjects
	// std::cout<<"size of bounding_boxes_clusters:"<<bounding_boxes_clusters.size()<<std::endl;
	for(size_t i = 0; i < bounding_boxes_clusters.size() ; i++){
		// For each cluster in the box
    	// std::cout<<"size of sub bounding_boxes_clusters:"<< bounding_boxes_clusters[i].size()<<std::endl;

		for(size_t j = 0; j < bounding_boxes_clusters[i].size() ; j++){
			int box_index = i;
			int cluster_index = j;
			int n_clusters = bounding_boxes_clusters[box_index][cluster_index].indices.size();

  	      	float min_x = 10000;
		    float min_y = 10000;
		    float min_z = 10000;

		    float max_x = -10000;
		    float max_y = -10000;
		    float max_z = -10000;

		    int total_pts = 0;
		    float total_x = 0;
		    float total_y = 0;
		    float total_z = 0;		    

			// For each point in the cluster
			for(size_t k = 0; k < n_clusters; k++){
				int pt_index = bounding_boxes_clusters[box_index][cluster_index].indices[k];

		    	float pt_x = voxelized_bounding_box_points[box_index].points[pt_index].x;
		      	float pt_y = voxelized_bounding_box_points[box_index].points[pt_index].y;
		      	float pt_z = voxelized_bounding_box_points[box_index].points[pt_index].z;

      	        // Update minimum bounding box points
	        	if (pt_x < min_x){  min_x = pt_x; }
    	    	if (pt_y < min_y){  min_y = pt_y; }        
        		if (pt_z < min_z){  min_z = pt_z; }
        		// Update maximum bounding box points        
        		if (pt_x > max_x){  max_x = pt_x; }
        		if (pt_y > max_y){  max_y = pt_y; }        
        		if (pt_z > max_z){  max_z = pt_z; }

        		total_x += pt_x; total_y += pt_y; total_z += pt_z;
        		total_pts++;  
			} // Finished with iterating through the points

			if (total_pts > 0){
				float mean_x = total_x / ( (float) total_pts);
				float mean_y=  total_y / ( (float) total_pts);
				float mean_z = total_z / ( (float) total_pts);

				bounding_box_clusters_humans.push_back(Cluster3D_BoundingBox(min_x, max_x, 
			  				min_y, max_y, min_z, max_z, mean_x, mean_y, mean_z,
							box_index, bounding_boxes_clusters[box_index][cluster_index]) );

			//ROS_INFO("mean x : %.3lf, y :  %.3lf, z : %.3lf \n", mean_x, mean_y, mean_z);

			} // end if statement



		} // Finished with this cluster
		// bounding_box_clusters_3D.push_back(vector_of_boxes_for_cluster);
	} // Finished with this DN box
}

void Bounding_Box_dobject::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

	Head_vel.resize(2,0.0);
	Head_vel[0]=msg->velocity[9];
	Head_vel[1]=msg->velocity[10];

	if(abs(Head_vel[0])>VEL_CRITERION)
		IsHeadMoving=true;
	else
		IsHeadMoving=false;

	// ROS_INFO("Head moving : %s",IsHeadMoving);
	 // 0.000138, -3.8e-05,
	// 0.000138, -0.001567,

}

void Bounding_Box_dobject::extract_candidate_dobject_boxes(){
	bounding_box_clusters_3D.clear();
	// For each DN Box, extract the candidate dobjects
	std::cout<<"size of bounding_boxes_clusters:"<<bounding_boxes_clusters.size()<<std::endl;
	for(size_t i = 0; i < bounding_boxes_clusters.size() ; i++){
		// For each cluster in the box
    std::vector<Cluster3D_BoundingBox> vector_of_boxes_for_cluster; 

		for(size_t j = 0; j < bounding_boxes_clusters[i].size() ; j++){
			int box_index = i;
			int cluster_index = j;
			int n_clusters = bounding_boxes_clusters[box_index][cluster_index].indices.size();

  	      	float min_x = 10000;
		    float min_y = 10000;
		    float min_z = 10000;

		    float max_x = -10000;
		    float max_y = -10000;
		    float max_z = -10000;

		    int total_pts = 0;
		    float total_x = 0;
		    float total_y = 0;
		    float total_z = 0;		    

			// For each point in the cluster
			for(size_t k = 0; k < n_clusters; k++){
				int pt_index = bounding_boxes_clusters[box_index][cluster_index].indices[k];

		    	float pt_x = voxelized_bounding_box_points[box_index].points[pt_index].x;
		      	float pt_y = voxelized_bounding_box_points[box_index].points[pt_index].y;
		      	float pt_z = voxelized_bounding_box_points[box_index].points[pt_index].z;

      	        // Update minimum bounding box points
	        	if (pt_x < min_x){  min_x = pt_x;}
    	    	if (pt_y < min_y){  min_y = pt_y;}        
        		if (pt_z < min_z){  min_z = pt_z;}
        		// Update maximum bounding box points        
        		if (pt_x > max_x){  max_x = pt_x;}
        		if (pt_y > max_y){  max_y = pt_y;}        
        		if (pt_z > max_z){  max_z = pt_z;}

        		total_x += pt_x; total_y += pt_y; total_z += pt_z;
        		total_pts++;  
			} // Finished with iterating through the points

		
			if (total_pts > 0){
			float mean_x = total_x / ( (float) total_pts);
			float mean_y=  total_y / ( (float) total_pts);
			float mean_z = total_z / ( (float) total_pts);				
			vector_of_boxes_for_cluster.push_back(Cluster3D_BoundingBox(min_x, max_x, 
	  				min_y, max_y, min_z, max_z, mean_x, mean_y, mean_z,
					box_index, bounding_boxes_clusters[box_index][cluster_index]) );
		


				// bounding_box_clusters_humans.push_back(createHumanMarker(min_x, max_x, 
			    // min_y, max_y, min_z, max_z, mean_x, mean_y, mean_z,
				// box_index, bounding_boxes_clusters[box_index][cluster_index]) );

			//ROS_INFO("mean x : %.3lf, y :  %.3lf, z : %.3lf \n", mean_x, mean_y, mean_z);

			} // end if statement

		
		} // Finished with this cluster
		// ROS_INFO("Number of Box made in extract d_object: %d",i );
		 bounding_box_clusters_3D.push_back(vector_of_boxes_for_cluster);
	} // Finished with this DN box
}


visualization_msgs::Marker Bounding_Box_dobject::createHumanMarker(const std::string &array_namespace,
																	   const double _x, const double _y, const double _z, 
																  	   const int &marker_index ){


	// std::cout<<"chair Creating : marker index :" <<marker_index<<std::endl;
	geometry_msgs::Vector3Stamped gV, tV;

    gV.vector.x = _x;
    gV.vector.y = _y;
    gV.vector.z = _z;

    //std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
    gV.header.stamp = ros::Time();
    gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
    listener.transformVector("/map", gV, tV);

	visualization_msgs::Marker marker_human;
	marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.ns = HUMAN_BOXES_NAMESPACE;
    marker_human.id = marker_index;

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_human.type = shape;

    marker_human.pose.position.x = tV.vector.x;
    marker_human.pose.position.y = tV.vector.y;
    marker_human.pose.position.z = 1;

    marker_human.pose.orientation.x = 0.0;
    marker_human.pose.orientation.y = 0.0;
    marker_human.pose.orientation.z = 0.0;
    marker_human.pose.orientation.w = 1.0;

    double temp_dist,temp_dist2,temp_dist3;
    temp_dist  =0.5;
    temp_dist2 =0.5;
    temp_dist3 =0.5;

    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
    marker_human.scale.x = std::abs(temp_dist);
    marker_human.scale.y = std::abs(temp_dist2);
    marker_human.scale.z = std::abs(temp_dist3);

    marker_human.color.r = 1.0;
    marker_human.color.g = 1.0;
    marker_human.color.b = 0.2;
    marker_human.color.a = 0.85;

    // human_markers_array.markers.push_back(marker_human);
    ROS_INFO("human_markers_array_pushback");
    return marker_human;



}



void Bounding_Box_dobject::createHumanPositionMarker(const std::string &array_namespace,
																	   const Cluster3D_BoundingBox &box, 
																  	   const int &marker_index, const int &apply_color){

		people_msgs::PositionMeasurement pos;


		geometry_msgs::Vector3Stamped gV, tV;
	    gV.vector.x = box.box_x_center;
	    gV.vector.y = box.box_y_center;
	    gV.vector.z = box.box_z_center;

	    gV.header.stamp = ros::Time();
	    gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
	    listener.transformVector(std::string("/map"), gV, tV);

		pos.header.stamp = ros::Time();
        pos.header.frame_id = "/map";
        pos.name = array_namespace;
        pos.object_id ="person 0";
        std::cout<<"marker idx : "<<marker_index<<std::endl;
        pos.pos.x = tV.vector.x+global_pose[0];
        pos.pos.y = tV.vector.y+global_pose[1];
        pos.pos.z = tV.vector.z;
        pos.reliability = 0.95;
        pos.covariance[0] = pow(0.01 / pos.reliability, 2.0);
        pos.covariance[1] = 0.0;
        pos.covariance[2] = 0.0;
        pos.covariance[3] = 0.0;
        pos.covariance[4] = pow(0.01 / pos.reliability, 2.0);
        pos.covariance[5] = 0.0;
        pos.covariance[6] = 0.0;
        pos.covariance[7] = 0.0;
        pos.covariance[8] = 10000.0;
        pos.initialization = 0;

		people.push_back(pos);

		// One_People_pos_pub.publish(pos);
	
    // leg_measurements_pub_.publish(array);

	return;

}



visualization_msgs::Marker Bounding_Box_dobject::createHumanMarker(const std::string &array_namespace,
																	   const Cluster3D_BoundingBox &box, 
																  	   const int &marker_index, const int &apply_color){

	// std::cout<<"Humamarker Creating : marker index :" <<marker_index<<std::endl;

	//geometry_msgs::Vector3Stamped gV, tV;
    //gV.vector.x = box.box_x_center;
    //gV.vector.y = box.box_y_center;
    //gV.vector.z = box.z_min;

    //gV.header.stamp = ros::Time();
    //gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
    //listener.transformVector(std::string("/map"), gV, tV);

	visualization_msgs::Marker marker_human;
	// marker_human.header.frame_id = "/map"; 
    marker_human.header.frame_id ="/head_rgbd_sensor_rgb_frame";
    marker_human.header.stamp = ros::Time::now();
    marker_human.ns = HUMAN_BOXES_NAMESPACE;
    marker_human.id = marker_index;

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_human.type = shape;

    marker_human.pose.position.x = box.box_x_center;
    marker_human.pose.position.y = box.box_y_center;
    marker_human.pose.position.z = box.z_min;


    // marker_human.pose.position.x = tV.vector.x+global_pose[0];
    // marker_human.pose.position.y = tV.vector.y+global_pose[1];
    // marker_human.pose.position.z = 1;

    marker_human.pose.orientation.x = 0.0;
    marker_human.pose.orientation.y = 0.0;
    marker_human.pose.orientation.z = 0.0;
    marker_human.pose.orientation.w = 1.0;

    double temp_dist,temp_dist2,temp_dist3;
    temp_dist  =0.5;
    temp_dist2 =0.5;
    temp_dist3 =0.5;

    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
    marker_human.scale.x = std::abs(temp_dist);
    marker_human.scale.y = std::abs(temp_dist2);
    marker_human.scale.z = std::abs(temp_dist3);

    marker_human.color.r = 1.0;
    marker_human.color.g = 1.0;
    marker_human.color.b = 0.2;
    marker_human.color.a = 0.85;

    // ROS_INFO("human_markers_array_pushback");
    return marker_human;

}

visualization_msgs::Marker Bounding_Box_dobject::createBoundingBoxMarker(const std::string &array_namespace,
																	   const Cluster3D_BoundingBox &box, 
																  	   const int &marker_index, const int &apply_color){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = (*cloud).header.frame_id; 
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = array_namespace;
    marker.id = marker_index;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = box.box_x_center;//(box.x_max + box.x_min)/2.0;
    marker.pose.position.y = box.box_y_center;//(box.y_max + box.y_min)/2.0;
    marker.pose.position.z = box.box_z_center;//(box.z_max + box.z_min)/2.0;
    ////////////////////////////////////
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = std::abs(box.x_max - box.x_min);
    marker.scale.y = std::abs(box.y_max - box.y_min);
    marker.scale.z = std::abs(box.z_max - box.z_min);
    // Set the color - be sure to set alpha to something non-zero!
    //int	apply_color = marker_index % colormap.size();

    marker.color.r = ((float) colormap[apply_color][0]) / 255.0;
    marker.color.g = ((float) colormap[apply_color][1]) / 255.0;
    marker.color.b = ((float) colormap[apply_color][2]) / 255.0;
    marker.color.a = 0.85;

    marker.lifetime = ros::Duration(1.0);

    return marker;
}



void Bounding_Box_dobject::extract_dobject_boxes(){
	dobject_3D_boxes.clear();
	// Returns the closest cluster found

	for(size_t i = 0; i < bounding_box_clusters_3D.size() ; i++){
		std::vector<Cluster3D_BoundingBox> current_dn_bounding_box;	

		if (bounding_box_clusters_3D[i].size() > 0){
			// Copy Bounding boxes
			for(size_t j = 0; j < bounding_box_clusters_3D[i].size() ; j++){
				current_dn_bounding_box.push_back(bounding_box_clusters_3D[i][j]);
			}
			// Sort Bounnding Boxes from Closest to Furthest
			std::sort (current_dn_bounding_box.begin(), current_dn_bounding_box.end(), distance_compare_obj);

			int cluster_box_index_indicating_dobject = 0; // The closest
			
			// Ignore if the "person" detected is further than 8m
			if (current_dn_bounding_box[cluster_box_index_indicating_dobject].mean_z > DEPTH_MAX){
				continue;
			}

			
			dobject_3D_boxes.push_back(current_dn_bounding_box[cluster_box_index_indicating_dobject]);
		}

	}

	//std::cout << "Number of 3D Box dobjects: " << dobject_3D_boxes.size() << std::endl;

}




// void publish_dobject_bounding boxes
void Bounding_Box_dobject::publish_dn_points(){
	// Publish the voxelized points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr prism_points_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	prism_points_msg->header = (*cloud).header;
	int apply_color = 0;

	for(size_t vb_i = 0; vb_i < voxelized_bounding_box_points.size(); vb_i++){
		for(size_t pt_i = 0; pt_i < voxelized_bounding_box_points[vb_i].points.size(); pt_i++){
		    pcl::PointXYZRGB pt_color;
	    	pt_color.x = voxelized_bounding_box_points[vb_i].points[pt_i].x;
	      	pt_color.y = voxelized_bounding_box_points[vb_i].points[pt_i].y;
	      	pt_color.z = voxelized_bounding_box_points[vb_i].points[pt_i].z;

  	
	      	uint8_t r = (uint8_t) colormap[apply_color][0];
	        uint8_t g = (uint8_t) colormap[apply_color][1];
	        uint8_t b = (uint8_t) colormap[apply_color][2];    

	        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
			prism_points_msg->points.push_back(pt_color);
		}
		apply_color++;
		apply_color = apply_color % colormap.size();
	}

	prism_voxels_pub.publish(prism_points_msg);
}

void Bounding_Box_dobject::publish_clusters(){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_points_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	cluster_points_msg->header = (*cloud).header;

	// For each box
	for(size_t i = 0; i < bounding_boxes_clusters.size() ; i++){
		int apply_color = 0;
		// For each cluster in the box
//		std::cout << "number of boxes " <<  bounding_boxes_clusters.size() << std::endl;
		for(size_t j = 0; j < bounding_boxes_clusters[i].size() ; j++){
			int n_clusters = bounding_boxes_clusters[i][j].indices.size();
//			std::cout << "Cluster indices size " << n_clusters << std::endl;
			// For each point in the cluster
			for(size_t k = 0; k < n_clusters; k++){
				int box_index = i;
				int cluster_index = j;
				int pt_index = bounding_boxes_clusters[box_index][cluster_index].indices[k];

			    pcl::PointXYZRGB pt_color;
		    	pt_color.x = voxelized_bounding_box_points[box_index].points[pt_index].x;
		      	pt_color.y = voxelized_bounding_box_points[box_index].points[pt_index].y;
		      	pt_color.z = voxelized_bounding_box_points[box_index].points[pt_index].z;

				// Apply color to the cluster
		        uint8_t r = (uint8_t) colormap[apply_color][0];
		        uint8_t g = (uint8_t) colormap[apply_color][1];
		        uint8_t b = (uint8_t) colormap[apply_color][2];    

		        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
				cluster_points_msg->points.push_back(pt_color);
			}
			apply_color++; // Change color for the next cluster
			apply_color = apply_color % colormap.size();
		}
	}


	cluster_voxels_pub.publish(cluster_points_msg);


}

//
void Bounding_Box_dobject::publish_clusters_boxes(){
    cluster_boxes_array.markers.clear();
	delete_previous_markers();

	//std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_clusters_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster
    int marker_index = 0;
	for(size_t i = 0; i < bounding_box_clusters_3D.size() ; i++){
		int apply_color = marker_index;
		// For each cluster in the box
//		std::cout << "number of boxes " <<  bounding_boxes_clusters.size() << std::endl;
		for(size_t j = 0; j < bounding_box_clusters_3D[i].size() ; j++){
			cluster_boxes_array.markers.push_back(createBoundingBoxMarker(CLUSTER_BOXES_NAMESPACE, bounding_box_clusters_3D[i][j], marker_index, apply_color));
			marker_index++;
		}
	}
	cluster_boxes_array_pub.publish(cluster_boxes_array);

}

void Bounding_Box_dobject::publish_dobject_3D_boxes(){
    dobject_boxes_array.markers.clear();
	//delete_previous_markers();
    int marker_index = 0;
	int apply_color = 45;
	for(size_t i = 0; i < dobject_3D_boxes.size() ; i++){
		dobject_boxes_array.markers.push_back(createBoundingBoxMarker(dobject_BOXES_NAMESPACE, dobject_3D_boxes[i], marker_index, apply_color));
		marker_index++;
	}
	dobject_boxes_array_pub.publish(dobject_boxes_array);
}


void Bounding_Box_dobject::publish_dobject_points(){
	// Publish the voxelized points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dobject_points_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	dobject_points_msg->header = (*cloud).header;
	int apply_color = GREEN_COLOR;

	for(size_t i = 0; i < dobject_3D_boxes.size(); i++){
		int box_index = dobject_3D_boxes[i].voxel_box_index; 
		for(size_t j = 0; j < dobject_3D_boxes[i].voxel_indices.indices.size(); j++){
			int pt_index = dobject_3D_boxes[i].voxel_indices.indices[j];

		    pcl::PointXYZRGB pt_color;
	    	pt_color.x = voxelized_bounding_box_points[box_index].points[pt_index].x;
	      	pt_color.y = voxelized_bounding_box_points[box_index].points[pt_index].y;
	      	pt_color.z = voxelized_bounding_box_points[box_index].points[pt_index].z;

	      	
	        uint8_t r = (uint8_t) colormap[apply_color][0];
	        uint8_t g = (uint8_t) colormap[apply_color][1];
	        uint8_t b = (uint8_t) colormap[apply_color][2];    

	        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
			dobject_points_msg->points.push_back(pt_color);
		}

	}

	dobject_points_pub.publish(dobject_points_msg);
}

void Bounding_Box_dobject::publish_human_boxes_delayed()
{

human_boxes_array_pub.publish(human_markers_array);
	
	people_msgs::PositionMeasurementArray  people_array;
    people_array.header.stamp = ros::Time::now();
    people_array.people = people;
	People_pos_pub.publish(people_array);


}
//mk for human
void Bounding_Box_dobject::publish_human_boxes(){


 //    int marker_index = 0;
	// int apply_color = 45;
	// for(size_t i = 0; i < dobject_3D_boxes.size() ; i++){
	// 	dobject_boxes_array.markers.push_back(createBoundingBoxMarker(dobject_BOXES_NAMESPACE, dobject_3D_boxes[i], marker_index, apply_color));
	// 	marker_index++;
	// }
	// dobject_boxes_array_pub.publish(dobject_boxes_array);
    delete_previous_human_markers();
 	// std::cout << "number of boxes " <<  bounding_box_clusters_humans.size() << std::endl;
    people.clear();
    human_markers_array.markers.clear();
    int marker_index = 0;
	for(size_t i = 0; i < dobject_3D_boxes.size() ; i++){
		int apply_color = marker_index;
			human_markers_array.markers.push_back(createHumanMarker(HUMAN_BOXES_NAMESPACE, dobject_3D_boxes[i], marker_index, apply_color));
			createHumanPositionMarker(HUMAN_BOXES_NAMESPACE, dobject_3D_boxes[i], marker_index, apply_color);
			marker_index++;
		// ROS_INFO("dObject_3D_boxes_size : %d", i);
	}

	// ROS_INFO("publishing human_boxes 2");
	//human_boxes_array_pub.publish(human_markers_array);


}

void Bounding_Box_dobject::delete_previous_human_markers(){
    
     human_markers_array.markers.clear();
    //human_markers_array
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = (*cloud).header.frame_id; 
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = HUMAN_BOXES_NAMESPACE;//"cluster_3D_boxes";
    marker.id = 0;
    marker.action = 3;					//visualization_msgs::Marker::DELETEALL;
    human_markers_array.markers.push_back(marker);
    human_boxes_array_pub.publish(human_boxes_array);
}



void Bounding_Box_dobject::delete_previous_markers(){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = (*cloud).header.frame_id; 
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = CLUSTER_BOXES_NAMESPACE;//"cluster_3D_boxes";
    marker.id = 0;
    marker.action = 3;//visualization_msgs::Marker::DELETEALL;
    cluster_boxes_array.markers.push_back(marker);
    cluster_boxes_array_pub.publish(cluster_boxes_array);
}


void Bounding_Box_dobject::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	global_pose.resize(3);

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

	
	global_pose[0]=msg->pose.position.x;
	global_pose[1]=msg->pose.position.y;
	global_pose[2]=yaw_tf;


 	// ros::Duration(0.05).sleep();
	// global_pose_callback
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "villa_3D_object_bounding_boxes_human");
	Bounding_Box_dobject bbh_obj;


  	// Create Subsribers
	bbh_obj.yolo_detectedObjects_sub = bbh_obj.node.subscribe<tmc_yolo2_ros::Detections>("yolo2_node/detections", 10, boost::bind(&Bounding_Box_dobject::yolo_detected_obj_callback, &bbh_obj, _1));	
    bbh_obj.registered_cloud_sub = bbh_obj.node.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 10, boost::bind(&Bounding_Box_dobject::cloud_callback, &bbh_obj, _1));

	bbh_obj.prism_voxels_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/prism_voxels", 0 );
	bbh_obj.cluster_voxels_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/cluster_voxels", 0 );
	bbh_obj.cluster_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/cluster_boxes", 0 );
	bbh_obj.dobject_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/dobject_boxes_3D", 0 );
	bbh_obj.People_coordinate_pub= bbh_obj.node.advertise<geometry_msgs::Point>( "/People_3Dcoordinate", 0 );
	bbh_obj.People_pos_pub=bbh_obj.node.advertise<people_msgs::PositionMeasurementArray>( "/people_tracker_measurements_array", 0 );
	bbh_obj.One_People_pos_pub=bbh_obj.node.advertise<people_msgs::PositionMeasurement>("/people_tracker_measurements", 0 );
	//mk
	bbh_obj.human_box_pub=bbh_obj.node.advertise<visualization_msgs::Marker>( "/human_target", 0 );
	bbh_obj.human_boxes_array_pub=bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/human_boxes", 0 );
	//
    
	bbh_obj.dobject_points_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/dobject_voxels", 0 );

	bbh_obj.number_of_detected_dobjects_pub = bbh_obj.node.advertise<std_msgs::Int8>("detection/number_of_detected_dobjects", 0);

	bbh_obj.global_pos_sub= bbh_obj.node.subscribe<geometry_msgs::PoseStamped>("/global_pose", 100, boost::bind(&Bounding_Box_dobject::global_pose_callback, &bbh_obj, _1));

	bbh_obj.Joint_states_sub =bbh_obj.node.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &Bounding_Box_dobject::joint_states_callback,&bbh_obj);



	  ros::Rate loop_rate(10);

	while (ros::ok())
  {
	 	
		if(bbh_obj.boxes.size()>0){
		
			bbh_obj.publish_human_boxes_delayed();
     	}
      bbh_obj.publish_dobject_3D_boxes();
      bbh_obj.publish_dobject_points();
     // bbh_obj.Publish_human_boxes();
     ros::spinOnce();
     loop_rate.sleep();  
  }




	ros::spin();
  return 0;
}
