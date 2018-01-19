#include <BoundingBoxDObject_nicolas.h>
#include <Cluster3DBoundingBoxDistanceCompare.h>

#include <ros/package.h>

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480

#define VOXEL_SIZE 0.01 //1cm voxels

#define CLUSTER_DIST_TOLERANCE 0.2

#define DEPTH_MAX 10.0

#define CLUSTER_BOXES_NAMESPACE "cluster_3D_boxes"
#define dobject_BOXES_NAMESPACE "dobject_3D_boxes"

#define GREEN_COLOR 0


Bounding_Box_dobject_nicolas::Bounding_Box_dobject_nicolas(): cloud(new pcl::PointCloud<pcl::PointXYZRGB>){
	init_vars();
	    // int tl_x = 0;
     //    int tl_y = 0;
     //    int width_bound = 200;
     //    int height_bound = 300;
     //    boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
     //    tl_x = 220;
     //    tl_y = 10;
     //    width_bound = 80;
     //    height_bound = 180;
     //    boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
}

bool Bounding_Box_dobject_nicolas::init_vars(){
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


Bounding_Box_dobject_nicolas::~Bounding_Box_dobject_nicolas(){}

void Bounding_Box_dobject_nicolas::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	srv.request.img_input = (*msg);
	received_image = true;
	// std::cout << "received_image bool to:" << received_image << std::endl;	
	}

void Bounding_Box_dobject_nicolas::yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg){
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
    std_msgs::Int8 num_msg;
    if(dobject_boxes_array.markers.size() >= boxes.size() ){;
        num_msg.data = boxes.size();
        number_of_detected_dobjects_pub.publish(num_msg);
    }
    }
            

/*    newObj.tl_x = left < 0 ? 0 : left;
    newObj.tl_y = top < 0 ? 0 : top;
    newObj.width = right - newObj.tl_x;
    newObj.height = bot - newObj.tl_y;*/
std::cout<< srv.request.rect_input.size()<<std::endl;
std::cout<<" get rect from yolo -> completed "<<std::endl;
}

void Bounding_Box_dobject_nicolas::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){
std::cout<< "trying to get pointcloud "<<std::endl;

	pcl::fromROSMsg(*msg, *cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_corners_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Set the same headers as original point cloud for simplicity   
	pt_corners_msg->header = (*cloud).header;
std::cout<< "trying to use grab service "<<std::endl;
	// 
	// For each box, cluster points using euclidean clustering
	// for each box, extract dobject points using euclidean clustering
	//
	if (boxes.size() > 0 && client.call(srv)){
		list_indices = srv.response.indices_output;
std::cout << "extracting points from pointcloud"<<std::endl;
		dn_extract_points(msg); 	// Extract points from each box.
		// voxelize_points(); 			// filter points from each box.
		// extract_clusters();			// Cluster the voxels
		// extract_candidate_dobject_boxes(); // Create bounding boxes on all the clusters
		// extract_dobject_boxes(); // Extract dobject boxes only;
std::cout<<"publishing points "<<std::endl;
		publish_objects_points();	
		// publish_dn_points();		// Visualize voxels bounded by dn box
		// publish_clusters(); 		//Visualize Clusters
		// publish_clusters_boxes();		
		// publish_dobject_3D_boxes();
		// publish_dobject_points();
	}
std::cout<< " finished cloud callback " << std::endl;

}

void Bounding_Box_dobject_nicolas::dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg){

	if (boxes.size() > 0){
		// bounding_box_points.clear();
		bounding_objects_points.clear();
		int ADD = 0;
		std::cout<< boxes.size() << " objects are seen"<<std::endl;
		bounding_objects_points.header = (*cloud).header;

		// For each box, extract pointcloud bounded by the darknet classifier. store pointcloud to a vector
		for (int b_i = 0; b_i < boxes.size(); b_i++){		

			pcl::PointXYZRGB pt_color;
			for (int ind = ADD + 1; ind < ADD + list_indices[ADD]+1; ind++)
			{
		    	pt_color.x = cloud->points[list_indices[ind]].x;
		      	pt_color.y = cloud->points[list_indices[ind]].y;
		      	pt_color.z = cloud->points[list_indices[ind]].z;
		        pt_color.rgb = cloud->points[list_indices[ind]].rgb;

		        // prism_points->points.push_back(pt_color);
		        bounding_objects_points.points.push_back(pt_color);

			}
			std::cout << "indice of ref = " <<  ADD << std::endl;
			std:: cout << "amount of indices proposed  :" << list_indices[ADD] <<std::endl;

			ADD = ADD + 1 + list_indices[ADD];

			std :: cout << " next iteration will start at indice : " << ADD+1<<std::endl;

		}//End of box for loop
	} 
}

void Bounding_Box_dobject_nicolas::publish_objects_points(){
	// for(size_t b_i = 0; b_i < bounding_box_points.size(); b_i++){
	// 	std::cout<<b_i<<std::endl;
		// Objects_points_pub.publish(bounding_box_points[b_i]);
	// }
	Objects_points_pub.publish(bounding_objects_points);
}


void Bounding_Box_dobject_nicolas::voxelize_points(){
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

void Bounding_Box_dobject_nicolas::extract_clusters(){
	bounding_boxes_clusters.clear();
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
    	ec.setMaxClusterSize (50000);
    	ec.setSearchMethod (tree);
    	ec.setInputCloud ( xyz_prism_points );
    	ec.extract (cluster_indices);
    	bounding_boxes_clusters.push_back(cluster_indices);
	    //std::cout << "Box id " << b_i << " has " << cluster_indices.size() << " clusters" << std::endl;
    }
//    std::cout << "Number of Bounding Boxes Clusters: " << bounding_boxes_clusters.size() << std::endl;
}


void Bounding_Box_dobject_nicolas::extract_candidate_dobject_boxes(){
	bounding_box_clusters_3D.clear();
	// For each DN Box, extract the candidate dobjects
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
				vector_of_boxes_for_cluster.push_back(Cluster3D_BoundingBox(min_x, max_x, 
			  				min_y, max_y, min_z, max_z, mean_x, mean_y, mean_z,
							box_index, bounding_boxes_clusters[box_index][cluster_index]) );
			} // end if statement
		
		} // Finished with this cluster
		 bounding_box_clusters_3D.push_back(vector_of_boxes_for_cluster);
	} // Finished with this DN box
}

visualization_msgs::Marker Bounding_Box_dobject_nicolas::createBoundingBoxMarker(const std::string &array_namespace,
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
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = std::abs(box.x_max - box.x_min);
    marker.scale.y = std::abs(box.y_max - box.y_min);
    marker.scale.z = std::abs(box.z_max - box.z_min);

    // Set the color -- be sure to set alpha to something non-zero!
    //int	apply_color = marker_index % colormap.size();

    marker.color.r = ((float) colormap[apply_color][0]) / 255.0;
    marker.color.g = ((float) colormap[apply_color][1]) / 255.0;
    marker.color.b = ((float) colormap[apply_color][2]) / 255.0;
    marker.color.a = 0.25;

    marker.lifetime = ros::Duration(1.0);

    return marker;
}


void Bounding_Box_dobject_nicolas::extract_dobject_boxes(){
    Cluster3D_BoundingBox_distance_compare distance_compare_obj;
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
			
			// Ignore if the "person" detected is further than 4m
			if (current_dn_bounding_box[cluster_box_index_indicating_dobject].mean_z > DEPTH_MAX){
				continue;
			}
	/*
			// Select only the top 3 closest boxes
			//current_dn_bounding_box.resize(CUTOFF_NUM);
				
			// Filter for minimum box size and maximum box height
			std::vector<int> indices_to_consider;
			for(size_t k = 0; k < current_dn_bounding_box.size() ; k++){
				float cb_size_x = current_dn_bounding_box[k].box_x_size();
				float cb_size_y = current_dn_bounding_box[k].box_y_size();			
				float cb_size_z = current_dn_bounding_box[k].box_z_size();
				float cb_max_height =  current_dn_bounding_box[k].y_max;

		       if ( ((cb_size_x >= BOX_WIDTH_TOL) && 
		       	     (cb_size_y >= BOX_HEIGHT_TOL) && 
		       	     (cb_size_z >= BOX_DEPTH_TOL)  && 
		       	     (cb_size_x < BOX_WIDTH_MAX_TOL) &&
		       	     (cb_size_y < BOX_HEIGHT_MAX_TOL) &&
	   	       	     (cb_size_z < BOX_DEPTH_MAX_TOL) &&    	       	     
		       	     (std::abs(cb_max_height + KINECT_HEIGHT) < MAX_HEIGHT_TOL) ) ){
		       			indices_to_consider.push_back(k);
		       }

			}

			
			// If no box satisfies this requirement, claim that the closest cluster must be dobject

			if (indices_to_consider.size() > 0){
				// There must be competing boxes. The tallest box at this point should be dobject
				// Now only select the box with the highest 
				float tallest_box_so_far = std::abs(current_dn_bounding_box[0].y_max); // highest so far
				int best_box_index = 0;
				for(size_t b_i = 0; b_i < indices_to_consider.size(); b_i++){

		  			 std::cout << "INDEX: " << b_i << " " << "dx dy dz" << 
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].x_max) << " " <<
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].y_max) << " " <<
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].z_max) << " " << std::endl;


					if (  std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].y_max) > tallest_box_so_far){
						best_box_index = indices_to_consider[b_i];
						tallest_box_so_far = std::abs(current_dn_bounding_box[b_i].y_max);
					}

				}
				cluster_box_index_indicating_dobject = best_box_index;
			}
			
			*/
			
			dobject_3D_boxes.push_back(current_dn_bounding_box[cluster_box_index_indicating_dobject]);
		}

	}

	//std::cout << "Number of 3D Box dobjects: " << dobject_3D_boxes.size() << std::endl;

}




// void publish_dobject_bounding boxes


void Bounding_Box_dobject_nicolas::publish_dn_points(){
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

void Bounding_Box_dobject_nicolas::publish_clusters(){

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
void Bounding_Box_dobject_nicolas::publish_clusters_boxes(){
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

void Bounding_Box_dobject_nicolas::publish_dobject_3D_boxes(){
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


void Bounding_Box_dobject_nicolas::publish_dobject_points(){
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


void Bounding_Box_dobject_nicolas::delete_previous_markers(){
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
