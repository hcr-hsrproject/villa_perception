#ifndef CLUSTER_3D_BOUNDING_BOX_H
#define CLUSTER_3D_BOUNDING_BOX_H

#include <pcl/PointIndices.h>

class Mean3DTrackingBox{
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

	Mean3DTrackingBox();
	Mean3DTrackingBox(const float _x_min, const float _x_max, 
						  const float _y_min, const float _y_max, 
						  const float _z_min, const float _z_max,
						  const float _mean_x, const float _mean_y, const float _mean_z,
						  const int _voxel_box_index, const pcl::PointIndices &_voxel_indices);

	Mean3DTrackingBox(const float _mean_x, const float _mean_y, const float _mean_z);	
	~Mean3DTrackingBox();	

};

#endif
