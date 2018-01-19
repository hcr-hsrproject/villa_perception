#include <Cluster3DBoundingBox.h>

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
