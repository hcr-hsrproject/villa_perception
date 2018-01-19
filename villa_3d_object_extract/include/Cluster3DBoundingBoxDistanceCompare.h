#ifndef CLUSTER_3D_BOUNDING_BOX_DISTANCE_COMPARE_H
#define CLUSTER_3D_BOUNDING_BOX_DISTANCE_COMPARE_H

#include <Cluster3DBoundingBox.h>

struct Cluster3D_BoundingBox_distance_compare {
	bool operator() (const Cluster3D_BoundingBox &lhs, const Cluster3D_BoundingBox &rhs) const{ 
		float lhs_dist = lhs.distance_from_origin();
		float rhs_dist = rhs.distance_from_origin();		 
		return lhs_dist < rhs_dist;	
	}

};

#endif
