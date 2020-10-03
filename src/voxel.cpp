#include "voxel.h"

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;

void downSampleVoxel(const cloudPtr &currCloud, cloudPtr &currVoxelCloud){
	pcl::VoxelGrid<pcl::PointXYZRGB>vg;
	vg.setInputCloud(currCloud);
	vg.setLeafSize( 0.01f, 0.01f, 0.01f);
	vg.filter(*currVoxelCloud);
	return;
}


