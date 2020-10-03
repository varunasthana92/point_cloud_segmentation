#ifndef _INCLUDE_VOXEL_
#define _INCLUDE_VOXEL_

#include <pcl/io/ply_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h> //for voxelgrid

void downSampleVoxel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &currCloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &currVoxelCloud);

#endif  /* _INCLUDE_VOXEL_ */