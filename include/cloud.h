#ifndef _INCLUDE_CLOUD_
#define _INCLUDE_CLOUD_

#include <iostream>
#include <opencv2/core.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
// #include <string>
// #include <math.h>
// #include <vector>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr createCloudFromImage(cv::Mat &RGBimage, cv::Mat &worldCord);

#endif /* _INCLUDE_CLOUD_ */