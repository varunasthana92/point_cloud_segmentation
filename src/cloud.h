#include <iostream>
#include <opencv2/core.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <string>
#include <math.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;

cloudPtr createCloudFromImage(cv::Mat &RGBimage, cv::Mat &worldCord);
