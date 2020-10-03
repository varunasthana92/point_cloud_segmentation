#include "cloud.h"

cloudPtr createCloudFromImage(cv::Mat &RGBimage, cv::Mat &worldCord) {
	cloudPtr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB pt;
	int w = RGBimage.size().width;
	size_t length = worldCord.size().width;

	cv::Vec3b color;
	for(auto i=0; i<length; i++) {
		color = RGBimage.at<cv::Vec3b>(i/w, i%w); 
		pt.r = color[2]; pt.g = color[1]; pt.b = color[0];
		pt.x = worldCord.at<float>(0,i); pt.y = worldCord.at<float>(1,i);
		pt.z = worldCord.at<float>(2,i);
		cloud->push_back(pt);
	}
	// pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	// viewer.showCloud(cloud);
	// while (!viewer.wasStopped())
	// {
	// }	
	return cloud;
}
