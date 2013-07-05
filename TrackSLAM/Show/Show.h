#ifndef _SHO
#define _SHO

#include "../Includes/includes.h"

// Show Cloud XYZ
void ShowCloudXYZ2(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, string name);
void ShowCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud);

// Show Cloud XYZRGBA
void ShowCloudXYZRGBA2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud, string name);
void ShowCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud);

// Show Image
void ShowImage2(cv::Mat & img, string name);
void ShowImage(cv::Mat & img);

// Show SumCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr ShowSumCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2 ); 
	
// Show 2Clouds
void Show2Clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2 ); 

#endif
