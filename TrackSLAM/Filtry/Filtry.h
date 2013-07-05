#ifndef _FIL
#define _FIL

#include "../Includes/includes.h"

// AntyNaN
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrAntyNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud);

// VoxelGrid
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrVoxelGrid2(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float LeafSize);
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud);

// AntySzum
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrAntySzum2(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float Mean, float Stddev);
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrAntySzum(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud);


/// KOLOR
// AntyNaN
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FiltrAntyNaN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud);

// VoxelGrid
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FiltrVoxelGrid2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud, float LeafSize);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FiltrVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud);

#endif
