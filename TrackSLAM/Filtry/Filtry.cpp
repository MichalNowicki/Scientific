#include "Filtry.h"

// AntyNaN
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrAntyNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(Cloud);
	pass.filter(*Output);
	
	return Output;
}

// VoxelGrid
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrVoxelGrid2(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float LeafSize)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::VoxelGrid<pcl::PointXYZ> Voxel;
	Voxel.setInputCloud (Cloud);
	Voxel.setLeafSize (LeafSize, LeafSize, LeafSize);
	Voxel.filter (*Output);
	
	return Output;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud)
{
	return FiltrVoxelGrid2(Cloud,0.01);
}

// AntySzum
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrAntySzum2(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float Mean, float Stddev)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (Cloud);
	sor.setMeanK (Mean);
	sor.setStddevMulThresh (Stddev);
	sor.filter (*Output);
	
	return Output;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr FiltrAntySzum2(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud)
{
	return FiltrAntySzum2(Cloud,50,1.0);
}

/// KOLOR
// AntyNaN
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FiltrAntyNaN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Output (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(Cloud);
	pass.filter(*Output);
	
	return Output;
}

// VoxelGrid
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FiltrVoxelGrid2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud, float LeafSize)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Output (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	pcl::VoxelGrid<pcl::PointXYZRGBA> Voxel;
	Voxel.setInputCloud (Cloud);
	Voxel.setLeafSize (LeafSize, LeafSize, LeafSize);
	Voxel.filter (*Output);
	
	return Output;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FiltrVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud)
{
	return FiltrVoxelGrid2(Cloud,0.01);
}


