#include "Show.h"

// Show Cloud XYZ
void ShowCloudXYZ2(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, string name)
{
	/*cout<<"Showing PointCloud without colors"<<endl;
	cout<<"Number of points : "<<PointCloud->points.size()<<endl;
	
	pcl::visualization::CloudViewer viewer(name.c_str());
	viewer.showCloud (PointCloud); 
	while (!viewer.wasStopped ())
	{
	}*/
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (PointCloud, 255, 0, 0);
	viewer.addPointCloud (PointCloud, point_cloud_color_handler, "original point cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
		  
	viewer.initCameraParameters ();
		
	setViewerPose(viewer);  
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}	
}

void ShowCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud)
{
	ShowCloudXYZ2(PointCloud, "Chmura punktow bez kolorow");
}

// Show Cloud XYZRGBA
void ShowCloudXYZRGBA2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud, string name)
{
	cout<<"Showing Color PointCloud "<<endl;
	cout<<"Number of points : "<<PointCloud->points.size()<<endl;
	
	pcl::visualization::CloudViewer viewer(name.c_str());
	viewer.showCloud (PointCloud);
	while (!viewer.wasStopped ())
	{
	}	
}

void ShowCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud)
{
	ShowCloudXYZRGBA2(PointCloud, "Chmura punktow RGB");
}


// Show Image
void ShowImage2(cv::Mat & img, string name)
{
	IplImage ImgShow = img;
	cvNamedWindow(name.c_str(), 1);
	cvShowImage(name.c_str(), &ImgShow);
	cvWaitKey();
}
void ShowImage(cv::Mat & img)
{
	ShowImage2(img, "Image");
}

// Show SumCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr ShowSumCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2 )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr SumCloud (new pcl::PointCloud<pcl::PointXYZ>);
	*SumCloud = *SumCloud + *PointCloud;
	*SumCloud = *SumCloud + *PointCloud2;
	
	ShowCloudXYZ(SumCloud);
	return SumCloud;
}

// Show 2Clouds
void Show2Clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2 )
{
	// Wizualizacja nalozonych chmur punktow
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (PointCloud, 255, 0, 0);
	viewer.addPointCloud (PointCloud, point_cloud_color_handler, "original point cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
		  
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler2 (PointCloud2, 0, 255, 0);
	viewer.addPointCloud (PointCloud2, point_cloud_color_handler2, "original point cloud2");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud2");
	viewer.initCameraParameters ();
		
	setViewerPose(viewer);  
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}	
}
