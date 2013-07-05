#ifndef _RGBD
#define _RGBD

#include "../Includes/includes.h"

#define size_width 640
#define size_height 480

class RGBDclass
{
public:

	// Parametry
	int HessianThresholdRGB, HessianThresholdD;

	// Zmienne Mat do odbioru danych Depth
	cv::Mat frameD[3],frameD_X[2], frameDdiscrete[2];
	
	// Zmienne Mat do odbioru danych RGB
	cv::Mat frameBGR[3], frameRGB;
	
	// Time detection + extraction
	double time;
	
	vector< pcl::PointXYZ> Keypoints3d[2];
	cv::Mat descriptors[4];

// Metody

	// Konstruktor
	RGBDclass();
	
	/// Adjusting frame to show
	void FrameDiscretisation(int which);
	void RemovingNaNs(int which);
	
	/// (2*size+1)x(2*size+1) mask at (b,a)
	double depthMask(int which, double a, double b, int size);
	
	/// Extracting
	vector<DeskryptorDouble> ExtractSURF(int which);
	vector<DeskryptorDouble> ExtractSURF_RGB(int which);
	void DetectSTAR(cv::Mat obraz);
	void ExtractProposed(int which, double eps, int minPts, int ile_RGB, int ile_D);
	
	
	// Converting RGBD->pointcloud && pointclouds->RGBD
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BuildPointCloudFromRGBD(int which);
	void BuildRGBDfromPointCloud(int which, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
	
	// Loading data
	void GetDataFromKinect(int which, int h);
	void GetContinuousRGBD();
	void LoadRGB(string name, int place);
	void LoadRGBD(int which);
	void LoadRGBD2(string name,string name2,int which);
	
	// Save 
	void SaveRGBD(int which);
	void SaveRGBD(int indeks,int which);
	void SaveFrameDasPNG(int indeks,int which);
	
};


#endif
