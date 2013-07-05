#ifndef __kinect__
#define __kinect__

#include "Includes/includes.h"

// Projekt
#include "RGBD/RGBDprocessing.h"


#define CLOUD_SIZE 12
class Kinect
{
public:
	// Parametry
	StrukturaParametrow Parameters;
	
	// Stream wyjsciowy do zapisu NARF keypoints
	ofstream zapis;
	
	//Bufory
	pcl::PointCloud<pcl::PointXYZ>::Ptr chmura[CLOUD_SIZE];
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr chmura_color[CLOUD_SIZE];
	vector<DeskryptorDouble> descriptors[CLOUD_SIZE];
	
	// Kinect - initial transformation
	Eigen::Matrix4f KinectInitial;
	
	// Lista poprawnych modeli
	list<Model> ListaPoprawnychModeli;
	
	// Czasy
	double StartTime, EndTime;



	// Konstruktor
	Kinect();	

	// Destruktor
	~Kinect();

	// Wczytanie parametrow
	void ReadParameters();
	
	// Ciagly zapis z Kinecta
	void GetContinuousClouds();
	
	// ICP
	pcl::PointCloud<pcl::PointXYZ>::Ptr ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr in2,Eigen::Matrix4f &final, double OutRejThre, double Epsilon, double Max_iter,double maxCorrespondeceDist);
	
	// Filtracja
	void Filtrowanie(int x);
	
	// Filtracja
	void FiltrowanieColor(int x);

	// Pobranie chmur z kinecta
	void PobierzZKinecta(int x);

	// Załadowanie chmur z plikow
	void ReadCloudsXYZRGBA(int x);
	
	// Załadowanie chmur z plikow old
	void ReadCloudsXYZ(int x);
	void ReadCloudsXYZ(int x, string name);
	
	// Deskrypcja + ICP
	void FeatureMatching();
	
	// Extracting NARF
	void ExtractNARF();
	
	// Wyliczenie deskryptorow
	void WyliczNARF();
	void WyliczNARFSURF();
	void WyliczSURF();
	void WyliczSURFSURF();
	void WyliczSURF2D();

	// New Class
	RGBDclass *RGBD;
	
	// probabilistic RANSAC
	int RANSACiter(double poprawnych, double wszystkich);
	
	
	// Error between 2 clouds
	void ComputeError();
	
	// Save Point Cloud
	void SavePointCloud(int which);
	void SavePointCloudColor(int which);

	void TestKabsch();
	
	// Return series of transformations
	void Series();
	
	void dbtest();
	void FeatureMatchingWithKnownCorrespondeces();
	
	
	/// Tracking
	void TrackingRun();
	void MatchingRun();
	void FeatureMatchingFromTracking();

};
#endif
