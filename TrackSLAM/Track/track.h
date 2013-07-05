#ifndef _TRA
#define _TRA

#include "../Includes/includes.h"


class Track
{
public:
	// Zmienne Mat do odbioru danych Depth
	cv::Mat frameD[3];
	
	// Zmienne Mat do odbioru danych RGB
	cv::Mat frameBGR[3];
	
	// Tracked points:
	// 0 - begin
	// 1 - last
	// 2 - most up-to-date
	vector<cv::Point2f> points[3];
	
	// Clare before next tracking
	void clearTrack();
	
	// Detection at start
	void newDetection();
	
	// Tracking after detection
	void doTracking();
	
	// Show tracked image
	void trackShow();
	
	// RANSAC and Kabsch
	Eigen::Matrix4f estimateTransformation(int pair_number, Constrain constrain, Eigen::Matrix4f KinectInitial);
	
	// My Runding
	int roundX(double X);
	int roundY(double Y);
	
	// Loading to track
	void LoadRGB(string name,int place);
	void LoadRGBD(string RGBname, string Dname, int place);

	// Euclidean distance of points
	double euclideanDistance( cv::Point2f p1, cv::Point2f p2 );
};

#endif _TRA
