#ifndef _DBSCAN
#define _DBSCAN

#include "../Includes/includes.h" 
class DBScan 
{
public:	
	DBScan();
	
	// Wylicza wektor klas (cluster) na podstawie badanego zbioru (BadanyZbior) i parametrów:
	// max odleglosci sasiada (eps)
	// minimalnej liczby punktów na klaster (MinPts)
	void Run(vector<cv::KeyPoint> & BadanyZbior, double eps, int MinPts, vector<int> & cluster);
};
#endif
