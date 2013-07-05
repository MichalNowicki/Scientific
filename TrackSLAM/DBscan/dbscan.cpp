#include "dbscan.h"

inline double distance(float &x, float &x2, float &y, float &y2)
{
	return ((x-x2)*(x-x2) + (y-y2)*(y-y2));
	
}
DBScan::DBScan(){
}
void DBScan::Run(vector<cv::KeyPoint> & BadanyZbior, double eps, int MinPts,vector<int> & cluster)
{
	int SizeOfZbior = BadanyZbior.size();
	
	// Similarity
	double **dist;
	
	dist = new double *[SizeOfZbior] ;
	for( int i = 0 ; i < SizeOfZbior; i++ )
		dist[i] = new double[SizeOfZbior];
		
	for(int i=0;i<SizeOfZbior;i++)
		for(int j=0;j<SizeOfZbior;j++)
			dist[i][j] = distance(BadanyZbior[i].pt.x, BadanyZbior[j].pt.x, BadanyZbior[i].pt.y, BadanyZbior[j].pt.y);
	
	// Run
	int C = 0;
	bool * visited;
	visited = new bool[SizeOfZbior]();
	
	bool * noise;
	noise = new bool[SizeOfZbior]();
	
	for(int i=0;i<SizeOfZbior;i++) cluster.push_back(0);
	
	// For all points
	for (int i=0;i<SizeOfZbior;i++)
	{
		if(visited[i] != true)
		{
				visited[i] = true;
				vector<int> indices;
				
				for(int k=0;k<SizeOfZbior;k++)
				{
					if(dist[i][k] < eps)
					{
						indices.push_back(k);
					}
				}
				
				if(indices.size() < MinPts) noise[i] = true;
				else
				{
					C++;
					// EXPAND CLUSTER
					cluster[i] = C;
					for (int j=0;j<indices.size();j++)
					{
							int x = indices[j];
							if(visited[x] != 1)
							{
									visited[x] = 1;
									vector<int> indices2;
									
									for(int k=0;k<SizeOfZbior;k++)
									{
										if(dist[x][k] < eps)
										{
											indices2.push_back(k);
										}
									}
									
									// Sasiad sasiada jest moim sasiadem
									if(indices2.size() >= MinPts) 
									{
										for(int g=0;g<indices2.size();g++) indices.push_back(indices2[g]);
									}
							}
							if(cluster[x] == 0) cluster[x] = C;
					}
				}
		}
	}
	
	// Clearing
	delete [] visited;
	for( int i = 0 ; i < SizeOfZbior ; i++ )
		delete [] dist[i] ;
	delete [] dist ;
}
