#include "track.h"


void Track::clearTrack()
{
	for (int i=0;i<3;i++) points[i].clear();
}


void Track::newDetection()
{
	double StartTime = pcl::getTime ();
	
	vector<cv::KeyPoint> keypoints;
	int ILE_TRACK = 400;
	
	
	//cv::Ptr<cv::FeatureDetector> detector(new cv::DynamicAdaptedFeatureDetector ( new cv::FastAdjuster(25,true),240, 300));
	//detector->detect( frameBGR[0], keypoints);
	
	cv::Ptr<cv::FeatureDetector> detector(new cv::DynamicAdaptedFeatureDetector (new cv::FastAdjuster(25,true),(ILE_TRACK-100)/6, ILE_TRACK/6 ));
	
	for(int i=0;i<6;i++)
	{
		vector<cv::KeyPoint> keypointsTMP;
		cv::Mat roi( frameBGR[0] , cv::Rect(0,i*80,640,80));
		detector->detect(roi, keypointsTMP);
		
		sort(keypointsTMP.begin(), keypointsTMP.end(), keyRESPONSEcompare);
		
		for (int j=0;j< ILE_TRACK/6 && j < keypointsTMP.size();j++)
		{
			keypointsTMP[j].pt.y += i*80;
			keypoints.push_back(keypointsTMP[j]);
		}
		
	}
	
	
	
	int ile_RGB = keypoints.size();
	DBScan dbscan;
	vector<int> clusters;
	// eps / minpts
	dbscan.Run(keypoints, 30, 3, clusters);
	
	vector< pair< cv::KeyPoint, int > > key_temp;
	for(int i=0;i<keypoints.size();i++)
		key_temp.push_back( make_pair(keypoints[i], clusters[i]) );
	
	sort (key_temp.begin(), key_temp.end(), keypointscompare);
	
	

	int liczba_grup = 0;
	for (int i=0;i<clusters.size();i++)
	{
		if(clusters[i] > liczba_grup ) liczba_grup = clusters[i];
	}
	liczba_grup++;
	int ile_wybrac_z_grupy = 2;
	int ile_bezgrupy = ILE_TRACK - ile_wybrac_z_grupy*liczba_grup;
	
	cout<< "LG: " << liczba_grup<<endl;
	
	keypoints.clear();
	cout<< "BEFORE dbscan: "<< key_temp.size()<<endl;
	cv::Point2f chosen;
	for (int i=0,w_grupie = 0,last = -1, keypoints_index = 0;i<key_temp.size();i++)
	{
		// Different labels - starting new group
		if ( last != key_temp[i].second ) w_grupie = 0;
		
		// It has been labeled		
		if ( key_temp[i].second != 0 )
		{	
			// Still new to put new objects to the group
			while( w_grupie < ile_wybrac_z_grupy)
			{
				if ( w_grupie == 0 )
				{
					keypoints.push_back(key_temp[i].first);
					last = key_temp[i].second;
					w_grupie++;
					keypoints_index ++;
					chosen = key_temp[i].first.pt;
				}
				// Adding the 2nd one to the group -> the distance has to be bigger than the threshold
				else if ( w_grupie != 0 &&  euclideanDistance(key_temp[i].first.pt , chosen ) > 40)						
				{
					cout<<"XXXXXXX"<<endl;
					keypoints.push_back(key_temp[i].first);
					last = key_temp[i].second;
					w_grupie++;
					keypoints_index ++;
				}
				i++;
				
				// The group ended
				if(key_temp[i].second != key_temp[i-1].second)
				{
					i--;
					break;
				}
			}
		}
		else
		{
			while( w_grupie < ile_bezgrupy )
			{
				keypoints.push_back(key_temp[i].first);
				last = key_temp[i].second;
				w_grupie++;
				keypoints_index ++;
				i++;
				if(key_temp[i].second != key_temp[i-1].second)
				{
					i--;
					break;
				}
			}
			break;
		}
	}
	cout<< "AFTER dbscan: "<< keypoints.size()<<endl;	
	
	sort(keypoints.begin(), keypoints.end(), keyRESPONSEcompare);
	
	
	points[0].clear();
	points[1].clear();
	points[2].clear();
	for (int i=0;i<keypoints.size() && i < ILE_TRACK;i++)
		points[0].push_back(keypoints[i].pt);
	
	cout<<"BEF subpix det"<<endl;
	cv::Mat tmp;
	cvtColor(frameBGR[0], tmp, CV_RGB2GRAY);
	cornerSubPix( tmp, points[0], cvSize( 11, 11 ), 
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	cout<<"AFT subpix det"<<endl;
	
	// Indirect copy
	points[1] = points[0];
	frameBGR[0].copyTo(frameBGR[1]);
	
	double EndTime = pcl::getTime ();
	cout<<"Detect to track time: " << EndTime - StartTime << endl;
}

/// TRACKing
void Track::doTracking()
{
	double StartTime = pcl::getTime ();
	vector<uchar> status;
    vector<float> err;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
    cv::Size winSize(31,31);
    
    // Calculating movement of features
	cv::calcOpticalFlowPyrLK(frameBGR[1], frameBGR[2], points[1], points[2], status, err, winSize, 3, termcrit, 0, 0.001);
	
	cv::Mat tmp;
	cout<<"PRZED subpix"<<endl;
	//cvtColor(frameBGR[2], tmp, CV_RGB2GRAY);
	//cornerSubPix( tmp, points[2], cvSize( 11, 11 ), 
	//			cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	cout<<"PO subpix"<<endl;
	
	int i = status.size() - 1;
	std::vector<cv::Point2f>::reverse_iterator it, it2;
	for (it = points[0].rbegin(), it2 = points[2].rbegin(); it != points[0].rend(); ++it, ++it2, i--)
	{
		if ( status[i] == 0 )
		{
				points[0].erase((it+1).base()); 
				points[2].erase((it2+1).base());
		}
	}
	
	std::swap(points[1], points[2]);
    swap(frameBGR[1], frameBGR[2]);
    double EndTime = pcl::getTime ();
    cout<<"Tracking time: "<<EndTime-StartTime<<"\t\t Cech : "<<points[1].size()<<endl;
}

void Track::trackShow()
{
    for(int i = 0; i < points[1].size(); i++ )
    {
		cv::circle( frameBGR[1], points[1][i], 3, cv::Scalar(0,255,0), -1, 8);
	}
    cv::imshow("LK Demo", frameBGR[1]);
    cv::waitKey(250);
}

int Track::roundX(double X)
{
	if ( X < 0 ) X = 0;
	else if ( X > 639 ) X = 639;
	return round(X);
}

int Track::roundY(double Y)
{
	if ( Y < 0 ) Y = 0;
	else if ( Y > 479 ) Y = 479;
	return round(Y);
}

Eigen::Matrix4f Track::estimateTransformation(int pair_number, Constrain constrain, Eigen::Matrix4f KinectInitial)
{
	// PARAMETRY FREIBURG
	double FX = 517.3, FY = 516.5, CX = 318.6, CY = 255.3;
	
	//cout<<"Poczatkowa liczba cech: " << points[0].size()<<endl;
	
	double StartTime = pcl::getTime(), EndTime;
	// Removing features without depth
	vector<cv::Point2f>::iterator it, it2;
	for (it = points[0].begin(), it2 = points[1].begin(); it != points[0].end(); ++it,++it2)
	{	
		
		pcl::PointXYZ a,b;
		a.x = a.y = b.x = b.y = 0; 
		a.z = frameD[0].at<float>(roundY(it->y),roundX(it->x));
		b.z = frameD[2].at<float>(roundY(it2->y),roundX(it2->x));
		if(pcl::isFinite(a) == false || pcl::isFinite(b) == false)
		{
			points[0].erase(it);
			points[1].erase(it2);
		}
	}
	//cout<<"Bez NaN liczba cech wynosi: " << points[0].size()<< " " << points[1].size()<<endl;
	
	/*LoadRGBD("1.png","1.xml",0);
	for(int i = 0; i < points[0].size(); i++ )
    {
		cv::circle( frameBGR[0], points[0][i], 3, cv::Scalar(255,255,0), -1, 8);
	}
	cv::imshow("LK Demo", frameBGR[0]);
    cv::waitKey(100);*/

    
	
	Eigen::Matrix4f BestTransformation;
	int BestLiczbaDopasowan = 0;
	vector<int> valid_indexes;
	vector<int> model;
	vector<int> NajlepszeIndeksy;
	// Glowna petla RANSACa
	int ile_iteracji = 500;
	
	

	int ile_pkt = points[1].size();
	
	vector<pcl::PointXYZ> point3d[2];
	for (int i=0;i<ile_pkt;i++)
	{
		pcl::PointXYZ xyz;
		cv::Point2f a = points[0][i];

		xyz.x = -(CX-a.x)*frameD[0].at<float>(roundY(a.y),roundX(a.x))/ FX;
		xyz.y = -(CY-a.y)*frameD[0].at<float>(roundY(a.y),roundX(a.x))/ FY;
		xyz.z = frameD[0].at<float>(roundY(a.y),roundX(a.x));
	
		point3d[0].push_back(xyz);
				
		a = points[1][i];
		xyz.x = -(CX-a.x)*frameD[2].at<float>(roundY(a.y),roundX(a.x))/ FX;
		xyz.y = -(CY-a.y)*frameD[2].at<float>(roundY(a.y),roundX(a.x))/ FY;
		xyz.z = frameD[2].at<float>(roundY(a.y),roundX(a.x));
		
		point3d[1].push_back(xyz);
	}
	
	
	vector<bool> inliers, bestinliers;
	for (int i=0;i<ile_pkt;i++)	
		inliers.push_back(false);
	
	// RANSAC
	for(int i=0;i<ile_iteracji;i++)
	{
			valid_indexes.clear();
			model.clear();
			
			for(int j=0;j< ile_pkt ;j++) valid_indexes.push_back(true);
			
			// Znajdz pary uzyte do stworzenia modelu
			while(model.size() < pair_number)
			{
				int a = rand() % ile_pkt;
				
				if(valid_indexes[a] == true)
				{
					valid_indexes[a] = false;
					model.push_back(a);
				}
			}
				
			// Po wyborze musimy policzyc najlepsza transformacje dla danego modelu
			// Kabsch
			
			//cout << "KABSCH " <<endl;
			
			// Zapisać punkty wejściowe w wektorze
			Eigen::MatrixXf P(pair_number,3),Q(pair_number,3);
			
			for(int j=0;j<pair_number;j++)
			{
				int p = model[j];
		
				P(j, 0) = point3d[0][p].x;
				P(j, 1) = point3d[0][p].y;
				P(j, 2) = point3d[0][p].z;
								
				Q(j, 0) = point3d[1][p].x;
				Q(j, 1) = point3d[1][p].y;
				Q(j, 2) = point3d[1][p].z;	
			}

			Eigen::Matrix4f OPT;
			// Wyliczenie optymalnej transformacji do OPT
			Kabsch(P,Q,OPT, pair_number);

			
			
			Eigen::Vector3f teemp;
	
			teemp.head<3>() = OPT.block<1,3>(3,0);


			int LiczbaDopasowan = 0;
			
			// Sprawdzamy miare dopasowania
			for(int w=0;w<ile_pkt;w++)
			{				
				pcl::PointXYZ xyz;
				Eigen::VectorXf punkt(4);
				
				punkt[0] = point3d[0][w].x;
				punkt[1] = point3d[0][w].y;
				punkt[2] = point3d[0][w].z;
				punkt[3] = 1;
				
				// Zastosowanie optymalnej transformacji
				punkt = OPT * punkt;
				
				// Przepisanie nowej wartości punktu do chmury
				xyz.x = punkt[0];
				xyz.y = punkt[1];
				xyz.z = punkt[2];
				
				pcl::PointXYZ xyz2;
				xyz2.x = point3d[1][w].x;
				xyz2.y = point3d[1][w].y;
				xyz2.z = point3d[1][w].z;
	
	
				if ( SquaredNorm(xyz,xyz2) < 0.01)
				{
					LiczbaDopasowan++ ;
					inliers[w] = true;
				}
				else
				{
					inliers[w] = false;
				}
			}
			
			float Obrot[3];
			Oblicz_Katy(OPT, Obrot);
	
			
			// Porownanie czy spelnia warunki podane w pliku .cfg i jest lepszym modelem od najlepszego
			if(Obrot[0] <= constrain.maxObrX && Obrot[0] >= constrain.minObrX && Obrot[1] <= constrain.maxObrY &&
				Obrot[1] >= constrain.minObrY && Obrot[2] <= constrain.maxObrZ && Obrot[2] >= constrain.minObrZ &&
				teemp[0] <= constrain.maxTx && teemp[0] >= constrain.minTx && teemp[1] <= constrain.maxTy && 
				teemp[1] >= constrain.minTy && teemp[2] <= constrain.maxTz && teemp[2] >= constrain.minTz)
			{
				
				if(LiczbaDopasowan >  BestLiczbaDopasowan)
				{
					BestTransformation = OPT;
					BestLiczbaDopasowan  = LiczbaDopasowan;
					NajlepszeIndeksy.clear();
					bestinliers = inliers;
					for(int j=0;j<pair_number;j++)
					{
						NajlepszeIndeksy.push_back(model[j]);
					}
				}
			}
			
			if ( BestLiczbaDopasowan * 1.0 / ile_pkt > 0.5 ) break;
	}
	EndTime = pcl::getTime();
	cout<<endl<<"TIME TAKEN : " << EndTime  - StartTime<<endl;
	cout<<endl<<"Liczba Dopasowanych: " << BestLiczbaDopasowan << " na " << points[1].size()<<endl;
	for(int i=0;i<NajlepszeIndeksy.size();i++)
	{
		//cout<<"PARA: "<<NajlepszeIndeksy[i]<<" - ";
	}

	//cout<<endl<<endl<<"TRANSFORMACJA :"<<endl<<BestTransformation<<endl;
	float Obrot[3];
	
		
	Oblicz_Katy(BestTransformation, Obrot);
	//cout<<"Obroty: "<<Obrot[0]<< " " << Obrot[1]<< " "<<Obrot[2]<<endl;		
	
	
	
	// Again estimation from inliers
	Eigen::MatrixXf P(BestLiczbaDopasowan,3),Q(BestLiczbaDopasowan,3);
	Eigen::Matrix4f OPT;
	for (int i=0, k = 0;i<ile_pkt;i++)
	{
		if (bestinliers[i] == true)
		{
			P(k, 0) = point3d[0][i].x;
			P(k, 1) = point3d[0][i].y;
			P(k, 2) = point3d[0][i].z;
								
			Q(k, 0) = point3d[1][i].x;
			Q(k, 1) = point3d[1][i].y;
			Q(k, 2) = point3d[1][i].z;
			
			k++;	
		}
	}
	Kabsch(P,Q,OPT, BestLiczbaDopasowan);
	cout<<endl<<"TRANSFORMACJA z inlierow :"<<endl<<OPT<<endl;
		
	Oblicz_Katy(OPT, Obrot);
	cout<<"Obroty z inlierow: "<<Obrot[0]<< " " << Obrot[1]<< " "<<Obrot[2]<<endl;		
	
	OPT = KinectInitial * OPT.inverse();
	
	Eigen::Quaternion<float> Quat(OPT.block<3,3>(0,0));
	cout<<"RES" << OPT(0,3)<<" "<<OPT(1,3)<<" "<<OPT(2,3)<<" "<<Quat.coeffs().x()<< " "<<Quat.coeffs().y()<<" "<<Quat.coeffs().z()<< " "<<Quat.coeffs().w()<<endl;
	
	
	return OPT;
}


void Track::LoadRGB(string name,int place)
{
	frameBGR[place] = cv::imread(name);	
}


void Track::LoadRGBD(string RGBname, string Dname, int place)
{
	LoadRGB(RGBname,place);
	
	cv::FileStorage odczyt;
	odczyt.open(Dname, cv::FileStorage::READ);
	odczyt["frameD"] >>frameD[place];
	odczyt.release();		
}


double Track::euclideanDistance( cv::Point2f p1, cv::Point2f p2 )
{
	cv::Point2f diff = p1 - p2;
	return sqrt(diff.x*diff.x + diff.y*diff.y );
}
