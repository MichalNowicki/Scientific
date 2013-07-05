#include "RGBDprocessing.h"

RGBDclass::RGBDclass()
{
	// Stworzenie zmienne Mat do odbioru Depth
	for(int i=0;i<2;i++)
	{
		frameD[i] = cv::Mat(size_height, size_width, CV_32F);
		frameD_X[i] = cv::Mat(size_height, size_width, CV_32F);
		frameDdiscrete[i] = cv::Mat(size_height, size_width, CV_8U);
		
		// Stworzenie zmiennej Mat do odbioru RGB
		frameBGR[i] = cv::Mat(size_height, size_width, CV_8UC3);
	}
	frameBGR[2] = cv::Mat(size_height, size_width, CV_8UC3);
	frameD[2] = cv::Mat(size_height, size_width, CV_32F);
	
	frameRGB = cv::Mat(size_height, size_width, CV_8UC3);
		
	time = 0;
}

///
/// Adjusting frame to show
///
void RGBDclass::FrameDiscretisation(int which)
{
	RemovingNaNs(which);
	
	float max = -1;
	for(int i=0;i<480;i++)
	{
		for(int j=0;j<640;j++)
		{
				
			if(frameD[which].at<float>(i,j) > max) max = frameD[which].at<float>(i,j);
		}
	}

	// Converting to <0.0 ; 255.0>
	cv::Mat frameTYM = cv::Mat(size_height, size_width, CV_32F);
	frameD[which].convertTo(frameTYM,CV_32F,255.0/(max),0);
	frameTYM.convertTo(frameDdiscrete[which],CV_8U);
}

void RGBDclass::RemovingNaNs(int which)
{
	for(int i=0;i<480;i++)
	{
		for(int j=0;j<640;j++)
		{
				pcl::PointXYZ a;
				a.x = a.y = 0; 
				a.z = frameD[which].at<float>(i,j);
				if(pcl::isFinite(a)==false)
					frameD[which].at<float>(i,j) = 0;
		}
	}
	
}

///
/// Mask
/// 

double RGBDclass::depthMask(int which, double a, double b, int size)
{
	if ( a - size < 0 || a + size >= 480 ||  size - size < 0 || size + size >= 640) return 0; 
	
	double sum = 0;
	int howmany = 0;
	for(int i= -size; i<=size;i++)
	{
		for(int j= -size;j<=size;j++)
		{
			if (frameD[which].at<char>(b+i,a+j) != 0)
			{
				sum += frameD[which].at<char>(b+i,a+j); 
				howmany++;
			}
		}
	}	
	int total = (2*size+1)*(2*size+1);
	// Return 0 if only less than half of neighbourhood has values
	if ( howmany < total / 2 ) return 0;
	
	return sum / howmany;
}

///
/// Extracting
///
inline bool myfunction (cv::KeyPoint i,cv::KeyPoint j) { return (i.response>j.response); }

vector<DeskryptorDouble> RGBDclass::ExtractSURF(int which)
{
	double StartTime = pcl::getTime ();
	
	// Deskryptor SURF
	cv::SurfFeatureDetector detectorBGR(HessianThresholdRGB), detectorD(HessianThresholdD);
	cv:: SurfDescriptorExtractor extractorBGR, extractorD;
	vector<cv::KeyPoint> keypointsBGR, keypointsD;
	
    detectorBGR.detect(frameBGR[which],keypointsBGR);
    detectorD.detect(frameDdiscrete[which],keypointsD);
	
	cout<< "!ExtractSURF! detection time: "<< pcl::getTime() - StartTime<<endl;
	
	StartTime = pcl::getTime();
	//cout<<"OpenCV info: "<<keypointsBGR.size()<< " BGR keypoints found"<<endl;
	//cout<<"OpenCV info: "<<keypointsD.size()<< " D keypoints found"<<endl;
	
	
	cv::Mat descriptorsBGR, descriptorsD;
	extractorBGR.compute(frameBGR[which],keypointsBGR,descriptorsBGR);
	extractorD.compute(frameDdiscrete[which],keypointsD,descriptorsD);
	

	vector<DeskryptorDouble> ret;
	for(int i=0;i<keypointsBGR.size();i++)
    {		
		DeskryptorDouble xxx;
		double a = keypointsBGR[i].pt.x, b = keypointsBGR[i].pt.y;
		double a_round = round(keypointsBGR[i].pt.x), b_round = round(keypointsBGR[i].pt.y);
		
		if( frameDdiscrete[which].at<char>(b_round,a_round) != 0 )
		{
			xxx.punkt.x = -(320-a)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.y = -(240-b)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.z = frameD[which].at<float>(b_round,a_round);
			
			for(int j=0;j<128;j++)
				xxx.deskryptor.push_back(descriptorsBGR.at<double>(i,j)*100000);
			xxx.RGB = 1;
			
			ret.push_back(xxx);
		}
	}
	
	for(int i=0;i<keypointsD.size();i++)
    {		
		DeskryptorDouble xxx;
		double a = keypointsD[i].pt.x, b = keypointsD[i].pt.y;
		double a_round = round(keypointsD[i].pt.x), b_round = round(keypointsD[i].pt.y);
		if( frameDdiscrete[which].at<char>(b_round,a_round) != 0 )
		{
			xxx.punkt.x = -(320-a)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.y = -(240-b)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.z = frameD[which].at<float>(b_round,a_round);
			
			for(int j=0;j<128;j++)
				xxx.deskryptor.push_back(descriptorsD.at<double>(i,j)*100000);
			xxx.RGB = 0;
			
			ret.push_back(xxx);
		}
	}
	time = pcl::getTime() - StartTime;
	
	cout<<"OpenCV info: "<<keypointsBGR.size()<< " BGR keypoints found"<<endl;
	cout<<"OpenCV info: "<<keypointsD.size()<< " D keypoints found"<<endl;
	
	//drawKeypoints(frameBGR[which], keypointsBGR, frameBGR[which]);
	//drawKeypoints(frameDdiscrete[which], keypointsD, frameDdiscrete[which]);
	
	// Wyswietlenie calosc [zmuszone rzutowanie na img]
	//ShowImage(frameBGR[which]);
	//ShowImage(frameDdiscrete[which]);
	
	return ret;
}

vector<DeskryptorDouble> RGBDclass::ExtractSURF_RGB(int which)
{
	double StartTime = pcl::getTime ();
	
	// Deskryptor SURF
	cv::SurfFeatureDetector detectorBGR(HessianThresholdRGB);
	cv:: SurfDescriptorExtractor extractorBGR;
	vector<cv::KeyPoint> keypointsBGR;
	
    detectorBGR.detect(frameBGR[which],keypointsBGR);

	cv::Mat descriptorsBGR;
	extractorBGR.compute(frameBGR[which],keypointsBGR,descriptorsBGR);
	

	vector<DeskryptorDouble> ret;
	for(int i=0;i<keypointsBGR.size();i++)
    {		
		DeskryptorDouble xxx;
		double a = keypointsBGR[i].pt.x, b = keypointsBGR[i].pt.y;
		double a_round = round(keypointsBGR[i].pt.x), b_round = round(keypointsBGR[i].pt.y);
		
		if( frameDdiscrete[which].at<char>(b_round,a_round) != 0 )
		{
			xxx.punkt.x = -(320-a)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.y = -(240-b)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.z = frameD[which].at<float>(b_round,a_round);
			
			for(int j=0;j<128;j++)
				xxx.deskryptor.push_back(descriptorsBGR.at<double>(i,j)*100000);
			xxx.RGB = 1;
			
			ret.push_back(xxx);
		}
	}
	time = pcl::getTime() - StartTime;
	
	//cout<<"OpenCV info: "<<keypointsBGR.size()<< " BGR keypoints found"<<endl;
	
	//drawKeypoints(frameBGR[which], keypointsBGR, frameBGR[which]);
	//drawKeypoints(frameDdiscrete[which], keypointsD, frameDdiscrete[which]);
	
	// Wyswietlenie calosc [zmuszone rzutowanie na img]
	//ShowImage(frameBGR[which]);
	//ShowImage(frameDdiscrete[which]);
	
	return ret;
}

void RGBDclass::ExtractProposed(int which, double eps, int minPts, int ile_RGB, int ile_D)
{
	double StartTime = pcl::getTime ();
	
	// Obraz RGB
	cv::SurfFeatureDetector detectorBGR(HessianThresholdRGB);
	vector<cv::KeyPoint> keypointsBGR;
	
	detectorBGR.detect(frameBGR[which],keypointsBGR);
	cout<<"OpenCV info: "<<keypointsBGR.size()<< " BGR keypoints found"<<endl;
	
	for (int i=0;i<keypointsBGR.size();i++)
	{
		double a = keypointsBGR[i].pt.x, b = keypointsBGR[i].pt.y;
		double a_round = round(keypointsBGR[i].pt.x), b_round = round(keypointsBGR[i].pt.y);
		
		if( frameDdiscrete[which].at<char>(b_round,a_round) == 0 )
		{
			keypointsBGR.erase(keypointsBGR.begin() +i);
			i--;
		}
	}
		
	
	DBScan dbscan;
	vector<int> clusters;
	dbscan.Run(keypointsBGR, eps, minPts, clusters);
	
	vector< pair< cv::KeyPoint, int > > key_temp;
	for(int i=0;i<keypointsBGR.size();i++)
		key_temp.push_back( make_pair(keypointsBGR[i], clusters[i]) );
	
	sort (key_temp.begin(), key_temp.end(), keypointscompare);


	int liczba_grup = key_temp[0].second;
	int ile_wybrac_z_grupy = 2;
	int ile_bezgrupy = ile_RGB - 2*liczba_grup;
	
	cout<< "LG: " << liczba_grup<<endl;
	
	keypointsBGR.clear();

	for (int i=0,w_grupie = 0,last = -1;i<key_temp.size();i++)
	{
		if ( last != key_temp[i].second ) w_grupie = 0;
		if ( key_temp[i].second != 0 )
		{	
			while( w_grupie < ile_wybrac_z_grupy)
			{
				keypointsBGR.push_back(key_temp[i].first);
				last = key_temp[i].second;
				w_grupie++;
				i++;
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
				keypointsBGR.push_back(key_temp[i].first);
				last = key_temp[i].second;
				w_grupie++;
				i++;
				if(key_temp[i].second != key_temp[i-1].second)
				{
					i--;
					break;
				}
			}
		}
	}
	cout<<"OpenCV info after clustering: "<<keypointsBGR.size()<< " BGR keypoints found"<<endl;

	for(int i=0;i<keypointsBGR.size();i++)
    {					
		DeskryptorDouble xxx;
		double a = keypointsBGR[i].pt.x, b = keypointsBGR[i].pt.y;
		double a_round = round(keypointsBGR[i].pt.x), b_round = round(keypointsBGR[i].pt.y);
		
		if( frameDdiscrete[which].at<char>(b_round,a_round) != 0 )
		{
			xxx.punkt.x = -(320-a)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.y = -(240-b)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.z = frameD[which].at<float>(b_round,a_round);
			Keypoints3d[which].push_back(xxx.punkt);
		}
		else
		{
			keypointsBGR.erase(keypointsBGR.begin() + i);
			i--;
		}		
	}
	
	cout<<"OpenCV info poprawnych 3D: "<<keypointsBGR.size()<< " BGR keypoints found"<<endl;
	cv:: SurfDescriptorExtractor extractorBGR;
	extractorBGR.compute(frameBGR[which],keypointsBGR,descriptors[which]);
	
	
	// FAST
	cv::Ptr<cv::FeatureDetector> detector(new cv::DynamicAdaptedFeatureDetector ( new cv::FastAdjuster(30,true),ile_D / 8 - 5, ile_D / 8 + 5, 3));
                              
	vector<cv::KeyPoint> keypointsD;
	
	for(int i=0;i<6;i++)
	{
		vector<cv::KeyPoint> keypoints;
		cv::Mat roi( frameDdiscrete[which] , cv::Rect(0,i*80,640,80));
		detector->detect(roi, keypoints);
		
		sort(keypoints.begin(), keypoints.end(), keyRESPONSEcompare);
		
		for (int j=0;j< 10 && j < keypoints.size();j++)
		{
			keypoints[j].pt.y += i*80;
			keypointsD.push_back(keypoints[j]);
		}
		
	}
    
    cout<<"OpenCV D keypoints: "<<keypointsD.size()<< " keypoints found"<<endl;
    
    cv::Mat descriptorsD;
	
	for(int i=0;i<keypointsD.size();i++)
    {		
		DeskryptorDouble xxx;
		double a = keypointsD[i].pt.x, b = keypointsD[i].pt.y;
		double a_round = round(keypointsD[i].pt.x), b_round = round(keypointsD[i].pt.y);
		if( frameDdiscrete[which].at<char>(b_round,a_round) != 0 )
		{
			xxx.punkt.x = -(320-a)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.y = -(240-b)*frameD[which].at<float>(b_round,a_round)*INV_FOCAL;
			xxx.punkt.z = frameD[which].at<float>(b_round,a_round);
			Keypoints3d[which].push_back(xxx.punkt);
		}
		else
		{
			keypointsD.erase(keypointsD.begin() + i);
			i--;
		}	
	}
	cv:: SurfDescriptorExtractor extractorD;
	extractorD.compute(frameDdiscrete[which],keypointsD,descriptors[which+2]);
	
	double EndTime = pcl::getTime();
	
	cout<<"Extract Proposed taken: " << EndTime - StartTime<<endl;
	
	drawKeypoints(frameBGR[which], keypointsBGR, frameBGR[which]);
	drawKeypoints(frameDdiscrete[which], keypointsD, frameDdiscrete[which]);
	
	// Wyswietlenie calosc [zmuszone rzutowanie na img]
	ShowImage(frameBGR[which]);
	ShowImage(frameDdiscrete[which]);
	
}

///
/// Converting RGBD->pointcloud && pointclouds->RGBD
///

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RGBDclass::BuildPointCloudFromRGBD(int which)
{
	//Zbudowanie point cloudu
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BuildCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	// Podstawowe info o chmurze
	BuildCloud->width = size_width *size_height;
	BuildCloud->height = 1;
	BuildCloud->is_dense = false;

	for(int i=0;i<480;i++)
	{
		for(int j=0;j<640;j++)
		{
			if ( frameD[which].at<float>(i,j) != 0 && frameD[which].at<float>(i,j) < 4 )
			{
				pcl::PointXYZRGBA point;
				point.x = -(320-j)*frameD[which].at<float>(i,j)*INV_FOCAL;
				point.y = -(240-i)*frameD[which].at<float>(i,j)*INV_FOCAL;
				point.z = frameD[which].at<float>(i,j);
						
				cv::Vec3b Color = frameBGR[which].at<cv::Vec3b>(i,j);
				point.r = Color.val[2];
				point.g = Color.val[1];
				point.b = Color.val[0];

				BuildCloud->points.push_back(point);
			}
		}
	}

	// Tymczasowy podglad
	//pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Copy from XYZRGB cloud to XYZ
	//copyPointCloud(*BuildCloud, *inCloud);
	
	return BuildCloud;
}

void RGBDclass::BuildRGBDfromPointCloud(int which, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	int ile = cloud->points.size();

	cv::Vec3b Zero(0,0,0);
	for(int i=0;i<480;i++)
	{
		for(int j=0;j<640;j++)
		{
			frameBGR[which].at<cv::Vec3b>(i,j) = Zero;
			frameD[which].at<float>(i,j) = 0;
		}
	}
	for(int w=0;w<ile;w++)
	{
		pcl::PointXYZRGBA pun = cloud->points[w];
		
		if(pcl::isFinite(pun)==true && pun.z != 0)
		{
			
			float i = round(240.0 + pun.y/(pun.z*INV_FOCAL) );
			float j = round(320.0 + pun.x/(pun.z*INV_FOCAL) );
	
			frameD[which].at<float>(i,j) = pun.z;
			
			cv::Vec3b Color;
			Color.val[2] = pun.r;
			Color.val[1] = pun.g;
			Color.val[0] = pun.b;
			frameBGR[which].at<cv::Vec3b>(i,j) = Color;
		}
	}

	FrameDiscretisation(which);
}

///
/// Loading data
///

void RGBDclass::GetDataFromKinect(int which, int h)
{
	
	frameD[which] = cv::Mat(size_height, size_width, CV_32F);
	frameBGR[which] = cv::Mat(size_height, size_width, CV_8UC3);
		
	// tworzymy strukturę dla przechwycenia danych z urządzenia
	StrukturaKinectaRGBD Kinect_Data;
	// tworzymy interfejs  który posluży do komunikacji z Kinectem
	pcl::OpenNIGrabber grabber;
	// obiekt do którego podłączamy streaming z kinecta	
	boost::signals2::connection KinectStream;
	
	// Dziwny callback
    KinectStream = grabber.registerCallback(boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float)>(boost::bind(&StrukturaKinectaRGBD::callback,&Kinect_Data, _1, _2, _3)));
 
    // podpinamy strukturę Kinect_Data jako docelową dla streamingu danych z kinecta
   	Kinect_Data.Streaming=1;

    // Uruchomienie polaczenia z Kinectem
    grabber.start();
    
	while(Kinect_Data.Streaming==1){}
	
	
	// Pobranie danych Depth
	Kinect_Data.imgD->fillDepthImage(frameD[which].cols, frameD[which].rows, (float*)frameD[which].data);
	// Pobranie danych o focal
	//inv_focal = Kinect_Data.focal;
	// Pobranie danych RGB
	Kinect_Data.imgRGB->fillRGB(frameRGB.cols, frameRGB.rows, frameRGB.data, frameRGB.step);
	// Conversja z RGB na BGR
	cv::cvtColor(frameRGB,frameBGR[which],CV_RGB2BGR);
	
	cout<<" Focal length " << Kinect_Data.focal<<endl;
	
	// Saving
	SaveRGBD(h);
	//SaveRGBDnew(h,h);
	
	cout<<"Done"<<endl;
	// Kinect disconnect
	KinectStream.disconnect();
}

void RGBDclass::GetContinuousRGBD()
{
	for(int h=0;true;h++)
	{
		cout<<"Wciśnij enter, aby zapisać " <<h+1<<" zdjecia"<<endl;
		getchar();	  

		GetDataFromKinect(0,h);
	}
}

void RGBDclass::LoadRGB(string name, int place)
{	
	frameBGR[place] = cv::imread(name);
}

void RGBDclass::LoadRGBD(int which)
{
	string name,name2;
	cout<<"Podaj nazwe pliku z Depth Image np. frameD.xml"<<endl;
	cin >> name;
	
	cout<<"Podaj nazwe pliku z BGR Image np. frameBGR.png"<<endl;
	cin >> name2;
	
	LoadRGBD2(name,name2, which);
}

void RGBDclass::LoadRGBD2(string name,string name2,int which)
{
	cv::FileStorage odczyt;
	odczyt.open(name, cv::FileStorage::READ);
	odczyt["frameD"] >>frameD[which];
	odczyt.release();
	
	frameBGR[which] = cv::imread(name2);

	// Znajd min i max w frameD 
	FrameDiscretisation(which);
}

///
/// Saving data
///

void RGBDclass::SaveRGBD(int which)
{
	SaveRGBD(which,0);
}

void RGBDclass::SaveRGBD(int indeks,int which)
{
	char nazwapliku[30];
	sprintf(nazwapliku,"frameD_%d.xml",indeks);
		
	cv::FileStorage zapis;
	zapis.open(nazwapliku, cv::FileStorage::WRITE);
	zapis << "frameD" << frameD[which];
	zapis.release();
	
	sprintf(nazwapliku,"frameBGR_%d.png",indeks);
	cv::imwrite(nazwapliku, frameBGR[which]);
}

void RGBDclass::SaveFrameDasPNG(int indeks,int which)
{
	cv::Mat frameDnew;
	frameDnew = cv::Mat(size_height, size_width, CV_16U);
	
	for(int i=0;i<480;i++)
	{
		for(int j=0;j<640;j++)
		{
				pcl::PointXYZ a;
				a.x = a.y = 0; 
				a.z = frameD[which].at<float>(i,j);
				if(pcl::isFinite(a)==false || a.z == 0)
					frameDnew.at<unsigned short int>(i,j) = 65535; 
				else
				{
					frameDnew.at<unsigned short int>(i,j) = (unsigned short int) (frameD[which].at<float>(i,j) * 5000); // Przepisanie do frame'a bez infow
				}
		}
	}
	
	char nazwapliku[30];
	sprintf(nazwapliku,"frameD_%d.png",indeks);
	cv::imwrite(nazwapliku, frameDnew);
}

