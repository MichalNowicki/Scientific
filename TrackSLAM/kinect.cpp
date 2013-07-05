#include "kinect.h"

Kinect::Kinect()
{
	for (int i=0;i<CLOUD_SIZE;i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr xxx (new pcl::PointCloud<pcl::PointXYZ>);
		this->chmura[i] = xxx;
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr yyy (new pcl::PointCloud<pcl::PointXYZRGBA>);
		this->chmura_color[i] = yyy;
	}
	this->RGBD = new RGBDclass();
    this->ReadParameters();
    this->RGBD->HessianThresholdRGB = Parameters.HessianThresholdRGB;
    this->RGBD->HessianThresholdD = Parameters.HessianThresholdD;

 	zapis.open("Kinect.log");	
}

Kinect::~Kinect()
{
	zapis.close();
}

void Kinect::ReadParameters()
{
	string linia;
	ifstream o;
	o.open("parameters.cfg");
	
	// Tymczasowe zmienne
	float KinectObrX, KinectObrY, KinectObrZ, KinectTx, KinectTy, KinectTz;
	
	while(!o.eof())
	{
		getline(o,linia);
		
		if(linia == "# Filtr antyNANowy")
		{
			o>>linia;
			if(linia == "1") Parameters.antyNaN = true;
			else 		 Parameters.antyNaN = false;
		}
		else if(linia == "# Filtr VoxelGrid (downsampling)")
		{
			o>>linia;
			if(linia == "1") Parameters.VoxelGrid = true;
			else 		 Parameters.VoxelGrid = false;
		}
		else if(linia == "# Filtr VoxelGrid (Leaf size)")
		{
			o>>linia;
			Parameters.LeafSize = atof(linia.c_str());
		}
		else if(linia == "# Filtr Antyszumowy 3D")
		{
			o>>linia;
			if(linia == "1") Parameters.antySzum = true;
			else 		 Parameters.antySzum = false;
		}
		else if(linia == "# RANSAC - ile iteracji")
		{
			o>>linia;			
			Parameters.iteration_number = atoi(linia.c_str());
		}
		else if(linia == "# Ile par deskyptorow uzyc do stworzenia modelu")
		{
			o>>linia;			
			Parameters.NARF_pair_number = atoi(linia.c_str());
		}
		else if(linia == "# RANSAC 1-on, 0-off")
		{
			o>>linia;
			if(linia == "1") Parameters.RANSAC = true;
			else 		 	 Parameters.RANSAC = false;
		}
		else if(linia == "# Threshold - kiedy uznany za inliera")
		{
			o>>linia;			
			Parameters.Threshold = atof(linia.c_str());
		}
		else if(linia == "# Ile procent pokrycia punktow")
		{
			o>>linia;			
			Parameters.NARF_minimal_matched = atof(linia.c_str());
		}
		else if(linia == "# MODE: 0- chmury z kinecta 1- wczytywanie z pliku")
		{
			o>>linia;
			if(linia == "1") Parameters.MODE = true;
			else 		 Parameters.MODE = false;
		}
		else if(linia == "# Rozdzielczosc katowa (NARF)")
		{
			o>>linia;			
			Parameters.ang_res = atof(linia.c_str());
		}
		else if(linia == "# Support size (NARF)")
		{
			o>>linia;			
			Parameters.support_size = atof(linia.c_str());
		}
		else if(linia == "# Hessian Threshold RGB")
		{
			o>>linia;			
			Parameters.HessianThresholdRGB = atoi(linia.c_str());
		}
		else if(linia == "# Hessian Threshold D")
		{
			o>>linia;			
			Parameters.HessianThresholdD = atoi(linia.c_str());
		}
		else if(linia == "# SURF size when only descriptor")
		{
			o>>linia;			
			Parameters.SURFsize= atoi(linia.c_str());
		}
		else if(linia == "# SURF2D expected number of keypoints RGB")
		{
			o>>linia;			
			Parameters.SURF2DExpectedRGB = atoi(linia.c_str());
		}
		else if(linia == "# SURF2D expected number of keypoints D")
		{
			o>>linia;			
			Parameters.SURF2DExpectedD = atoi(linia.c_str());
		}
		else if(linia == "# SURF2D DBScan MinPts")
		{
			o>>linia;			
			Parameters.DBScanMinPts = atoi(linia.c_str());
		}
		else if(linia == "# SURF2D DBScan eps")
		{
			o>>linia;			
			Parameters.DBScanEps = atoi(linia.c_str());
		}
		else if(linia == "# Maksymalna liczba iteracji RANSACa bez poprawy")
		{
			o>>linia;			
			Parameters.MaxLiczbaIteracjiBezPoprawy = atoi(linia.c_str());
		}
		else if(linia == "# Zadany error do RANSACa")
		{
			o>>linia;			
			Parameters.RANSACerror = atof(linia.c_str());
		}
		else if(linia == "# Zadane prawdopodobienstwo sukcesu RANSACa")
		{
			o>>linia;			
			Parameters.RANSACsuccessrate = atof(linia.c_str());
		}
		else if(linia == "# minObrX")
		{
			o>>linia;			
			Parameters.constrain.minObrX = atof(linia.c_str());
		}
		else if(linia == "# maxObrX")
		{
			o>>linia;			
			Parameters.constrain.maxObrX = atof(linia.c_str());
		}
		else if(linia == "# minObrY")
		{
			o>>linia;			
			Parameters.constrain.minObrY = atof(linia.c_str());
		}
		else if(linia == "# maxObrY")
		{
			o>>linia;			
			Parameters.constrain.maxObrY = atof(linia.c_str());
		}
		else if(linia == "# minObrZ")
		{
			o>>linia;			
			Parameters.constrain.minObrZ = atof(linia.c_str());
		}
		else if(linia == "# maxObrZ")
		{
			o>>linia;			
			Parameters.constrain.maxObrZ = atof(linia.c_str());
		}
		else if(linia == "# minTx")
		{
			o>>linia;			
			Parameters.constrain.minTx = atof(linia.c_str());
		}
		else if(linia == "# maxTx")
		{
			o>>linia;			
			Parameters.constrain.maxTx = atof(linia.c_str());
		}
		else if(linia == "# minTy")
		{
			o>>linia;			
			Parameters.constrain.minTy = atof(linia.c_str());
		}
		else if(linia == "# maxTy")
		{
			o>>linia;			
			Parameters.constrain.maxTy = atof(linia.c_str());
		}
		else if(linia == "# minTz")
		{
			o>>linia;			
			Parameters.constrain.minTz = atof(linia.c_str());
		}
		else if(linia == "# maxTz")
		{
			o>>linia;			
			Parameters.constrain.maxTz = atof(linia.c_str());
		}
		else if(linia == "# ICP max iterations")
		{
			o>>linia;			
			Parameters.ICP_max_iterations = atoi(linia.c_str());
		}
		else if(linia == "# Transformation epsilon")
		{
			o>>linia;			
			Parameters.ICPepsilon = atof(linia.c_str());
		}
		else if(linia == "# Outlier RejectionThreshold")
		{
			o>>linia;			
			Parameters.OutRejThre = atof(linia.c_str());
		}
		else if(linia == "# maxCorrespondence distance")
		{
			o>>linia;			
			Parameters.maxCorrespondeceDist = atof(linia.c_str());
		}
		else if(linia == "# minTz")
		{
			o>>linia;			
			Parameters.constrain.minTz = atof(linia.c_str());
		}
		else if(linia == "# maxTz")
		{
			o>>linia;			
			Parameters.constrain.maxTz = atof(linia.c_str());
		}
		else if(linia == "# ICP max iterations")
		{
			o>>linia;			
			Parameters.ICP_max_iterations = atoi(linia.c_str());
		}
		else if(linia == "# Transformation epsilon")
		{
			o>>linia;			
			Parameters.ICPepsilon = atof(linia.c_str());
		}
		else if(linia == "# Outlier RejectionThreshold")
		{
			o>>linia;			
			Parameters.OutRejThre = atof(linia.c_str());
		}
		else if(linia == "# maxCorrespondence distance")
		{
			o>>linia;			
			Parameters.maxCorrespondeceDist = atof(linia.c_str());
		}
		else if(linia == "# Kinect - obrot X")
		{
			o>>linia;			
			KinectObrX = atof(linia.c_str());
		}
		else if(linia == "# Kinect - obrot Y")
		{
			o>>linia;			
			KinectObrY = atof(linia.c_str());
		}
		else if(linia == "# Kinect - obrot Z")
		{
			o>>linia;			
			KinectObrZ = atoi(linia.c_str());
		}
		else if(linia == "# Kinect - translacja X")
		{
			o>>linia;			
			KinectTx = atof(linia.c_str());
		}
		else if(linia == "# Kinect - translacja Y")
		{
			o>>linia;			
			KinectTy = atof(linia.c_str());
		}
		else if(linia == "# Kinect - translacja Z")
		{
			o>>linia;			
			KinectTz = atof(linia.c_str());
		}				
	}
	calculateTransformation(KinectInitial,pcl::deg2rad(KinectObrX), pcl::deg2rad(KinectObrY), pcl::deg2rad(KinectObrZ), KinectTx, KinectTy, KinectTz);
}

///
/// Filtering 
///

void Kinect::Filtrowanie(int x)
{
	// Filtry antyNAN
	if(Parameters.antyNaN == true)
		chmura[x] = FiltrAntyNaN(chmura[x]);


	// Filtr downsampling
	if(Parameters.VoxelGrid == true)
		chmura[x] = FiltrVoxelGrid2(chmura[x],Parameters.LeafSize);
		
	// Filtr antyszumowy 3D
	if(Parameters.antySzum == true)
		chmura[x] = FiltrAntySzum2(chmura[x],50,0.1);
}

void Kinect::FiltrowanieColor(int x)
{
	// Filtr downsampling
	if(Parameters.VoxelGrid == true)
		chmura_color[x] = FiltrVoxelGrid2(chmura_color[x],Parameters.LeafSize);
}

///
/// Computing features
///

void Kinect::ExtractNARF()
{	
	// Wartości potrzebne do stworzenia range_image, który jest potrzebny do wyliczenia NARF
	Parameters.ang_res = pcl::deg2rad (Parameters.ang_res);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

	bool rotation_invariant = true;
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;

	// Start czasu
	StartTime = pcl::getTime ();
	
	// Chmury, ktore beda podawane do funkcji wyliczajacych narf
	pcl::PointCloud<pcl::PointXYZ>& point_cloud = *chmura[0];
	pcl::PointCloud<pcl::PointXYZ>& point_cloud2 = *chmura[1];

	// Pozycja sensora w ukladzie wspolrzednych
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	 
  	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	  
	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
	boost::shared_ptr<pcl::RangeImage> range_image_ptr2 (new pcl::RangeImage);

	pcl::RangeImage& range_image = *range_image_ptr;   
	pcl::RangeImage& range_image2 = *range_image_ptr2;  
 
	range_image.createFromPointCloud (point_cloud, Parameters.ang_res, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
		                           scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	range_image2.createFromPointCloud (point_cloud2, Parameters.ang_res, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
		                           scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	range_image.setUnseenToMaxRange (); 
	range_image2.setUnseenToMaxRange (); 


	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::RangeImageBorderExtractor range_image_border_extractor2;
	pcl::NarfKeypoint narf_keypoint_detector;
	pcl::NarfKeypoint narf_keypoint_detector2;

	narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
	narf_keypoint_detector2.setRangeImageBorderExtractor (&range_image_border_extractor2);

	narf_keypoint_detector.setRangeImage (&range_image);
	narf_keypoint_detector2.setRangeImage (&range_image2);

	narf_keypoint_detector.getParameters ().support_size = Parameters.support_size;
	narf_keypoint_detector2.getParameters ().support_size = Parameters.support_size;
	  
	// Wyliczenie NARF keypoints
	pcl::PointCloud<int> keypoint_indices;
 	pcl::PointCloud<int> keypoint_indices2;

	narf_keypoint_detector.compute (keypoint_indices);
	narf_keypoint_detector2.compute (keypoint_indices2);

	cout << "1 chmura : Found "<<keypoint_indices.points.size ()<<" key points.\n";
	cout << "2 chmura : Found "<<keypoint_indices2.points.size ()<<" key points.\n";

	zapis << "1 chmura : Found "<<keypoint_indices.points.size ()<<" key points.\n";
	zapis << "2 chmura : Found "<<keypoint_indices2.points.size ()<<" key points.\n";
	  
	// NARF descriptors
    std::vector<int> keypoint_indices_1,keypoint_indices_2;
	keypoint_indices_1.resize (keypoint_indices.points.size ());
	keypoint_indices_2.resize (keypoint_indices2.points.size ());
	
	for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
		keypoint_indices_1[i]=keypoint_indices.points[i];
		
	for (unsigned int i=0; i<keypoint_indices2.size (); ++i) // This step is necessary to get the right vector type
		keypoint_indices_2[i]=keypoint_indices2.points[i];
		
	// Wylicz deskryptory dla keypoints	
	pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices_1), narf_descriptor2 (&range_image2, &keypoint_indices_2);
	narf_descriptor.getParameters ().support_size = narf_descriptor2.getParameters ().support_size = Parameters.support_size;
	narf_descriptor.getParameters ().rotation_invariant = narf_descriptor2.getParameters ().rotation_invariant  = true;


	pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors(new pcl::PointCloud<pcl::Narf36>), narf_descriptors2(new pcl::PointCloud<pcl::Narf36>);
	
	narf_descriptor.compute (*narf_descriptors);
	narf_descriptor2.compute (*narf_descriptors2);
	
	EndTime = pcl::getTime ();
	cout<<"<time> NARF took : "<<EndTime-StartTime<< " seconds " << endl;
	zapis<<"<time> NARF took : "<<EndTime-StartTime<< " seconds " << endl;
	
	// Podsumowanie
	cout << "Cloud 1:\tExtracted "<<narf_descriptors->size ()<<" descriptors for "<<keypoint_indices.points.size ()<< " keypoints.\n";
	cout << "Cloud 2:\tExtracted "<<narf_descriptors2->size ()<<" descriptors for "<<keypoint_indices2.points.size ()<< " keypoints.\n";
	
	zapis << "Cloud 1:\tExtracted "<<narf_descriptors->size ()<<" descriptors for "<<keypoint_indices.points.size ()<< " keypoints.\n";
	zapis << "Cloud 2:\tExtracted "<<narf_descriptors2->size ()<<" descriptors for "<<keypoint_indices2.points.size ()<< " keypoints.\n";
	
	descriptors[0].clear();
	descriptors[1].clear();
	// Przepisanie deskryptorow do wlasnego formatu danych
    for(int i=0;i<narf_descriptors->points.size();i++)
    {
		DeskryptorDouble xxx;
		xxx.punkt.x = narf_descriptors->points[i].x;
		xxx.punkt.y = narf_descriptors->points[i].y;
		xxx.punkt.z = narf_descriptors->points[i].z;
		
		for(int j=0;j<36;j++)
			xxx.deskryptor.push_back(narf_descriptors->points[i].descriptor[j]);
		descriptors[0].push_back(xxx);
	}
	for(int i=0;i<narf_descriptors2->points.size();i++)
    {
		DeskryptorDouble xxx;
		xxx.punkt.x = narf_descriptors2->points[i].x;
		xxx.punkt.y = narf_descriptors2->points[i].y;
		xxx.punkt.z = narf_descriptors2->points[i].z;
		
		for(int j=0;j<36;j++)
			xxx.deskryptor.push_back(narf_descriptors2->points[i].descriptor[j]);
		descriptors[1].push_back(xxx);
	}
}

void Kinect::WyliczNARF()
{
	ExtractNARF();
}

void Kinect::WyliczNARFSURF()
{
	// NARF
	ExtractNARF();
	
	vector<cv::KeyPoint> keypointsBGR[2];
	cv::Mat descriptorsBGR[2];
	
	// Przeliczenie punktow 3D na 2D
	for(int k=0;k<2;k++)
	{
		for (int i=0;i<descriptors[k].size();i++)
		{
			cv::KeyPoint miejsce;
			miejsce.pt.x =	round(320.0 + descriptors[k][i].punkt.x/(descriptors[k][i].punkt.z*INV_FOCAL) ) ;
			miejsce.pt.y =	round(240.0 + descriptors[k][i].punkt.y/(descriptors[k][i].punkt.z*INV_FOCAL) );
			miejsce.size = Parameters.SURFsize;
			keypointsBGR[k].push_back(miejsce);
		}
		
		
		// Deskrypcja na obrazie kolorowym
		cv:: SurfDescriptorExtractor extractorBGR;
		
		double StartTime = pcl::getTime();
		extractorBGR.compute(RGBD->frameBGR[k],keypointsBGR[k],descriptorsBGR[k]);
		cout << "<time> SURF on NARF took : "<<pcl::getTime()-StartTime<< " seconds " << endl;
		zapis << "<time> SURF on NARF took : "<<pcl::getTime()-StartTime<< " seconds " << endl;

		// Dodanie do deskryptora
		for(int i=0;i<keypointsBGR[k].size();i++)
		{		
			//cout<<descriptorsBGR[k].at<double>(i,0)*10000<<endl;
			for(int j=0;j<128;j++)
			{
				descriptors[k][i].deskryptor.push_back(	descriptorsBGR[k].at<double>(i,j)*100000 );
			}
		}
	}
}

void Kinect::WyliczSURF()
{
	descriptors[0] = RGBD->ExtractSURF_RGB(0);
	zapis << "<time> SURF on RGB image took : "<<RGBD->time<< " seconds " << endl;
	cout << "<time> SURF on RGB image took : "<<RGBD->time<< " seconds " << endl;
	
	descriptors[1] = RGBD->ExtractSURF_RGB(1);
	zapis << "<time> SURF on RGB image took : "<<RGBD->time<< " seconds " << endl;
	cout << "<time> SURF on RGB image took : "<<RGBD->time<< " seconds " << endl;
	
}

void Kinect::WyliczSURFSURF()
{
	descriptors[0] = RGBD->ExtractSURF(0);
	for(int i=0;i<descriptors[0].size();i++)
	{
		if(descriptors[0][i].RGB == 0)
		{
			descriptors[2].push_back(descriptors[0][i]);
			descriptors[0].erase(descriptors[0].begin() + i);
			i--;
		}
	}
	cout << "<time> SURF on RGB & D images took : "<<RGBD->time<< " seconds " << endl;
	zapis << "<time> SURF on RGB & D images took : "<<RGBD->time<< " seconds " << endl;
	
	descriptors[1] = RGBD->ExtractSURF(1);
	for(int i=0;i<descriptors[1].size();i++)
	{
		if(descriptors[1][i].RGB == 0)
		{
			descriptors[3].push_back(descriptors[1][i]);
			descriptors[1].erase(descriptors[1].begin() + i);
			i--;
		}
	}
	cout << "<time> SURF on RGB & D images took : "<<RGBD->time<< " seconds " << endl;
	zapis << "<time> SURF on RGB & D images took : "<<RGBD->time<< " seconds " << endl;
	
}

void Kinect::WyliczSURF2D()
{
	RGBD->ExtractProposed(0, Parameters.DBScanEps, Parameters.DBScanMinPts, Parameters.SURF2DExpectedRGB, Parameters.SURF2DExpectedD);
	RGBD->ExtractProposed(1, Parameters.DBScanEps, Parameters.DBScanMinPts, Parameters.SURF2DExpectedRGB, Parameters.SURF2DExpectedD);
}

///
/// Feature Matching
///

void Kinect::FeatureMatching()
{
	cout<<"Start feature matching"<<endl;
	zapis<<"Start feature matching"<<endl;


	// Wyzerowanie listy modeli
	ListaPoprawnychModeli.clear();
	
	
	double StartTime;
	double EndTime = pcl::getTime ();
	StartTime = EndTime;

	// Pary wybrane po kryterium euklidesowym i minimum podobienstwa
	// RGB + D
	vector<ParaDeskryptorow> ParyWybrane[2];
	
	// Start time
	StartTime = pcl::getTime();
	
	// Policzenie roznic RGB
	struct ParaDeskryptorow PotenPara;
	float errorek,min;
	for(int z = 0 ; z<3;z+=2) // Ile zbiorow par - para to deskryptor(x,x+1) oraz (x+2,x+3)
	{
		for(int j=0;j<descriptors[z].size();j++) // Dla kazdego deskryptora z 1 zestawu
		{	
			min = -1;
			for(int k=0;k<descriptors[z+1].size();k++) // Dla kazdego deskryptora z 2 zestawu
			{
				errorek = 0;
				for(int i=0;i<descriptors[z][0].deskryptor.size();i++) // Dla dlugosci deskryptora
				{
					// Roznica jakosciowa
					errorek += pow(abs(descriptors[z][j].deskryptor[i] - descriptors[z+1][k].deskryptor[i]),2);
				}
				if(errorek< min || min == -1) min = errorek;
			}
			
			// Odrzucanie większych różnic niż 3 minima
			PotenPara.indeks1 = j;
			for(int k=0;k<descriptors[z+1].size();k++)
			{
				errorek = 0;
				for(int i=0;i<descriptors[z][0].deskryptor.size();i++)
				{
					errorek += pow(abs(descriptors[z][j].deskryptor[i] - descriptors[z+1][k].deskryptor[i]),2);
				}
				double odl = SquaredNorm(descriptors[0][j].punkt, descriptors[1][k].punkt);
				
				if(errorek < 3*min && odl < Parameters.constrain.maxTx*Parameters.constrain.maxTx + Parameters.constrain.maxTy*Parameters.constrain.maxTy + Parameters.constrain.maxTz*Parameters.constrain.maxTz) 
				{
					PotenPara.indeks2 = k;
					PotenPara.value = errorek;
					ParyWybrane[z/2].push_back(PotenPara);
				}
			}
		}
	}
	
	EndTime = pcl::getTime ();
	cout<<"<time> Odrzucenie par deskryptorow zajelo : "<<EndTime-StartTime<< " seconds " << endl;
	zapis<<"<time> Odrzucenie par deskryptorow zajelo : "<<EndTime-StartTime<< " seconds " << endl;
	
	for(int z=0;z<3;z+=2)
	{
		cout<<z/2<<" zestaw par = "<<descriptors[z].size()<< " x " << descriptors[z+1].size()<<endl;
		cout<<"Wszystkich par z "<<z/2<<" zestawu bylo : "<<descriptors[z].size()*descriptors[z+1].size()<<endl;
		cout<<"Liczba par po odrzuceniu dalszych niz 3 minima oraz odleglosc euklidesowych: "<<ParyWybrane[z/2].size()<<endl;
	
		zapis<<z/2<<" zestaw par = "<<descriptors[z].size()<< " x " << descriptors[z+1].size()<<endl;
		zapis<<"Wszystkich par z "<<z/2<<" zestawu bylo : "<<descriptors[z].size()*descriptors[z+1].size()<<endl;
		zapis<<"Liczba par po odrzuceniu dalszych niz 3 minima oraz odleglosc euklidesowych: "<<ParyWybrane[z/2].size()<<endl;
	}
	

	StartTime = pcl::getTime();
	
	int WybranyIndeks = -1, IleWybranychPar = 0;
	vector<int> IndeksyNajlepszego, Naj[2];
	vector<bool> UzytePary[2];
	
	for (int z =0 ;z<2;z++) for(int i=0;i<ParyWybrane[z].size();i++) UzytePary[z].push_back(false);

	// Petla znajdujaca najlepsze dopasowania tworzące model
	// RGB i potem D
	
	for (int zb = 0 ; zb < 2; zb++)
	{
		while(IleWybranychPar != Parameters.NARF_pair_number && ParyWybrane[zb].size() > 15)
		{
			min = 99999;
			for(int i=0;i<ParyWybrane[zb].size();i++)
			{
				if(ParyWybrane[zb][i].value < min )
				{
					min = ParyWybrane[zb][i].value;
					WybranyIndeks = i;
				}
			}
			
			if(UzytePary[zb][ParyWybrane[zb][WybranyIndeks].indeks1] == false && UzytePary[zb][ParyWybrane[zb][WybranyIndeks].indeks2] == false)
			{
				UzytePary[zb][ParyWybrane[zb][WybranyIndeks].indeks1] = true;
				UzytePary[zb][ParyWybrane[zb][WybranyIndeks].indeks2] = true;
				Naj[zb].push_back(WybranyIndeks);
				IleWybranychPar++;
			}
			ParyWybrane[zb][WybranyIndeks].value = 9999999;
		}
		IleWybranychPar = 0;
	}
	
	cout<<" Po wybraniu par"<<endl;
	// Najlepsze 4 modele z 3 najlepszych RGB i 3 najlepszych D
	int przesuniecie = ParyWybrane[0].size();
	
	// Pierwszy indeks to ktory zbior dopasowan (0 - rgb, 1 - d)
	// RRR
	IndeksyNajlepszego.push_back(Naj[0][0]);
	IndeksyNajlepszego.push_back(Naj[0][1]);
	IndeksyNajlepszego.push_back(Naj[0][2]);
	
	if (ParyWybrane[1].size() >= 3)
	{
		// RRD
		IndeksyNajlepszego.push_back(Naj[0][0]);
		IndeksyNajlepszego.push_back(Naj[0][1]);
		IndeksyNajlepszego.push_back(Naj[1][0]+przesuniecie);
		
		// RDD
		IndeksyNajlepszego.push_back(Naj[0][0]);
		IndeksyNajlepszego.push_back(Naj[1][0]+przesuniecie);
		IndeksyNajlepszego.push_back(Naj[1][1]+przesuniecie);
		
		// LiniaPocz
		IndeksyNajlepszego.push_back(Naj[1][0]+przesuniecie);
		IndeksyNajlepszego.push_back(Naj[1][1]+przesuniecie);
		IndeksyNajlepszego.push_back(Naj[1][2]+przesuniecie);
	}
	
	//cout<< "--0"<<endl;
	int size1 = descriptors[0].size();
	int size2 = descriptors[1].size();
	int size3 = descriptors[2].size();
	int size4 = descriptors[3].size();
	int size12 = size1*size2;
	int size34 = size3*size4;
	int LiczbaIteracjiBezPoprawy = 0;
	vector <bool> valid_indexes,valid_indexes2;
	vector <int> model;
	
	// chmury do wyliczenia errorow
	pcl::PointCloud<pcl::PointXYZ>::Ptr chmura_RGB1(new pcl::PointCloud<pcl::PointXYZ>),chmura_RGB2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr chmura_D1(new pcl::PointCloud<pcl::PointXYZ>),chmura_D2(new pcl::PointCloud<pcl::PointXYZ>);
	
	//cout<< "X0"<<endl;		
	chmura_RGB1->width    = size1;
	chmura_RGB2->width    = size2;
	chmura_D1->width	  = size3;
	chmura_D2->width	  = size4;
	chmura_RGB1->height   = chmura_RGB2->height		= chmura_D1->height	  = chmura_D2->height = 1;
	chmura_RGB1->is_dense = chmura_RGB2->is_dense 	= chmura_D1->is_dense    = chmura_D2->is_dense  = false;
			  
	chmura_RGB1->points.resize (size1);
	chmura_RGB2->points.resize (size2);
	if ( size3 > 0 ) chmura_D1->points.resize (size3);
	if ( size4 > 0 ) chmura_D2->points.resize (size4);
	
	//cout<< "XX0"<<endl;
	
	
	// Przepisanie punktów deskryptorów do chmury 2
	for(int j=0;j<size2;j++)
	{
		//cout<<"PKT "<<j<<"/"<<size3<<" " << descriptors[2][j].punkt.x<< " "<<descriptors[2][j].punkt.y<< " "<<descriptors[2][j].punkt.z<<endl;
		chmura_RGB2->points[j].x = descriptors[1][j].punkt.x ;
		chmura_RGB2->points[j].y = descriptors[1][j].punkt.y ;
		chmura_RGB2->points[j].z = descriptors[1][j].punkt.z ;
	}
	for(int j=0;j<size4;j++)
	{
		//cout<<"PKT "<<j<<"/"<<size4<<" " << descriptors[3][j].punkt.x<< " "<<descriptors[3][j].punkt.y<< " "<<descriptors[3][j].punkt.z<<endl;
		
		chmura_D2->points[j].x = descriptors[3][j].punkt.x ;
		chmura_D2->points[j].y = descriptors[3][j].punkt.y ;
		chmura_D2->points[j].z = descriptors[3][j].punkt.z ;
	}

	// Inicjalizacja FLANN dla NARF
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, kdtree2;
	
	// Liczymy error w stosunku do chmury deskryptorow 
	kdtree.setInputCloud(chmura_RGB2);
	if (ParyWybrane[1].size() >= 3) 	kdtree2.setInputCloud(chmura_D2);
	
	
	// Zmienne do zapisu najlepszych wartości z RANSACa
	Eigen::Matrix4f BestTransformation = Eigen::Matrix4f::Identity();
	float BestError = 99999999, BestProcentPunktow;
	
	srand(time(0));
	
	
	
	double ile_par_pop;
	if (ParyWybrane[1].size() >= 3) 
		// 0.5 *( mniejszy z 1 zestawu + mniejsze z 2 zestawu)
		ile_par_pop = 0.35 * ( (size1>size2 ? size2: size1) + (size3>size4 ? size4: size3));
	else
		ile_par_pop = 0.35 * (size1>size2 ? size2: size1);
		
	int wielkosc = ParyWybrane[0].size()+ParyWybrane[1].size(); // ile par do przeiterowania
	int ile_iteracji;
	
	cout<<"### " << ile_par_pop << "  " <<wielkosc<<endl;
	ile_iteracji = RANSACiter(ile_par_pop,wielkosc*1.0f);
	
	EndTime = pcl::getTime ();
	cout<<"<time> Przygotowanie RANSACa zajelo : "<<EndTime-StartTime<< " seconds " << endl;
	zapis<<"<time> Przygotowanie RANSACa zajelo : "<<EndTime-StartTime<< " seconds " << endl;
	
	cout<<" Propozycja iteracji z probabilistic model : " <<ile_iteracji<<endl;
	zapis<<" Propozycja iteracji z probabilistic model : " <<ile_iteracji<<endl;
	
	if(Parameters.iteration_number == 0)
			Parameters.iteration_number = ile_iteracji;
	
	// Glowna petla RANSACa
	for(int i=0;i<Parameters.iteration_number;i++)
	{
			//cout<<"ITER : " << i << endl;
			// Przepisanie punktów deskryptorów do chmury 1
			for(int j=0;j<size1;j++)
			{
				 chmura_RGB1->points[j].x = descriptors[0][j].punkt.x ;
				 chmura_RGB1->points[j].y = descriptors[0][j].punkt.y ;
				 chmura_RGB1->points[j].z = descriptors[0][j].punkt.z ;
			}
			for(int j=0;j<size3;j++)
			{
				 chmura_D1->points[j].x = descriptors[2][j].punkt.x ;
				 chmura_D1->points[j].y = descriptors[2][j].punkt.y ;
				 chmura_D1->points[j].z = descriptors[2][j].punkt.z ;
			}
			valid_indexes.clear();
			model.clear();
			
			for(int j=0;j< wielkosc ;j++) valid_indexes.push_back(true);
			
			if(Parameters.RANSAC == 0 )//|| i == 0)
			{
				// Najlepsze pod wzgledem podobienstwa
				for(int k=0;k<Parameters.NARF_pair_number;k++)
				{
					model.push_back(IndeksyNajlepszego[k]);
				}
				
				// TO DO
				// Gdy mamy 2 sety par to trzeba dac iteracje na 4 i pozdejmowac po kolei
			}
			else
			{
				
				// Znajdz pary uzyte do stworzenia modelu
				while(model.size() < Parameters.NARF_pair_number)
				{
					//int a = rand() % wielkosc; // Random number modulo liczby par
					int a = rand() % wielkosc;
					if (a < ParyWybrane[0].size())
					{
						
						if(valid_indexes[ParyWybrane[0][a].indeks1] == true && valid_indexes[ParyWybrane[0][a].indeks2] == true)
						{
							pcl::PointXYZ aaa,bbb,g,g2;
							// Sprawdzenie, czy na pewno punkty maja inne x y z
							bool good = true;
							//double EPS = numeric_limits<double>::epsilon();
							g.x = descriptors[0][ParyWybrane[0][a].indeks1].punkt.x;
							g.y = descriptors[0][ParyWybrane[0][a].indeks1].punkt.y;
							g.z = descriptors[0][ParyWybrane[0][a].indeks1].punkt.z;
							g2.x= descriptors[1][ParyWybrane[0][a].indeks2].punkt.x;
							g2.y= descriptors[1][ParyWybrane[0][a].indeks2].punkt.y;
							g2.z= descriptors[1][ParyWybrane[0][a].indeks2].punkt.z;
							
							/*for(int b=0;b<model.size();b++)
							{
								int t_t = model[b];
								int a = ParyWybrane[0][t_t].indeks1;
								int b = ParyWybrane[0][t_t].indeks2;
								
								good = DiffPoints(g,descriptors[0][a].punkt) && DiffPoints(g,descriptors[1][b].punkt);
							}*/
							/*for(int b=0;b<model.size();b++)
							{
								int t_t = model[b];
								if(t_t < ParyWybrane[0].size())
								{
									int a = ParyWybrane[0][t_t].indeks1;
									int b = ParyWybrane[0][t_t].indeks2;
									aaa = 
									bbb = descriptors[1][b].punkt;
								}
								else
								{
									t_t -= ParyWybrane[0].size();
									int a = ParyWybrane[1][t_t].indeks1;
									int b = ParyWybrane[1][t_t].indeks2;
									aaa = descriptors[2][a].punkt;
									bbb = descriptors[3][b].punkt;
								}

								if ( SquaredNorm(g,aaa) < EPS || SquaredNorm(g2,bbb) < EPS)
								{
									good = false;
									break;
								}
							}*/
							if(good)
							{
								valid_indexes[ParyWybrane[0][a].indeks1] = false;
								valid_indexes[ParyWybrane[0][a].indeks2] = false;
								model.push_back(a);	
							}				
						}
					}
					else
					{
							
						a-=ParyWybrane[0].size();
						if(valid_indexes[ParyWybrane[0].size() + ParyWybrane[1][a].indeks1] == true && 
							valid_indexes[ParyWybrane[0].size() + ParyWybrane[1][a].indeks2] == true)
						{
							pcl::PointXYZ aaa,bbb,g,g2;
							// Sprawdzenie, czy na pewno punkty maja inne x y z
							bool good = true;
							double EPS = numeric_limits<double>::epsilon();
							g.x = descriptors[2][ParyWybrane[1][a].indeks1].punkt.x;
							g.y = descriptors[2][ParyWybrane[1][a].indeks1].punkt.y;
							g.z = descriptors[2][ParyWybrane[1][a].indeks1].punkt.z;
							g2.x= descriptors[3][ParyWybrane[1][a].indeks2].punkt.x;
							g2.y= descriptors[3][ParyWybrane[1][a].indeks2].punkt.y;
							g2.z= descriptors[3][ParyWybrane[1][a].indeks2].punkt.z;
							
							
							/*for(int b=0;b<model.size();b++)
							{
								int t_t = model[b];
								if(t_t < ParyWybrane[0].size())
								{
									int a = ParyWybrane[0][t_t].indeks1;
									int b = ParyWybrane[0][t_t].indeks2;
									aaa = descriptors[0][a].punkt;
									bbb = descriptors[1][b].punkt;
								}
								else
								{
									t_t -= ParyWybrane[0].size();
									int a = ParyWybrane[1][t_t].indeks1;
									int b = ParyWybrane[1][t_t].indeks2;
									aaa = descriptors[2][a].punkt;
									bbb = descriptors[3][b].punkt;
								}

								if ( SquaredNorm(g,aaa) < EPS || SquaredNorm(g2,bbb) < EPS)
								{
									good = false;
									break;
								}
							}*/
							if(good)
							{
								valid_indexes[a+ParyWybrane[1][a].indeks1] = false;
								valid_indexes[a+ParyWybrane[1][a].indeks2] = false;
								model.push_back(a+ParyWybrane[0].size());	
							}						
						}
						
					}
					
					
				}
			}
				
			// Po wyborze musimy policzyc najlepsza transformacje dla danego modelu
			// Kabsch
			
			//cout << "KABSCH " <<endl;
			
			// Zapisać punkty wejściowe w wektorze
			Eigen::MatrixXf P(Parameters.NARF_pair_number,3),Q(Parameters.NARF_pair_number,3);
			for(int j=0;j<Parameters.NARF_pair_number;j++)
			{
				int p = model[j];
				int KtorySet = 0;
				if (p >= ParyWybrane[0].size()) // Czy z seta 1 czy z seta 2
				{
					KtorySet = 1;
					p-=ParyWybrane[0].size();
					
				}
				int indeks = ParyWybrane[KtorySet][p].indeks1;
				P(j, 0) = descriptors[KtorySet*2][indeks].punkt.x ;
				P(j, 1) = descriptors[KtorySet*2][indeks].punkt.y ;
				P(j, 2) = descriptors[KtorySet*2][indeks].punkt.z ; 
				
				indeks = ParyWybrane[KtorySet][p].indeks2;
				Q(j, 0) = descriptors[KtorySet*2+1][indeks].punkt.x ;
				Q(j, 1) = descriptors[KtorySet*2+1][indeks].punkt.y ;
				Q(j, 2) = descriptors[KtorySet*2+1][indeks].punkt.z ; 	
			}
			//cout<<endl<<"P : " << endl<<P <<endl<< "Q: " <<endl<<Q<<endl;
			Eigen::Matrix4f OPT;
			// Wyliczenie optymalnej transformacji do OPT
			Kabsch(P,Q,OPT);
			 
			float ERR  = 0;
			int inliers_number = 0;
			
			// Przesuniecie chmury NARF
			TransformSelf(chmura_RGB1,OPT);
			if (ParyWybrane[1].size() >= 3)  TransformSelf(chmura_D1,OPT);
				
			ERR  = 0;
			inliers_number = 0;

			// Dla wszystkich punktów z przesuniętej chmury1 znajdz najblizszy w chmurze 2
			for(int j=0;j<size1;j++)
			{
				vector<int> a;
				vector<float> odleglosc;
				
				if ( kdtree.nearestKSearch (chmura_RGB1->points[j], 1, a, odleglosc) > 0 )
				{
					// Sprawdzenie, ile mamy inlinierów
					if(odleglosc[0] < Parameters.Threshold)
					{
						// Policzenie fitness score dla tego modelu dla inlierów
						ERR += odleglosc[0];
						if(ERR>1000000) cout<<"ZAKRES"<<endl;
						inliers_number++;
					}					
				}
				// Gdy i tak nie bedzie mniejszego erroru
				if(ERR > BestError) break;
				// Gdy i tak nie spelni warunku procentow
				else if( (size3 + size1 - j - 1 + inliers_number) *100 < Parameters.NARF_minimal_matched * (size1+size3) ) break;
			}
			// Dla wszystkich punktów z przesuniętej chmury1 znajdz najblizszy w chmurze 2
			if (ParyWybrane[1].size() >= 3)  
				for(int j=0;j<size3;j++)
				{
					vector<int> a;
					vector<float> odleglosc;
					
					if ( kdtree2.nearestKSearch (chmura_D1->points[j], 1, a, odleglosc) > 0 )
					{
						// Sprawdzenie, ile mamy inlinierów
						if(odleglosc[0] < Parameters.Threshold)
						{
							// Policzenie fitness score dla tego modelu dla inlierów
							ERR += odleglosc[0];
							if(ERR>1000000) cout<<"ZAKRES"<<endl;
							inliers_number++;
						}
					}
					// Gdy i tak nie bedzie mniejszego erroru
					if(ERR > BestError) break;
					// Gdy i tak nie spelni warunku procentow
					else if( (size3 - j - 1 + inliers_number) *100 < Parameters.NARF_minimal_matched * (size1+size3) ) break;
				}
			float ProcentPunktow;
			ProcentPunktow = float(inliers_number)*100.0/(size1+size3);
			
			float Obrot[3];
			Oblicz_Katy(OPT, Obrot);
			
			Eigen::Vector3f teemp;
	
			teemp.head<3>() = OPT.block<1,3>(3,0);;
			
			//cout<<i<< 	" ERR = "<<ERR<<" | " << size1 << " " << size2 << " " << size3 << " " << size4 <<"| " <<inliers_number<<" | pr "<< ProcentPunktow<<endl;
			
			// Porownanie czy spelnia warunki podane w pliku .cfg i jest lepszym modelem od najlepszego
			if((Obrot[0] <= Parameters.constrain.maxObrX && Obrot[0] >= Parameters.constrain.minObrX && Obrot[1] <= Parameters.constrain.maxObrY && Obrot[1] >= Parameters.constrain.minObrY && Obrot[2] <= Parameters.constrain.maxObrZ && Obrot[2] >= Parameters.constrain.minObrZ &&
				teemp[0] <= Parameters.constrain.maxTx && teemp[0] >= Parameters.constrain.minTx && teemp[1] <= Parameters.constrain.maxTy && teemp[1] >= Parameters.constrain.minTy && teemp[2] <= Parameters.constrain.maxTz && teemp[2] >= Parameters.constrain.minTz && ProcentPunktow > Parameters.NARF_minimal_matched) || (Parameters.RANSAC == 0) )
			{
				
				if(ERR < BestError|| Parameters.RANSAC == 0)
				{
					BestError = ERR;
					BestTransformation = OPT;
					BestProcentPunktow = ProcentPunktow;
					
					IndeksyNajlepszego.clear();
					for(int z =0 ;z<model.size();z++) IndeksyNajlepszego.push_back(model[z]);
					
					LiczbaIteracjiBezPoprawy=0;
				}
				else LiczbaIteracjiBezPoprawy++;
				
				// Dodanie modelu do listy modeli
				Model p;
				p.Error = ERR;
				p.Transformation = OPT;
				p.ProcentPunktow = ProcentPunktow;
				for (int k=0;k<model.size();k++) p.indeksy_par.push_back(model[k]);
					
				ListaPoprawnychModeli.push_back(p);
			}
			else LiczbaIteracjiBezPoprawy++;
			
			// Nie nastapiła poprawa przez N iteracji to break
			if(LiczbaIteracjiBezPoprawy>Parameters.MaxLiczbaIteracjiBezPoprawy && BestError<Parameters.RANSACerror) 
			{
				cout<<"RANSAC did "<<i<<" iteracji"<<endl;
				zapis<<"RANSAC did "<<i<<" iteracji"<<endl;	
				break;
			}
	}
	EndTime = pcl::getTime ();
	cout<<"<time>  RANSAC zajal : "<<EndTime-StartTime<<endl;
	zapis<<"<time>  RANSAC zajal : "<<EndTime-StartTime<<endl;
	StartTime = EndTime;
	
	cout<<endl<<endl<<"-------- Best model --------"<<endl;
	cout<<"-------- ProPkt : "<<BestProcentPunktow <<endl;
	cout<<"-------- ERR : "<<BestError<<endl<<"TRANSFORMACJA :"<<endl<<BestTransformation<<endl;
	
	zapis<<endl<<endl<<"-------- Best model --------"<<endl;
	zapis<<"-------- ProPkt : "<<BestProcentPunktow <<endl;
	zapis<<"-------- ERR : "<<BestError<<endl<<"TRANSFORMACJA :"<<endl<<BestTransformation<<endl;

	// Najlepsza transformacja z RANSACA uzyta do przesuniecia chmury 1
	pcl::PointCloud<pcl::PointXYZ>::Ptr tym;
	
	// Start time
	StartTime = pcl::getTime ();
			
	// Przekształcenie chmury 1 przez zadany obrót i translacje
	tym = Transform(chmura[0], BestTransformation);
	  
	EndTime = pcl::getTime ();
	cout<<"<time> Zastosowanie przeksztalcenia zajelo : "<<EndTime-StartTime<< " seconds " << endl;
	zapis<<"<time> Zastosowanie przeksztalcenia zajelo : "<<EndTime-StartTime<< " seconds " << endl;
	
	// Wypisanie obrotów dla najlepszego model
	float Orient[3];
	  
	Oblicz_Katy(BestTransformation, Orient);
	cout<<"Deskryptory NARF sugerują takie obroty w osiach:"<<endl;	
	cout<<"Obrót w osi x "<<Orient[0]<<endl;
	cout<<"Obrót w osi y "<<Orient[1]<<endl;
	cout<<"Obrót w osi z "<<Orient[2]<<endl;
	
	zapis<<"Deskryptory NARF sugerują takie obroty w osiach:"<<endl;	
	zapis<<"Obrót w osi x "<<Orient[0]<<endl;
	zapis<<"Obrót w osi y "<<Orient[1]<<endl;
	zapis<<"Obrót w osi z "<<Orient[2]<<endl;	  
	
	
		
	  
	cout<<endl<<"ICP dla NARF starting pose"<<endl;
			
	// Start time
	StartTime = pcl::getTime ();
	
	// Algorytm IPC
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f final, ICPresult;
	
	icp = ICP(tym,chmura[1],ICPresult, Parameters.OutRejThre, Parameters.ICPepsilon, Parameters.ICP_max_iterations, Parameters.maxCorrespondeceDist);
	  
	EndTime = pcl::getTime ();
	cout<<"<time> ICP took : "<<EndTime-StartTime<< " seconds " << endl;
	zapis<<"<time> ICP took : "<<EndTime-StartTime<< " seconds " << endl;
	
	cout<< "Macierz transformacji po NARF + ICP" <<endl;
	zapis << "Macierz transformacji po NARF + ICP" <<endl;
	
	 // Przesuniecie koncowe = Przeksztalcenie_ICP * Przeksztalcenie_Deskryptorow

	final = ICPresult * BestTransformation;
	  
	cout << final << endl;
	zapis<< final << endl;
	 
	// Zapisanie jakim obrotom to odpowiada
	Oblicz_Katy(final, Orient);
	cout<<"Obrót w osi x "<<Orient[0]<<endl;
	cout<<"Obrót w osi y "<<Orient[1]<<endl;
	cout<<"Obrót w osi z "<<Orient[2]<<endl;

	zapis<<"Obrót w osi x "<<Orient[0]<<endl;
	zapis<<"Obrót w osi y "<<Orient[1]<<endl;
	zapis<<"Obrót w osi z "<<Orient[2]<<endl;
	
	
	// ########
// Chmurki + keypointy 1 i 2
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);

	
	
	// Przepisanie punktów deskryptorów do chmury 1
	for(int j=0;j<size1;j++)
	{
		chmura_RGB1->points[j].x = descriptors[0][j].punkt.x ;
		chmura_RGB1->points[j].y = descriptors[0][j].punkt.y ;
		chmura_RGB1->points[j].z = descriptors[0][j].punkt.z ;
	}
	for(int j=0;j<size2;j++)
	{
		chmura_RGB2->points[j].x = descriptors[1][j].punkt.x ;
		chmura_RGB2->points[j].y = descriptors[1][j].punkt.y ;
		chmura_RGB2->points[j].z = descriptors[1][j].punkt.z ;
	}
	for(int j=0;j<size3;j++)
	{
		chmura_D1->points[j].x = descriptors[2][j].punkt.x ;
		chmura_D1->points[j].y = descriptors[2][j].punkt.y ;
		chmura_D1->points[j].z = descriptors[2][j].punkt.z ;
	}
	for(int j=0;j<size4;j++)
	{
		chmura_D2->points[j].x = descriptors[3][j].punkt.x ;
		chmura_D2->points[j].y = descriptors[3][j].punkt.y ;
		chmura_D2->points[j].z = descriptors[3][j].punkt.z ;
	}
	// Chmura docelowa - red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler_ (chmura[1], 255, 0, 0);
	viewer.addPointCloud (chmura[1], point_cloud_color_handler_, "original point cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
	  
	// Chmura poczatkowa - green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler_2 (chmura[0], 0, 255, 0);
	viewer.addPointCloud (chmura[0], point_cloud_color_handler_2, "original point cloud2");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud2");
		
	// keypointy chmura poczatkowa - green i blue
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (chmura_RGB1, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ> (chmura_RGB1, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler2 (chmura_D1, 0, 0, 255);
	viewer.addPointCloud<pcl::PointXYZ> (chmura_D1, keypoints_color_handler2, "keypoints2");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints2");
	
	// keypointy chmura docelowa - red i red+blue
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler3 (chmura_RGB2, 255, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ> (chmura_RGB2, keypoints_color_handler3, "keypoints3");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints3");
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler4 (chmura_D2, 255, 0, 255);
	viewer.addPointCloud<pcl::PointXYZ> (chmura_D2, keypoints_color_handler4, "keypoints4");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints4");
	
	// keypointy wybrane i dopasowanie
	pcl::PointCloud<pcl::PointXYZ>::Ptr LiniaPocz(new pcl::PointCloud<pcl::PointXYZ>), LiniaKoniec(new pcl::PointCloud<pcl::PointXYZ>);
	LiniaPocz->points.resize (Parameters.NARF_pair_number);
	LiniaKoniec->points.resize (Parameters.NARF_pair_number);
	for(int k=0;k<IndeksyNajlepszego.size();k++)
	{
		int xyz = IndeksyNajlepszego[k];
		int KtorySet = 0;
		if ( xyz >= ParyWybrane[0].size() )
		{
			xyz -= ParyWybrane[0].size();
			KtorySet = 1;
		}
		LiniaPocz->points[k].x = descriptors[KtorySet*2][ ParyWybrane[KtorySet][xyz].indeks1 ].punkt.x;
		LiniaPocz->points[k].y = descriptors[KtorySet*2][ ParyWybrane[KtorySet][xyz].indeks1 ].punkt.y;
		LiniaPocz->points[k].z = descriptors[KtorySet*2][ ParyWybrane[KtorySet][xyz].indeks1 ].punkt.z;
		LiniaKoniec->points[k].x = descriptors[KtorySet*2+1][ ParyWybrane[KtorySet][xyz].indeks2 ].punkt.x;
		LiniaKoniec->points[k].y = descriptors[KtorySet*2+1][ ParyWybrane[KtorySet][xyz].indeks2 ].punkt.y;
		LiniaKoniec->points[k].z = descriptors[KtorySet*2+1][ ParyWybrane[KtorySet][xyz].indeks2 ].punkt.z;
	}
	
	// Wybrane keypointy do stworzenia modelu z chmury 1
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> LiniaPocz_color_handler (LiniaPocz, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ> (LiniaPocz , LiniaPocz_color_handler, "LiniaPocz");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "LiniaPocz");
	
	// wybrane keypointy do stworzenia modelu z chmury 2
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> LiniaPocz_color_handler2 (LiniaKoniec, 255, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ> (LiniaKoniec , LiniaPocz_color_handler2, "LiniaKoniec");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "LiniaKoniec");	
	for(int k=0;k<IndeksyNajlepszego.size();k++)
	{
		const string www = "bbb" + char(k);
		viewer.addLine(LiniaPocz->points[k],LiniaKoniec->points[k], www);
	}
	// SHOW 1
	viewer.initCameraParameters ();
	setViewerPose (viewer);  
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}
// #
// Po transformacji
// #

	pcl::visualization::PCLVisualizer viewer2 ("3D Viewer");
	viewer2.setBackgroundColor (1, 1, 1);
	
	TransformSelf(chmura_RGB1, BestTransformation);
	TransformSelf(chmura_D1, BestTransformation);
	TransformSelf(chmura[0], BestTransformation);
	
	// Docelowa chmura
	viewer2.addPointCloud (chmura[1], point_cloud_color_handler_, "original point cloud");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
	  
	// chmura poczatkowa po matchingu
	viewer2.addPointCloud (chmura[0], point_cloud_color_handler_2, "original point cloud2");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud2");
		
	// keypointy chmury poczatkowej po matchingu
	viewer2.addPointCloud<pcl::PointXYZ> (chmura_RGB1, keypoints_color_handler, "keypoints");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");

	viewer2.addPointCloud<pcl::PointXYZ> (chmura_D1, keypoints_color_handler2, "keypoints2");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints2");
	
	// keypointy docelowe
	viewer2.addPointCloud<pcl::PointXYZ> (chmura_RGB2, keypoints_color_handler3, "keypoints3");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints3");
	
	viewer2.addPointCloud<pcl::PointXYZ> (chmura_D2, keypoints_color_handler4, "keypoints4");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints4");
	
	// keypointy wybrane i dopasowanie
	TransformSelf(LiniaPocz,BestTransformation);
	
	// Linia
	viewer2.addPointCloud<pcl::PointXYZ> (LiniaPocz , LiniaPocz_color_handler, "LiniaPocz");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "LiniaPocz");
	
	viewer2.addPointCloud<pcl::PointXYZ> (LiniaKoniec , LiniaPocz_color_handler2, "LiniaKoniec");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "LiniaKoniec");	
	
	for(int k=0;k<IndeksyNajlepszego.size();k++)
	{
		const string www = "bbb" + char(k);
		viewer2.addLine(LiniaPocz->points[k],LiniaKoniec->points[k], www);
	}
	 // SHOW 1
	viewer2.initCameraParameters ();
	setViewerPose (viewer2);  
	while (!viewer2.wasStopped ())
	{
		viewer2.spinOnce ();
		pcl_sleep(0.01);
	}
	
// #
// Po ICP
// #

	pcl::visualization::PCLVisualizer viewer3 ("3D Viewer");
	viewer3.setBackgroundColor (1, 1, 1);
	
	TransformSelf(chmura_RGB1, ICPresult);
	TransformSelf(chmura_D1, ICPresult);
	TransformSelf(chmura[0], ICPresult);
	
	// Docelowa chmura
	viewer3.addPointCloud (chmura[1], point_cloud_color_handler_, "original point cloud");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
	  
	// chmura po matchingu + ICP
	viewer3.addPointCloud (chmura[0], point_cloud_color_handler_2, "original point cloud2");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud2");
		
	// keypointy chmury poczatkowej po matchingu + ICP
	viewer3.addPointCloud<pcl::PointXYZ> (chmura_RGB1, keypoints_color_handler, "keypoints");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");

	viewer3.addPointCloud<pcl::PointXYZ> (chmura_D1, keypoints_color_handler2, "keypoints2");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints2");
	
	// keypointy docelowe 
	viewer3.addPointCloud<pcl::PointXYZ> (chmura_RGB2, keypoints_color_handler3, "keypoints3");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints3");
	
	viewer3.addPointCloud<pcl::PointXYZ> (chmura_D2, keypoints_color_handler4, "keypoints4");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints4");
	
	// keypointy wybrane i dopasowanie
	TransformSelf(LiniaPocz,ICPresult);
	
	// Linia
	viewer3.addPointCloud<pcl::PointXYZ> (LiniaPocz , LiniaPocz_color_handler, "LiniaPocz");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "LiniaPocz");
	
	viewer3.addPointCloud<pcl::PointXYZ> (LiniaKoniec , LiniaPocz_color_handler2, "LiniaKoniec");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "LiniaKoniec");	
	
	for(int k=0;k<IndeksyNajlepszego.size();k++)
	{
		const string www = "bbb" + char(k);
		viewer3.addLine(LiniaPocz->points[k],LiniaKoniec->points[k], www);
	}
	 // SHOW 3
	viewer3.initCameraParameters ();
	setViewerPose (viewer3);  
	while (!viewer3.wasStopped ())
	{
		viewer3.spinOnce ();
		pcl_sleep(0.01);
	}
	
	cout<<"Indeksy najlepszego " << ParyWybrane[0].size()<< " "<<ParyWybrane[1].size()<<endl;
	for(int i=0;i<IndeksyNajlepszego.size();i++)
	{
		if (IndeksyNajlepszego[i]< ParyWybrane[0].size() ) cout<<"1 set par : " <<IndeksyNajlepszego[i]<<endl ;
		else cout<<"2 set par : " <<IndeksyNajlepszego[i]<<endl ;
	}
	cout<<endl;
	
	cout<< "START "<<wielkosc<<endl;
	int ilosc = 0;
	for(int w=0;w<wielkosc;w++)
	{
		int a = w;
		if (a < ParyWybrane[0].size())
		{				
			pcl::PointXYZ g,g2;
			// Sprawdzenie, czy na pewno punkty maja inne x y z
			double EPS = numeric_limits<double>::epsilon();
							
							
			Eigen::VectorXf punkt(4);
			punkt[0] = descriptors[0][ParyWybrane[0][a].indeks1].punkt.x;
			punkt[1] = descriptors[0][ParyWybrane[0][a].indeks1].punkt.y;
			punkt[2] = descriptors[0][ParyWybrane[0][a].indeks1].punkt.z;
			punkt[3] = 1;
			g2.x= descriptors[1][ParyWybrane[0][a].indeks2].punkt.x;
			g2.y= descriptors[1][ParyWybrane[0][a].indeks2].punkt.y;
			g2.z= descriptors[1][ParyWybrane[0][a].indeks2].punkt.z;
							
			// Zastosowanie optymalnej transformacji
			punkt = BestTransformation * punkt;
				
			// Przepisanie nowej wartości punktu do chmury
			g.x = punkt[0];
			g.y = punkt[1];
			g.z = punkt[2];	
	
			if ( SquaredNorm(g,g2) < 0.1)
			{
				ilosc++ ;
			}
		}
		else
		{	
			a-=ParyWybrane[0].size();
					
			pcl::PointXYZ g,g2;
			// Sprawdzenie, czy na pewno punkty maja inne x y z
		
			double EPS = numeric_limits<double>::epsilon();
			
			Eigen::VectorXf punkt(4);
			punkt[0] =  descriptors[2][ParyWybrane[1][a].indeks1].punkt.x;
			punkt[1] = descriptors[2][ParyWybrane[1][a].indeks1].punkt.y;
			punkt[2] = descriptors[2][ParyWybrane[1][a].indeks1].punkt.z;
			punkt[3] = 1;
			g2.x= descriptors[3][ParyWybrane[1][a].indeks2].punkt.x;
			g2.y= descriptors[3][ParyWybrane[1][a].indeks2].punkt.y;
			g2.z= descriptors[3][ParyWybrane[1][a].indeks2].punkt.z;
							
			// Zastosowanie optymalnej transformacji
			punkt = BestTransformation * punkt;
				
			// Przepisanie nowej wartości punktu do chmury
			g.x = punkt[0];
			g.y = punkt[1];
			g.z = punkt[2];	
	
			if ( SquaredNorm(g,g2) < 0.1 )
			{
				ilosc++ ;
			}
		}
	}
						
	cout<<"Pokrytych par : " << ilosc<<endl;
	
	//cout<<KinectInitial.inverse()
}	

void Kinect::FeatureMatchingWithKnownCorrespondeces()
{
	
	cout<<"Start feature matching"<<endl;

	double StartTime;
	
	// Start time
	StartTime = pcl::getTime();
	
	vector< cv::DMatch > matches;
	vector< cv::DMatch > good_matches;
	
	cv::FlannBasedMatcher matcher;
	matcher.match( (RGBD->descriptors[0]), (RGBD->descriptors[1]), matches );
	double max_dist = 0; double min_dist = 100;
	
	//cout<<"--- Matches = Found : " << matches.size()<<endl;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < (RGBD->descriptors[0]).rows; i++ )
	{
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	//cout<<" # " << min_dist<<endl;
	for( int i = 0; i < (RGBD->descriptors[0]).rows; i++ )
	{
		// Euclidean distance
		int a = matches[i].queryIdx;
		int b = matches[i].trainIdx;
		if( matches[i].distance < 3*min_dist && SquaredNorm((RGBD->Keypoints3d[0][a]), (RGBD->Keypoints3d[1][b])) < Parameters.constrain.maxTx*Parameters.constrain.maxTx + Parameters.constrain.maxTy*Parameters.constrain.maxTy + Parameters.constrain.maxTz*Parameters.constrain.maxTz)
		{
			good_matches.push_back( matches[i]);
		}
	}
	int RGBmatches = good_matches.size();
	
	// D
	matcher.match( (RGBD->descriptors[2]), (RGBD->descriptors[3]), matches );
	max_dist = 0;
	min_dist = 100;
	
	//cout<<"--- Matches = Found : " << matches.size()<<endl;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < (RGBD->descriptors[2]).rows; i++ )
	{
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	//cout<<" # " << min_dist<<endl;
	for( int i = 0; i < (RGBD->descriptors[2]).rows; i++ )
	{
		// Euclidean distance
		int a = matches[i].queryIdx;
		int b = matches[i].trainIdx;
		if( matches[i].distance < 5*min_dist && SquaredNorm((RGBD->Keypoints3d[0][a]), (RGBD->Keypoints3d[1][b])) < Parameters.constrain.maxTx*Parameters.constrain.maxTx + Parameters.constrain.maxTy*Parameters.constrain.maxTy + Parameters.constrain.maxTz*Parameters.constrain.maxTz)
		{
			good_matches.push_back( matches[i]);
		}
	}
	//cout<<"--- GOOD_Matches = Found : " << good_matches.size()<<endl;
	
	double EndTime = pcl::getTime();
	cout<<endl<<"Flann based matcher: " << EndTime  - StartTime<<endl;
	StartTime = pcl::getTime();
	
	float wszystkich = good_matches.size();
	float poprawnych = wszystkich / 4;
	int ile_iteracji = log(1-0.98) / log(1-(poprawnych/wszystkich)*(poprawnych-1)/(wszystkich-1))+1;
	
	//cout<<" Propozycja iteracji z probabilistic model : " <<ile_iteracji<<endl;
	//zapis<<" Propozycja iteracji z probabilistic model : " <<ile_iteracji<<endl;
	
	Eigen::Matrix4f BestTransformation;
	int BestLiczbaDopasowan = 0;
	vector<int> valid_indexes;
	vector<int> model;
	vector<int> NajlepszeIndeksy;
	
	vector<bool> inliers, bestinliers;
	for (int i=0;i<wszystkich;i++)	
		inliers.push_back(false);
		
	// Glowna petla RANSACa
	for(int i=0;i<ile_iteracji;i++)
	{
			valid_indexes.clear();
			model.clear();
			
			for(int j=0;j< good_matches.size() ;j++) valid_indexes.push_back(true);
			
			// Znajdz pary uzyte do stworzenia modelu
			while(model.size() < Parameters.NARF_pair_number)
			{
				int a = rand() % good_matches.size();
				
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
			Eigen::MatrixXf P(Parameters.NARF_pair_number,3),Q(Parameters.NARF_pair_number,3);
			
			for(int j=0;j<Parameters.NARF_pair_number;j++)
			{
				int p = model[j];
				
				int a = good_matches[p].queryIdx;
				// JEZELI D to
				if (p>RGBmatches) a+=RGBmatches;
				P(j, 0) = RGBD->Keypoints3d[0][a].x ;
				P(j, 1) = RGBD->Keypoints3d[0][a].y ;
				P(j, 2) = RGBD->Keypoints3d[0][a].z ; 
				
				int b = good_matches[p].trainIdx;
				// JEZELI D to
				if (p>RGBmatches) b+=RGBmatches;
				Q(j, 0) = RGBD->Keypoints3d[1][b].x ;
				Q(j, 1) = RGBD->Keypoints3d[1][b].y ;
				Q(j, 2) = RGBD->Keypoints3d[1][b].z ; 	
			}

			Eigen::Matrix4f OPT;
			// Wyliczenie optymalnej transformacji do OPT
			Kabsch(P,Q,OPT);
			 
			float Obrot[3];
			Oblicz_Katy(OPT, Obrot);
			
			Eigen::Vector3f teemp;
	
			teemp.head<3>() = OPT.block<1,3>(3,0);


			int LiczbaDopasowan = 0;
			for(int w=0;w<good_matches.size();w++)
			{
				int a = good_matches[w].queryIdx;
				int b = good_matches[w].trainIdx;
				
				// JEZELI D to
				if (w>RGBmatches) 
				{	
					a+=RGBmatches;
					b+=RGBmatches;
				}
				pcl::PointXYZ xyz;
				Eigen::VectorXf punkt(4);
				punkt[0] = RGBD->Keypoints3d[0][a].x;
				punkt[1] = RGBD->Keypoints3d[0][a].y;
				punkt[2] = RGBD->Keypoints3d[0][a].z;
				punkt[3] = 1;
				
				// Zastosowanie optymalnej transformacji
				punkt = OPT * punkt;
				
				// Przepisanie nowej wartości punktu do chmury
				xyz.x = punkt[0];
				xyz.y = punkt[1];
				xyz.z = punkt[2];
	
				if ( SquaredNorm(xyz,RGBD->Keypoints3d[1][b]) < 0.01)
				{
					LiczbaDopasowan++ ;
					inliers[w] = true;
				}
				else
				{
					inliers[w] = false;
				}
			}
			
			// Porownanie czy spelnia warunki podane w pliku .cfg i jest lepszym modelem od najlepszego
			if(Obrot[0] <= Parameters.constrain.maxObrX && Obrot[0] >= Parameters.constrain.minObrX && Obrot[1] <= Parameters.constrain.maxObrY &&
				Obrot[1] >= Parameters.constrain.minObrY && Obrot[2] <= Parameters.constrain.maxObrZ && Obrot[2] >= Parameters.constrain.minObrZ &&
				teemp[0] <= Parameters.constrain.maxTx && teemp[0] >= Parameters.constrain.minTx && teemp[1] <= Parameters.constrain.maxTy && 
				teemp[1] >= Parameters.constrain.minTy && teemp[2] <= Parameters.constrain.maxTz && teemp[2] >= Parameters.constrain.minTz)
			{
				
				if(LiczbaDopasowan >  BestLiczbaDopasowan)
				{
					BestTransformation = OPT;
					BestLiczbaDopasowan = LiczbaDopasowan;
					NajlepszeIndeksy.clear();
					bestinliers = inliers;
					for(int j=0;j<Parameters.NARF_pair_number;j++)
					{
						NajlepszeIndeksy.push_back(model[j]);
					}
				}
			}
	}
	EndTime = pcl::getTime();
	cout<<endl<<"RANSAC time : " << EndTime  - StartTime<<endl;
	//cout<<endl<<"TRANSFORMACJA :"<<endl<<BestTransformation<<endl;
	cout<<endl<<"Liczba Dopasowanych: " << BestLiczbaDopasowan << " na " << good_matches.size()<<endl;
	for(int i=0;i<NajlepszeIndeksy.size();i++)
	{
		cout<<"PARA: "<<NajlepszeIndeksy[i]<<" - ";
		if(NajlepszeIndeksy[i]<RGBmatches) cout<< "RGB"<<endl;
		else cout<< "D"<<endl;
	}
	
	//cout<<endl<<"TRANSFORMACJA po przeliczeniu :"<<endl<<KinectInitial*BestTransformation<<endl;
	zapis<<BestTransformation;
	
	
	BestTransformation = KinectInitial * BestTransformation;
	cout<<BestTransformation<<endl;
	cout<<endl<<"TRANSFORMACJA z matchingu:"<<endl<<BestTransformation<<endl;
	float Obrot[3];
	Oblicz_Katy(BestTransformation, Obrot);
	cout<<"Obroty z matchingu: "<<Obrot[0]<< " " << Obrot[1]<< " "<<Obrot[2]<<endl;
	cout<<"---------------------------------"<<endl;
	//TransformSelf(chmura_color[0], BestTransformation);
	//TransformSelf(chmura_color[1], KinectInitial);
	//*chmura_color[0] = *chmura_color[0] + *chmura_color[1];
	//ShowCloudXYZRGBA(chmura_color[0]);
	
	
	Eigen::MatrixXf P(BestLiczbaDopasowan,3),Q(BestLiczbaDopasowan,3);
	Eigen::Matrix4f OPT;
	for (int i=0, k = 0;i<good_matches.size();i++)
	{
		if (bestinliers[i] == true)
		{
			int a = good_matches[i].queryIdx;
			int b = good_matches[i].trainIdx;
				
			// JEZELI D to
			if (i>RGBmatches) 
			{	
				a+=RGBmatches;
				b+=RGBmatches;
			}

				
			P(k, 0) = RGBD->Keypoints3d[0][a].x;
			P(k, 1) = RGBD->Keypoints3d[0][a].y;
			P(k, 2) = RGBD->Keypoints3d[0][a].z;
								
			Q(k, 0) = RGBD->Keypoints3d[1][b].x;
			Q(k, 1) = RGBD->Keypoints3d[1][b].y;
			Q(k, 2) = RGBD->Keypoints3d[1][b].z;
			
			k++;	
		}
	}
	Kabsch(P,Q,OPT, BestLiczbaDopasowan);
	cout<<endl<<"TRANSFORMACJA matchingu z inlierow :"<<endl<<OPT<<endl;
	Obrot[3];
	Oblicz_Katy(OPT, Obrot);
	cout<<"Obroty matchingu z inlierow: "<<Obrot[0]<< " " << Obrot[1]<< " "<<Obrot[2]<<endl;
	
}	

pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect::ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr in2,Eigen::Matrix4f &final, double OutRejThre, double Epsilon, double Max_iter,double maxCorrespondeceDist)
{
	
	cout<<"ICP start"<<endl;
	zapis<<"*********************"<<endl;
	zapis<<"***   ICP START   ***"<<endl;
	zapis<<"*********************"<<endl;
	  
	 
	// Obiekt ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	  
	// Parametry
	icp.setRANSACOutlierRejectionThreshold (OutRejThre);
	icp.setTransformationEpsilon (Epsilon);
	icp.setMaximumIterations (Max_iter);
	icp.setMaxCorrespondenceDistance (maxCorrespondeceDist);
	  
	// chmura wejsciowa dla ICP
	icp.setInputCloud(in);
		
	// chmura targetowa dla ICP
	icp.setInputTarget(in2);
	
	// Z input1 do input2
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
	
	
	double StartTime = pcl::getTime ();
	icp.align(*Final);
	double EndTime = pcl::getTime ();
	cout<<"<time> ICP : "<<EndTime-StartTime<<endl;
	zapis<<"<time> ICP : "<<EndTime-StartTime<<endl;

	cout << "ICP has converged:" << icp.hasConverged() << endl << "Score: "<< icp.getFitnessScore() << endl;
	zapis<< "ICP has converged:" << icp.hasConverged() << endl << "Score: "<< icp.getFitnessScore() << endl;
	  
	
	  
	cout<< "Macierz transformacji" <<endl;
	zapis << "Macierz transformacji" <<endl;
	  
	final=icp.getFinalTransformation();
	  
	cout << final << endl;
	zapis<< final << endl;
	  
	  
	float Orient[3];
	Oblicz_Katy(final, Orient);
	cout<<"Obrot w osi x "<<Orient[0]<<endl;
	cout<<"Obrot w osi y "<<Orient[1]<<endl;
	cout<<"Obrot w osi z "<<Orient[2]<<endl;

	zapis<<"Obrot w osi x "<<Orient[0]<<endl;
	zapis<<"Obrot w osi y "<<Orient[1]<<endl;
	zapis<<"Obrot w osi z "<<Orient[2]<<endl;
	   
	return Final;
}

int Kinect::RANSACiter(double poprawnych, double wszystkich)
{
	return (log(1-Parameters.RANSACsuccessrate) / log(1-(poprawnych/wszystkich)*(poprawnych-1)/(wszystkich-1) *(poprawnych-2)/(wszystkich-2)))+1;  
}

///
/// Loading data
///

void Kinect::GetContinuousClouds()
{
	// tworzymy strukturę dla przechwycenia danych z urządzenia
	StrukturaKinectaXYZRGBA Kinect_Data;
	// tworzymy interfejs  który posluży do komunikacji z Kinectem
	pcl::OpenNIGrabber grabber;
	// obiekt do którego podłączamy streaming z kinecta	
	boost::signals2::connection KinectStream;
	
	// Dziwny callback
    KinectStream = grabber.registerCallback (boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>(boost::bind(&StrukturaKinectaXYZRGBA::callback,&Kinect_Data, _1)));
 
    // podpinamy strukturę Kinect_Data jako docelową dla streamingu danych z kinecta
   	Kinect_Data.Streaming=1;

    // Uruchomienie polaczenia z Kinectem
    grabber.start();
    
    // Niemy getchar :D
	getchar();

	for(int h=0;true;h++)
	{
		cout<<"Wciśnij enter, aby zapisać " <<h+1<<" chmurę"<<endl;
		getchar();	  

		while(Kinect_Data.Streaming==1){}

		// Kinect disconnect
		KinectStream.disconnect();
	
		char nazwapliku[30];
		sprintf(nazwapliku,"chmura_%d.pcd",h);
		pcl::io::savePCDFileASCII (nazwapliku,*Kinect_Data.pc);

		KinectStream = grabber.registerCallback (boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>(boost::bind(&StrukturaKinectaXYZRGBA::callback,&Kinect_Data, _1)));
	  }
}


void Kinect::PobierzZKinecta(int x)
{		
	// tworzymy strukturę dla przechwycenia danych z urządzenia
	StrukturaKinectaXYZRGBA Kinect_Data;
	// tworzymy interfejs  który posluży do komunikacji z Kinectem
	pcl::OpenNIGrabber grabber;
	// obiekt do którego podłączamy streaming z kinecta	
	boost::signals2::connection KinectStream;
	
	// Dziwny callback
    KinectStream = grabber.registerCallback (boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>(boost::bind(&StrukturaKinectaXYZRGBA::callback,&Kinect_Data, _1)));
 
    // podpinamy strukturę Kinect_Data jako docelową dla streamingu danych z kinecta
   	Kinect_Data.Streaming=1;

    // Uruchomienie polaczenia z Kinectem
    grabber.start();

	cout<<"Wcisnij enter, aby zapisac chmure"<<endl;
	getchar(); 
	
	while(Kinect_Data.Streaming==1){}

	// Kinect disconnect
	KinectStream.disconnect();
	
	char *str;
	sprintf(str, "chmura_kinecta_%d.pcd", x);

	// Zapis chmury punktow
	pcl::io::savePCDFileASCII (str,*Kinect_Data.pc);

	printf("Zapisano chmure prosto z kinecta\n");
		
	*chmura_color[x] = *Kinect_Data.pc;
	copyPointCloud(*chmura_color[x] , *chmura[x]);
		
	Filtrowanie(x);
}

void Kinect::ReadCloudsXYZRGBA(int x)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudcolor (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xxx (new pcl::PointCloud<pcl::PointXYZ>);
	
	string nazwa;
	cout<<"Podaj nazwe " << x << " chmury RGBA: np. chmura.pcd"<<endl;
	cin>>nazwa;

	pcl::io::loadPCDFile<pcl::PointXYZRGBA> (nazwa, *cloudcolor);

	chmura[x] = xxx;
	chmura_color[x] = cloudcolor;
	copyPointCloud(*chmura_color[x] , *chmura[x]);


	/*for (int i = 0;i<chmura[1]->points.size();i++)
			if (chmura[1]->points[i].z > 2.2 )
				chmura[1]->points[i].x = chmura[1]->points[i].y = chmura[1]->points[i].z = 0;
				
	for (int i = 0;i<chmura[0]->points.size();i++)
			if (chmura[0]->points[i].z > 2.2 )
				chmura[0]->points[i].x = chmura[0]->points[i].y = chmura[0]->points[i].z = 0;*/
				
	cout<<"Size : " << chmura_color[0]->points.size()<<endl;
	Filtrowanie(x);
	cout<<"Size po filtracji " << chmura[x]->points.size()<<endl;
}

void Kinect::ReadCloudsXYZ(int x)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	string nazwa;
	cout<<"Podaj nazwe " <<x <<" chmury: np. chmura.pcd"<<endl;
	cin>>nazwa;

	pcl::io::loadPCDFile<pcl::PointXYZ> (nazwa, *cloud);
	
	chmura[x] = cloud;
	
	Filtrowanie(x);
}

void Kinect::ReadCloudsXYZ(int x, string name)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ> (name, *cloud);
	
	chmura[x] = cloud;
	
	Filtrowanie(x);
}

///
/// Saving data
///

void Kinect::SavePointCloud(int which)
{
	char nazwapliku[30];
	sprintf(nazwapliku,"cloud_%d.pcd",which);
	// Zapis point cloud
	pcl::io::savePCDFileASCII (nazwapliku,*chmura[which]);
}

void Kinect::SavePointCloudColor(int which)
{
	char nazwapliku[30];
	sprintf(nazwapliku,"cloudcolor_%d.pcd",which);
	// Zapis point cloud
	pcl::io::savePCDFileASCII (nazwapliku,*chmura_color[which]);
}


///
/// TRACKING
///

void Kinect::TrackingRun()
{
	Track *tr;
	tr = new Track();
	
	//1311878194.3308 -2.3174 -2.1669 0.5873 -0.7164 0.2093 -0.1835 0.6398
	// Expected
	//1311878195.8202 -2.3078 -2.1795 0.5876 -0.6829 0.2973 -0.2661 0.6120
	
	// GT * OPT^-1
	// -2.31913 -2.17148 0.581527 -0.679413 0.300578 -0.27038 0.612393
	
	
	// DESK initial 1.3112 0.8507 1.5186 0.8851 0.2362 -0.0898 -0.3909
	// LUB 1.3161 0.8465 1.5198 0.8866 0.2293 -0.0894 -0.3917
	// ROOM initial -0.8680 0.6052 1.5630 0.8225 -0.3882 0.1622 -0.3828
	
	Eigen::Quaternion<float> Q_ST(-0.3882, 0.1622 , -0.3828 ,0.8225);
	KinectInitial.block<3,3>(0,0) = Q_ST.toRotationMatrix();
	KinectInitial(0,3) = -0.8680;
	KinectInitial(1,3) = 0.6052 ;
	KinectInitial(2,3) = 1.5630 ;
	
	
	Eigen::Quaternion<float> Q_2(0.6120,-0.6829 ,0.2973  ,-0.2661);
	Eigen::Matrix4f Quater2;
	Quater2.block<3,3>(0,0) = Q_2.toRotationMatrix();
	Quater2(0,3) = -2.3078;
	Quater2(1,3) = -2.1795 ;
	Quater2(2,3) = 0.5873 ;
	// DODAC KINECT INITAL DO CALOSCI !
	Quater2 = KinectInitial.inverse()*Quater2;
	
	float Obrot[3];
	Oblicz_Katy(KinectInitial, Obrot);
	//cout<<"Obroty REAL: "<<Obrot[0]<< " " << Obrot[1]<< " "<<Obrot[2]<<endl;
	Oblicz_Katy(Quater2, Obrot);
	//cout<<"Trans REAL: " << Quater2(0,3)<< " " << Quater2(1,3) << " " << Quater2(2,3)<<endl;
	//cout<<"Obroty REAL: "<<Obrot[0]<< " " << Obrot[1]<< " "<<Obrot[2]<<endl;		
	
	ofstream zzz;
	zzz.open("result.info");
	// TYMCZASOWO
	ifstream ooo;
	ooo.open("matched2");
	cv::namedWindow( "LK Demo", 1 );
	
	char RGBname[30], Dname[30];
	string ind;
	int j=0,zz=0;
	int STEP_SIZE = 15;

	//KinectInitial = Eigen::Matrix4f::Identity();
	for (int i=2;i<600;i++)
	{
		ooo>>ind;
		
		if(i== 2)
		{
			sprintf(RGBname,"%d.png",i-1);
			sprintf(Dname,"%d.xml",i-1);

			tr->LoadRGBD(RGBname,Dname,0);
			tr->newDetection();
		//	tr->trackShow();
		//	cin>>i;
		}
		sprintf(RGBname,"%d.png",i);
		sprintf(Dname,"%d.xml",i);
		tr->LoadRGBD(RGBname,Dname,2);
		cout<<"Loaded : " << i<<"\t";
		
		tr->doTracking();
		
		//tr->trackShow();
		
		cout<<"Checking stop condition"<<endl;
		if(i%STEP_SIZE == j || tr->points[0].size() < 120)
		{
			if ( i%STEP_SIZE == j)
			{
				j = i%STEP_SIZE;
			}
			cout<<"?"<<endl;
			KinectInitial = tr->estimateTransformation(3, Parameters.constrain, KinectInitial);
			Eigen::Quaternion<float> Quat(KinectInitial.block<3,3>(0,0));
			zzz<<ind<<" "<< KinectInitial(0,3)<<" "<<KinectInitial(1,3)<<" "<<KinectInitial(2,3)<<" "<<Quat.coeffs().x()<< " "<<Quat.coeffs().y()<<" "<<Quat.coeffs().z()<< " "<<Quat.coeffs().w()<<endl;
	
			cout<<"Loading new data"<<endl;
			tr->LoadRGBD(RGBname,Dname,0);
			tr->newDetection();
			/*Eigen::Quaternion<float> Q_2(0.998123,0.007768 ,0.059496   ,0.012236 );
			TR.block<3,3>(0,0) = Q_2.toRotationMatrix();
			TR(0,3) = 0.013758;  
			TR(1,3) = 0.031461;
			TR(2,3) = -0.001568;*/
		}
		/*if(i%15==0)
		{
			cout<<"iteracja : " <<zz<<endl;
			RGBD->LoadRGBD2(Dname,RGBname,1);
			(RGBD)->RemovingNaNs(1);
			chmura_color[1] = (RGBD)->BuildPointCloudFromRGBD(1);
			FiltrowanieColor(1);
				
			TransformSelf(chmura_color[1], KinectInitial);
			//ShowCloudXYZRGBA(chmura_color[1]);
			
			// Creating sum cloud
			if ( zz == 0 ) 
			{
				copyPointCloud(*chmura_color[1],*chmura_color[0]);
				zz ++;
			}
			else *chmura_color[0] = *chmura_color[0] + *chmura_color[1];
		}*/
	}
	//ShowCloudXYZRGBA(chmura_color[0]);
	zzz.close();	
	
}

void Kinect::MatchingRun()
{		
	// Reading data from files
	(RGBD)->LoadRGBD2("1.xml","1.png",0);
	(RGBD)->LoadRGBD2("14.xml","14.png",1);

	// Calculating clouds
	chmura_color[0] = (RGBD)->BuildPointCloudFromRGBD(0);
	chmura_color[1] = (RGBD)->BuildPointCloudFromRGBD(1);
		
	copyPointCloud(*chmura_color[0],*chmura[0]);
	copyPointCloud(*chmura_color[1],*chmura[1]);
			
	// Filtrowanie 
	Filtrowanie(0);
	Filtrowanie(1);
		
	// Wyliczenie informacji wg SURF2D i nowy feature matching
	WyliczSURF2D();
	FeatureMatchingWithKnownCorrespondeces();	
}
