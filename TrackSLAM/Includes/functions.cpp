#include "includes.h"


void Oblicz_Katy(Eigen::Matrix4f &transformation, float *Orient)
{
	//obrót wokól osi x
     //Orient[0]=(atan2(transformation(2,1),transformation(2,2)))*180/PI;
	//obrót wokół osi y
     //Orient[1]=(atan2(-transformation(2,0),sqrt(transformation(2,1))+pow(transformation(2,2),2)))*180/PI;
	//obrót wokół osi z
     //Orient[2]=(atan2(transformation(1,0),transformation(0,0)))*180/PI;
     
	if (transformation(0,0) == 1.0f)
	{
		Orient[0] = atan2(transformation(0,2), transformation(2,3))*180/PI;
		Orient[1] = 0;
		Orient[2] = 0;
	}
	else if (transformation(0,0) == -1.0f)
	{
		Orient[0] = atan2(transformation(0,2), transformation(2,3))*180/PI;
		Orient[1] = 0;
		Orient[2] = 0;
	}
	else 
	{
		Orient[0] = atan2(-transformation(2,0), transformation(0,0))*180/PI;
		Orient[1] = asin(transformation(1,0))*180/PI;
		Orient[2] = atan2(-transformation(1,2),transformation(1,1))*180/PI;
	}
}

void setViewerPose (pcl::visualization::PCLVisualizer& viewer)
{
	Eigen::Vector3f pos_vector =  Eigen::Vector3f (0, 0, 0);
	Eigen::Vector3f look_at_vector = Eigen::Vector3f (0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector =  Eigen::Vector3f (0, -1, 0);

	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],look_at_vector[0],look_at_vector[1],look_at_vector[2],up_vector[0],up_vector[1],up_vector[2]);
	/*viewer.camera_.pos[0] = pos_vector[0];
	viewer.camera_.pos[1] = pos_vector[1];
	viewer.camera_.pos[2] = pos_vector[2];
	viewer.camera_.focal[0] = look_at_vector[0];
	viewer.camera_.focal[1] = look_at_vector[1];
	viewer.camera_.focal[2] = look_at_vector[2];
	viewer.camera_.view[0] = up_vector[0];
	viewer.camera_.view[1] = up_vector[1];
	viewer.camera_.view[2] = up_vector[2];*/
	viewer.updateCamera ();
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Transform(pcl::PointCloud<pcl::PointXYZ>::Ptr chmura, Eigen::Matrix4f &transformation)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tym(new pcl::PointCloud<pcl::PointXYZ>);
	tym->width    = chmura->width;
	tym->height   = chmura->height;
	tym->is_dense = false;
	tym->points.resize (chmura->width * chmura->height);
	
	// Przekształcenie chmury 1 przez zadany obrót i translacje
	for(int j=0;j<chmura->points.size();j++)
	{
		// Przepisanie punktu chmury i przesunięcie punktu do miejsca, dla którego liczony był 1 obrót
		Eigen::VectorXf punkt(4);
		punkt[0] = chmura->points[j].x;
		punkt[1] = chmura->points[j].y;
		punkt[2] = chmura->points[j].z;
		punkt[3] = 1;
				
		// Zastosowanie optymalnej transformacji
		punkt = transformation * punkt;
				
		// Przepisanie nowej wartości punktu do chmury
		tym->points[j].x = punkt[0];
		tym->points[j].y = punkt[1];
		tym->points[j].z = punkt[2];
	}
	return tym;
}
void TransformSelf(pcl::PointCloud<pcl::PointXYZ>::Ptr chmura, Eigen::Matrix4f &transformation)
{	
	// Przekształcenie chmury 1 przez zadany obrót i translacje
	for(int j=0;j<chmura->points.size();j++)
	{
		// Przepisanie punktu chmury i przesunięcie punktu do miejsca, dla którego liczony był 1 obrót
		Eigen::VectorXf punkt(4);
		punkt[0] = chmura->points[j].x;
		punkt[1] = chmura->points[j].y;
		punkt[2] = chmura->points[j].z;
		punkt[3] = 1;
				
		// Zastosowanie optymalnej transformacji
		punkt = transformation * punkt;
				
		// Przepisanie nowej wartości punktu do chmury
		chmura->points[j].x = punkt[0];
		chmura->points[j].y = punkt[1];
		chmura->points[j].z = punkt[2];
	}
}
void TransformSelf(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr chmura, Eigen::Matrix4f &transformation)
{	
	// Przekształcenie chmury 1 przez zadany obrót i translacje
	for(int j=0;j<chmura->points.size();j++)
	{
		// Przepisanie punktu chmury i przesunięcie punktu do miejsca, dla którego liczony był 1 obrót
		Eigen::VectorXf punkt(4);
		punkt[0] = chmura->points[j].x;
		punkt[1] = chmura->points[j].y;
		punkt[2] = chmura->points[j].z;
		punkt[3] = 1;
				
		// Zastosowanie optymalnej transformacji
		punkt = transformation * punkt;
				
		// Przepisanie nowej wartości punktu do chmury
		chmura->points[j].x = punkt[0];
		chmura->points[j].y = punkt[1];
		chmura->points[j].z = punkt[2];
	}
}
double SquaredNorm(pcl::PointXYZ a ,pcl::PointXYZ b)
{
	return ( (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)+ (a.z - b.z) * (a.z - b.z));
}

bool DiffPoints(pcl::PointXYZ a, pcl::PointXYZ b)
{
	double eps = numeric_limits<double>::epsilon();
	return  ( SquaredNorm(a,b) > eps );
}
void calculateTransformation(Eigen::Matrix4f &transformation,float alpha, float beta, float gamma, float tx, float ty, float tz)
{

    float s1=sin(alpha);
    float s2=sin(beta);
    float s3=sin(gamma);
    float c1=cos(alpha);
    float c2=cos(beta);
    float c3=cos(gamma);

    float r11=c2; float r12=-c3*s2; float r13=s2*s3;
    float r21=c1*s2; float r22=c1*c2*c3-s1*s3; float r23=-c3*s1-c1*c2*s3;
    float r31=s1*s2; float r32=c1*s3+c2*c3*s1; float r33=c1*c3-c2*s1*s3;
    
    r11 =  1;
    r12 = r13 = 0;
    r21 = 0;
    r22 = c1;
    r23 = -s1;
    r31 = 0;
    r32 = s1;
    r33 = c1;

	transformation = Eigen::Matrix4f::Identity();

    transformation(0,0)=r11;
    transformation(0,1)=r12;
    transformation(0,2)=r13;

    transformation(1,0)=r21;
    transformation(1,1)=r22;
    transformation(1,2)=r23;

    transformation(2,0)=r31;
    transformation(2,1)=r32;
    transformation(2,2)=r33;

    transformation(0,3)=tx;
    transformation(1,3)=ty;
    transformation(2,3)=tz;
}

Eigen::Matrix3f Quaternion2Matrix(Eigen::Quaternion<float> Q)
{
	Q.toRotationMatrix();
}

bool keypointscompare (pair< cv::KeyPoint, int >   i,pair< cv::KeyPoint, int >  j) 
{ 
	if(i.second != j.second) return i.second > j.second;
	return (i.first.response > j.first.response); 
}
bool keyRESPONSEcompare (cv::KeyPoint i, cv::KeyPoint j) 
{ 
	return i.response > j.response; 
};

string convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
