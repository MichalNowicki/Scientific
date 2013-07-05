#include "Matching.h"

void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &OPT)
{
	Kabsch(P,Q,OPT,3);
}


void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &OPT, int ile)
{
	float MEAN[6];
	// Przesuniecie do srodka
	for(int j=0;j<6;j++) MEAN[j] = 0.0f;
			
	for(int j=0;j<ile;j++)
	{
		for(int h=0;h<3;h++) MEAN[h] += P(j,h);
		for(int h=0;h<3;h++) MEAN[h+3] += Q(j,h);
	}
	for(int j=0;j<6;j++) MEAN[j] /= ile;
		
	// Po policzeniu arytmetycznej musimy odjac
	for(int j=0;j<ile;j++)
	{
		for(int h=0;h<3;h++) P(j,h) -= MEAN[h];
		for(int h=0;h<3;h++) Q(j,h) -= MEAN[h+3];
	} 
			
	// Policzenie transpose P * Q
	Eigen::MatrixXf A = P.transpose() * Q;
			
	// SVD	
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf V = svd.matrixU(), W = svd.matrixV();

	// Czy prawoskretny
	Eigen::Matrix3f U = Eigen::MatrixXf::Identity(3,3);
	U(2,2) = sgn(A.determinant());
			
	// Macierz rotacji to:
	U = W * U * V.transpose();
			
	// Przesunięcie
	Eigen::Vector3f T;		
	T[0] = - MEAN[0];
	T[1] = - MEAN[1];
	T[2] = - MEAN[2]; 
	T = U*T;
	T[0] += MEAN[3];
	T[1] += MEAN[4];
	T[2] += MEAN[5]; 
	
	OPT <<  Eigen::Matrix4f::Identity();
	OPT.block<3,3>(0,0) = U.block<3,3>(0,0);
	OPT.block<3,1>(0,3) = T.head<3>();

}

