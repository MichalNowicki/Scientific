#ifndef _MAT
#define _MAT

#include <Eigen/Eigen>
#include <math.h>

template <typename T> int sgn(T val)
{
    return (val > T(0)) - (val < T(0));
}

// Wyliczenie optymalnej macierzy transformacji
void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &OPT);

// Wyliczenie optymalnej macierzy transformacji dla podanej liczby par
void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &OPT, int ile);

#endif
