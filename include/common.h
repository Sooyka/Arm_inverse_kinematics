#ifndef __COMMON_LIBS_H__
#define __COMMON_LIBS_H__

#include <iostream>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <vector>
// #define _GNU_SOURCE 
#include <fenv.h>
#include "spdlog/spdlog.h"
#include<unistd.h>  
// int feenableexcept(FE_INVALID | FE_OVERFLOW);
// int feenableexcept(FE_ALL_EXCEPT & ~FE_INEXACT);
// #include <boost/range/combine.hpp>
// #include "boost/tuple/tuple_io.hpp"
// #include <tuple>

using namespace Eigen;

struct Coordinates
{
    double x_r;
    double y_r;
    double z_r;
    double x_t;
    double y_t;
    double z_t;
};

Matrix4d exponential_coordinates_to_SE3(const Coordinates &);

Matrix3d exponential_coordinates_to_SO3(double x_r, double y_r, double z_r);

Coordinates SE3_to_exponential_coordinates(const Matrix4d &);

Vector3d SO3_to_exponential_coordinates(const Matrix3d &);

struct Sn_theta
{
    Matrix4d Sn;
    double theta;
};

struct Bn_theta
{
    Matrix4d Bn;
    double theta;
};

Matrix4d Sn_to_Bn(const Matrix4d& Sn, const Matrix4d& M);

Coordinates se3_to_exponential_coordinates(const Matrix4d& m);

#endif