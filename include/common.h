#ifndef __COMMON_LIBS_H__
#define __COMMON_LIBS_H__

#include <iostream>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <vector>
// #include <boost/range/combine.hpp>
// #include "boost/tuple/tuple_io.hpp"
// #include <tuple>

using namespace Eigen;

struct Coordinates
{
    float x_r;
    float y_r;
    float z_r;
    float x_t;
    float y_t;
    float z_t;
};

Matrix4f exponential_coordinates_to_SE3(const Coordinates &);

Matrix3f exponential_coordinates_to_SO3(float x_r, float y_r, float z_r);

Coordinates SE3_to_exponential_coordinates(const Matrix4f &);

Vector3f SO3_to_exponential_coordinates(const Matrix3f &);

struct Sn_theta
{
    Matrix4f Sn;
    float theta;
};

struct Bn_theta
{
    Matrix4f Bn;
    float theta;
};

Matrix4f Sn_to_bn(const Matrix4f& Sn, const Matrix4f& M);

#endif