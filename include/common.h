#ifndef __COMMON_LIBS_H__
#define __COMMON_LIBS_H__

#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>
// #include <boost/range/combine.hpp>
// #include "boost/tuple/tuple_io.hpp"
// #include <tuple>

using namespace Eigen;

struct coordinates
{
    float x_r;
    float y_r;
    float z_r;
    float x_t;
    float y_t;
    float z_t;
};

Matrix4f exponential_coordinates_to_se3(const coordinates&);

#endif