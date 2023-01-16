#include "common.h"
#include "ArmVis.h"

Matrix4f exponential_coordinates_to_se3(const Coordinates & coordinates)
{
    float x_r, y_r, z_r, x_t, y_t, z_t;
    // auto &[x_r, y_r, z_r, x_t, y_t, z_t] = coordinates;
    x_r = coordinates.x_r;
    y_r = coordinates.y_r;
    z_r = coordinates.z_r;
    x_t = coordinates.x_t;
    y_t = coordinates.y_t;
    z_t = coordinates.z_t;
    Matrix4f lie_alg_matrix;
    lie_alg_matrix <<   0,  -z_r,  y_r,  x_t,
                        z_r,  0,  -x_r,  y_t,
                        -y_r,  x_r,  0,  z_t,
                        0,  0,  0,  0;
    return lie_alg_matrix;
}