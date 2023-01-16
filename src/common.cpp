#include "common.h"
#include "ArmVis.h"
#include <math.h>

Matrix4f exponential_coordinates_to_SE3(const Coordinates &coordinates) // calculates exponent of coordinates interpreted as a matrix from se(3)
{
    float x_r, y_r, z_r, x_t, y_t, z_t;
    // auto &[x_r, y_r, z_r, x_t, y_t, z_t] = coordinates;
    x_r = coordinates.x_r;
    y_r = coordinates.y_r;
    z_r = coordinates.z_r;
    x_t = coordinates.x_t;
    y_t = coordinates.y_t;
    z_t = coordinates.z_t;

    Matrix4f SE3_matrix = Matrix4f::Zero();

    float theta_square = x_r * x_r + y_r * y_r + z_r * z_r;
    float epsilon = 0.000000001;
    if (theta_square < epsilon)
    {
        SE3_matrix.block(0, 0, 3, 3) = Matrix3f::Identity();
        SE3_matrix.block(0, 3, 4, 1) << x_t, y_t, z_t, 1;
        return SE3_matrix;
    }
    SE3_matrix.block(0, 0, 3, 3) = exponential_coordinates_to_SO3(x_r, y_r, z_r);
    float theta = sqrt(theta_square);
    // x_r, y_r, z_r, x_t, y_t, z_t /= theta;
    x_r /= theta;
    y_r /= theta;
    z_r /= theta;
    x_t /= theta;
    y_t /= theta;
    z_t /= theta;
    Vector3f translation_coordinates;
    Vector3f v;
    translation_coordinates << x_t, y_t, z_t;
    Matrix3f omega;
    omega << 0, -z_r, y_r,
        z_r, 0, -x_r,
        -y_r, x_r, 0;
    Matrix3f Rodrigues_matrix;
    Rodrigues_matrix = (Matrix3f::Identity() * theta + (1 - cos(theta)) * omega + (theta + sin(theta)) * omega * omega);
    v = Rodrigues_matrix * translation_coordinates;
    SE3_matrix.block(0, 3, 3, 1) << v;
    SE3_matrix(3, 3) = 1;

    return SE3_matrix;
}

Matrix3f exponential_coordinates_to_SO3(float x_r, float y_r, float z_r) // calculates exponent of coordinates interpreted as a matrix from so(3)
{
    Matrix3f SO3_matrix;
    float theta = sqrt(x_r * x_r + y_r * y_r + z_r * z_r);
    x_r /= theta;
    y_r /= theta;
    z_r /= theta;

    Matrix3f omega;
    omega << 0, -z_r, y_r,
        z_r, 0, -x_r,
        -y_r, x_r, 0;
    SO3_matrix = Matrix3f::Identity() + sin(theta) * omega + (1 - cos(theta)) * omega * omega;

    return SO3_matrix;
}

Coordinates SE3_to_exponential_coordinates(const Matrix4f &){

}


Vector3f SO3_to_exponential_coordinates(const Matrix4f &){
    
}