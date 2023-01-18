#include "common.h"
#include "ArmVis.h"

#define EPSILON 0.001

Matrix4f exponential_coordinates_to_SE3(const Coordinates &coordinates) // calculates exponent of coordinates interpreted as a matrix from se(3)
{
    float x_r, y_r, z_r, x_t, y_t, z_t;
    x_r = coordinates.x_r;
    y_r = coordinates.y_r;
    z_r = coordinates.z_r;
    x_t = coordinates.x_t;
    y_t = coordinates.y_t;
    z_t = coordinates.z_t;

    Matrix4f SE3_matrix = Matrix4f::Zero();

    float theta_square = x_r * x_r + y_r * y_r + z_r * z_r;
    float epsilon = 0.001;
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
    Rodrigues_matrix = (Matrix3f::Identity() * theta + (1 - cos(theta)) * omega + (theta - sin(theta)) * omega * omega);
    v = Rodrigues_matrix * translation_coordinates;
    SE3_matrix.block(0, 3, 3, 1) << v;
    SE3_matrix(3, 3) = 1;

    return SE3_matrix;
}

Matrix3f exponential_coordinates_to_SO3(float x_r, float y_r, float z_r) // calculates exponent of coordinates interpreted as a matrix from so(3)
{
    Matrix3f SO3_matrix;
    float epsilon = 0.001;
    float theta_square = x_r * x_r + y_r * y_r + z_r * z_r;
    if (theta_square < epsilon)
    {

        return Matrix3f::Identity();
    }
    const float theta = sqrt(theta_square);
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

Coordinates SE3_to_exponential_coordinates(const Matrix4f &matrix)
{
    Matrix3f R = matrix.block(0, 0, 3, 3);
    Vector3f p = matrix.block(0, 3, 3, 1);
    Coordinates coordinates;
    float epsilon = 0.001;
    Matrix3f diff = R - Matrix3f::Identity();
    Matrix3f omega;
    float theta;
    Vector3f v;

    if (diff.norm() < epsilon)
    {

        v = p;
        coordinates.x_r = 0;
        coordinates.y_r = 0;
        coordinates.z_r = 0;
        coordinates.x_t = v(0);
        coordinates.y_t = v(1);
        coordinates.z_t = v(2);
        return coordinates;
    }

    Vector3f rotation_coordinates = SO3_to_exponential_coordinates(R);
    float x_r, y_r, z_r;
    x_r = rotation_coordinates(0);
    y_r = rotation_coordinates(1);
    z_r = rotation_coordinates(2);

    coordinates.x_r = x_r;
    coordinates.y_r = y_r;
    coordinates.z_r = z_r;

    theta = sqrt(x_r * x_r + y_r * y_r + z_r * z_r);
    x_r /= theta;
    y_r /= theta;
    z_r /= theta;

    omega << 0, -z_r, y_r,
        z_r, 0, -x_r,
        -y_r, x_r, 0;

    v = (1 / theta * Matrix3f::Identity() - 0.5 * omega + (1 / theta - 0.5 * cos(theta / 2) / sin(theta / 2)) * omega * omega) * p * theta;

    coordinates.x_t = v(0);
    coordinates.y_t = v(1);
    coordinates.z_t = v(2);

    return coordinates;
}

Vector3f SO3_to_exponential_coordinates(const Matrix3f &R)
{
    float x_r, y_r, z_r;
    const float trace = R.trace();
    Matrix3f omega;
    Vector3f coordinates;
    Matrix3f diff = R - Matrix3f::Identity();
    float epsilon = 0.001;

    if (diff.norm() < epsilon)
    {
        coordinates(0) = 0;
        coordinates(1) = 0;
        coordinates(2) = 0;
        return coordinates;
    }

    float trace_diff = trace + 1;
    float r[3];
    Vector3f r_pi;
    if (abs(trace_diff) < epsilon || abs(0.5 * (trace - 1)) > 1)
    {
        int i = 0;
        if (R(1, 1) >= R(i, i))
        {   
            i = 1;
            if (R(2, 2) > R(i, i))
            {
                i = 2;
            }
        }

        r[0] = R(0, i);
        r[1] = R(1, i);
        r[2] = R(2, i);
        r[i]++;
        r_pi << r[0], r[1], r[2];
        coordinates = M_PI * 1 / sqrt(2 * (1 + R(i,i))) * r_pi;
        return coordinates;
    }

    const float theta = acos(0.5 * (trace - 1));
    if (abs(sin(theta)) < epsilon)
    {
        int i = 0;
        if (R(1, 1) >= R(i, i))
        {
            i = 1;
            if (R(2, 2) > R(i, i))
            {
                i = 2;
            }
        }

        r[0] = R(0, i);
        r[1] = R(1, i);
        r[2] = R(2, i);
        r[i]++;
        r_pi << r[0], r[1], r[2];
        coordinates = M_PI * 1 / sqrt(2 * (1 + R(i,i))) * r_pi;
        return coordinates;
    }
    omega = 1 / (2 * sin(theta)) * (R - R.transpose());

    coordinates(0) = omega(2, 1);
    coordinates(1) = omega(0, 2);
    coordinates(2) = omega(1, 0);

    coordinates *= theta;

    return coordinates;
}

Matrix4f Sn_to_Bn(const Matrix4f &Sn, const Matrix4f &M)
{
    return M.inverse() * Sn * M;
}

Coordinates se3_to_exponential_coordinates(const Matrix4f &m)
{

    Coordinates coordinates;

    coordinates.x_r = m(2, 1);
    coordinates.y_r = m(0, 2);
    coordinates.z_r = m(1, 0);
    coordinates.x_t = m(0, 3);
    coordinates.y_t = m(1, 3);
    coordinates.z_t = m(2, 3);

    return coordinates;
}