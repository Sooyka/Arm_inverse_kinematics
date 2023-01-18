#include "common.h"
#include "ArmVis.h"

#define EPSILON 0.00001

Matrix4d exponential_coordinates_to_SE3(const Coordinates &coordinates) // calculates exponent of coordinates interpreted as a matrix from se(3)
{
    double x_r, y_r, z_r, x_t, y_t, z_t;
    x_r = coordinates.x_r;
    y_r = coordinates.y_r;
    z_r = coordinates.z_r;
    x_t = coordinates.x_t;
    y_t = coordinates.y_t;
    z_t = coordinates.z_t;

    Matrix4d SE3_matrix = Matrix4d::Zero();

    double theta_square = x_r * x_r + y_r * y_r + z_r * z_r;
    double epsilon = 0.00001;
    if (theta_square < epsilon)
    {
        SE3_matrix.block(0, 0, 3, 3) = Matrix3d::Identity();
        SE3_matrix.block(0, 3, 4, 1) << x_t, y_t, z_t, 1;
        return SE3_matrix;
    }
    SE3_matrix.block(0, 0, 3, 3) = exponential_coordinates_to_SO3(x_r, y_r, z_r);
    double theta = sqrt(theta_square);
    // x_r, y_r, z_r, x_t, y_t, z_t /= theta;
    x_r /= theta;
    y_r /= theta;
    z_r /= theta;
    x_t /= theta;
    y_t /= theta;
    z_t /= theta;
    Vector3d translation_coordinates;
    Vector3d v;
    translation_coordinates << x_t, y_t, z_t;
    Matrix3d omega;
    omega << 0, -z_r, y_r,
        z_r, 0, -x_r,
        -y_r, x_r, 0;
    Matrix3d Rodrigues_matrix;
    Rodrigues_matrix = (Matrix3d::Identity() * theta + (1 - cos(theta)) * omega + (theta - sin(theta)) * omega * omega);
    v = Rodrigues_matrix * translation_coordinates;
    SE3_matrix.block(0, 3, 3, 1) << v;
    SE3_matrix(3, 3) = 1;

    return SE3_matrix;
}

Matrix3d exponential_coordinates_to_SO3(double x_r, double y_r, double z_r) // calculates exponent of coordinates interpreted as a matrix from so(3)
{
    Matrix3d SO3_matrix;
    double epsilon = 0.00001;
    double theta_square = x_r * x_r + y_r * y_r + z_r * z_r;
    if (theta_square < epsilon)
    {

        return Matrix3d::Identity();
    }
    const double theta = sqrt(theta_square);
    x_r /= theta;
    y_r /= theta;
    z_r /= theta;

    Matrix3d omega;
    omega << 0, -z_r, y_r,
        z_r, 0, -x_r,
        -y_r, x_r, 0;
    SO3_matrix = Matrix3d::Identity() + sin(theta) * omega + (1 - cos(theta)) * omega * omega;

    return SO3_matrix;
}

Coordinates SE3_to_exponential_coordinates(const Matrix4d &matrix)
{
    Matrix3d R = matrix.block(0, 0, 3, 3);
    Vector3d p = matrix.block(0, 3, 3, 1);
    Coordinates coordinates;
    double epsilon = 0.00001;
    Matrix3d diff = R - Matrix3d::Identity();
    Matrix3d omega;
    double theta;
    Vector3d v;

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

    Vector3d rotation_coordinates = SO3_to_exponential_coordinates(R);
    double x_r, y_r, z_r;
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

    v = (1 / theta * Matrix3d::Identity() - 0.5 * omega + (1 / theta - 0.5 * cos(theta / 2) / sin(theta / 2)) * omega * omega) * p * theta;

    coordinates.x_t = v(0);
    coordinates.y_t = v(1);
    coordinates.z_t = v(2);

    return coordinates;
}

Vector3d SO3_to_exponential_coordinates(const Matrix3d &R)
{
    double x_r, y_r, z_r;
    const double trace = R.trace();
    Matrix3d omega;
    Vector3d coordinates;
    Matrix3d diff = R - Matrix3d::Identity();
    double epsilon = 0.00001;

    if (diff.norm() < epsilon)
    {
        coordinates(0) = 0;
        coordinates(1) = 0;
        coordinates(2) = 0;
        return coordinates;
    }

    double trace_diff = trace + 1;
    double r[3];
    Vector3d r_pi;
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

    const double theta = acos(0.5 * (trace - 1));
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

Matrix4d Sn_to_Bn(const Matrix4d &Sn, const Matrix4d &M)
{
    return M.inverse() * Sn * M;
}

Coordinates se3_to_exponential_coordinates(const Matrix4d &m)
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