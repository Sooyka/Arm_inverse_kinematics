#include "vis_interface.h"

std::vector<float> arm_to_float_vector(const Arm &arm)
{
    double scaling_parameter = 2;
    Matrix4d vis_scaling_matrix = Matrix4d::Identity();
    vis_scaling_matrix(1, 1) = scaling_parameter;
    vis_scaling_matrix(3, 3) = scaling_parameter;
    std::vector<float> arm_raw_coordinates;
    Matrix4d joint_segment_matrix = Matrix4d::Identity();
    std::vector<double> raw_matrix_double;
    std::vector<Joint> joints = arm.get_joints();
    std::vector<Segment> segments = arm.get_segments();
    Joint joint;
    Segment segment;
    int no_of_joints = joints.size();
    if (no_of_joints > 0)
    {
        joint = joints[0];
        segment = segments[0];
        joint_segment_matrix = joint_segment_matrix * exponential_coordinates_to_SE3(joint.get_coordinates());
        raw_matrix_double = matrix_to_double_vector(joint_segment_matrix * segment_scaling_for_drawing(segment) * vis_scaling_matrix);
        for (int j = 0; j < 16; j++)
        {
            arm_raw_coordinates.push_back(raw_matrix_double[j]);
        }
    }
    for (int i = 1; i <= no_of_joints - 1; i++)
    {
        joint = joints[i];
        segment = segments[i - 1];

        joint_segment_matrix = joint_segment_matrix * exponential_coordinates_to_SE3(segment.get_coordinates());
        joint_segment_matrix = joint_segment_matrix * exponential_coordinates_to_SE3(joint.get_coordinates());
        raw_matrix_double = matrix_to_double_vector(joint_segment_matrix * segment_scaling_for_drawing(segments[i]) * vis_scaling_matrix);
        for (int j = 0; j < 16; j++)
        {
            arm_raw_coordinates.push_back((float)raw_matrix_double[j]);
        }
    }
    raw_matrix_double = matrix_to_double_vector(arm.effector_frame());
    for (int j = 0; j < 16; j++)
    {
        arm_raw_coordinates.push_back((float)raw_matrix_double[j]);
    }
    raw_matrix_double = matrix_to_double_vector(arm.Sn_product());
    for (int j = 0; j < 16; j++)
    {
        arm_raw_coordinates.push_back((float)raw_matrix_double[j]);
    }
    raw_matrix_double = matrix_to_double_vector(arm.Bn_product());
    for (int j = 0; j < 16; j++)
    {
        arm_raw_coordinates.push_back((float)raw_matrix_double[j]);
    }
    return arm_raw_coordinates;
}

std::vector<double> matrix_to_double_vector(const Matrix4d &matrix)
{
    std::vector<double> raw_matrix_double;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            raw_matrix_double.push_back(matrix(j, i));
        }
    }
    return raw_matrix_double;
}

Matrix4d segment_scaling_for_drawing(const Segment &segment)
{
    Coordinates coordinates = segment.get_coordinates();
    double x_t = coordinates.x_t;
    double y_t = coordinates.y_t;
    double z_t = coordinates.z_t;

    double scaling = sqrt(x_t * x_t + y_t * y_t + z_t * z_t);

    Matrix4d scaling_matrix = Matrix4d::Identity();

    scaling_matrix(1, 1) = scaling;

    return scaling_matrix;
}