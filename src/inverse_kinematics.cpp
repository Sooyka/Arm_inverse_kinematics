#include "inverse_kinematics.h"

// using namespace Eigen;

// void Hello::print()
// {
//     std::cout << "Hello Headers!" << std::endl;
// }

void Joint::set_joint_coordinates(const Matrix4f &coordinates)
{
    coordinates_ = coordinates;
};
const Matrix4f &Joint::get_joint_coordinates() const
{
    return coordinates_;
};

void Segment::set_segment_coordinates(const Matrix4f &coordinates)
{
    coordinates_ = coordinates;
};
const Matrix4f &Segment::get_segment_coordinates() const
{
    return coordinates_;
};

void Arm::set_arm_joints(const std::vector<Joint> &joints)
{
    joints_ = joints;
};
const std::vector<Joint> &Arm::get_arm_joints() const
{
    // return std::tuple<std::vector<Joint> &, std::vector<float> &>{&joints_, &segments_};
    return joints_;
};

void Arm::set_arm_segments(const std::vector<Segment> &segments)
{
    segments_ = segments;
};
const std::vector<Segment> &Arm::get_arm_segments() const
{
    // return std::tuple<std::vector<Joint> &, std::vector<float> &>{&joints_, &segments_};
    return segments_;
};

Matrix4f Arm::effector_position_and_orientation() const
{
    Matrix4f effector_position = Matrix4f::Identity();
    Joint joint;
    Segment segment;
    for (const auto joint_and_segment : boost::combine(joints_, segments_))
    {
        joint = boost::get<0>(joint_and_segment);
        segment = boost::get<1>(joint_and_segment);

        effector_position = effector_position * segment.get_segment_coordinates();
        effector_position = effector_position * joint.get_joint_coordinates();
    }
    return effector_position;
};

Matrix4f Arm::get_nth_segment_coordinates(int n) const 
{
    Matrix4f a = Matrix4f::Identity();
    return a;
};

Matrix4f Arm::get_nth_joint_coordinates(int n) const 
{
    Matrix4f a = Matrix4f::Identity();
    return a;
};