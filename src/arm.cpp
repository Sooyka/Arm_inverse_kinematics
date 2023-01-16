#include "arm.h"

void Joint::set_coordinates(const Coordinates &coordinates)
{
    coordinates_ = coordinates;
};
const Coordinates &Joint::get_coordinates() const
{
    return coordinates_;
};

void Joint::set_type(const JointType &joint_type)
{
    type_ = joint_type;
}

const JointType &Joint::get_type() const
{
    return type_;
}

void Segment::set_coordinates(const Coordinates &coordinates)
{
    coordinates_ = coordinates;
};
const Coordinates &Segment::get_coordinates() const
{
    return coordinates_;
};

void Arm::set_joints(const std::vector<Joint> &joints)
{
    joints_ = joints;
};
const std::vector<Joint> &Arm::get_joints() const
{
    return joints_;
};

void Arm::set_segments(const std::vector<Segment> &segments)
{
    segments_ = segments;
};
const std::vector<Segment> &Arm::get_segments() const
{
    return segments_;
};

Matrix4f Arm::effector_frame() const
{
    Matrix4f effector_frame = Matrix4f::Identity();
    std::vector<Joint> joints = get_joints();
    std::vector<Segment> segments = get_segments();
    Joint joint;
    Segment segment;
    int no_of_joints = joints.size();
    for (int i = no_of_joints - 1; i >= 0; i--)
    {
        joint = joints[i];
        segment = segments[i];

        effector_frame = exponential_coordinates_to_SE3(segment.get_coordinates()) * effector_frame;
        effector_frame = exponential_coordinates_to_SE3(joint.get_coordinates()) * effector_frame;
    }
    return effector_frame;
};

Matrix4f Arm::get_nth_segment_frame(int n) const
{
    Matrix4f segment_frame = Matrix4f::Identity();
    std::vector<Joint> joints = get_joints();
    std::vector<Segment> segments = get_segments();
    Joint joint;
    Segment segment;

    if (segments.size() < n)
    {
        return effector_frame();
    }
    joint = joints[n - 1];
    segment_frame = segment_frame * exponential_coordinates_to_SE3(joint.get_coordinates());
    for (int i = n - 2; i >= 0; i--)
    {
        joint = joints[i];
        segment = segments[i];
        segment_frame = exponential_coordinates_to_SE3(segment.get_coordinates()) * segment_frame;
        segment_frame = exponential_coordinates_to_SE3(joint.get_coordinates()) * segment_frame;
    }

    return segment_frame;
};

Matrix4f Arm::get_nth_joint_frame(int n) const
{
    Matrix4f joint_frame = Matrix4f::Identity();
    std::vector<Joint> joints = get_joints();
    std::vector<Segment> segments = get_segments();
    Joint joint;
    Segment segment;
    if (joints.size() < n)
    {
        return effector_frame();
    }
    for (int i = n - 2; i >= 0; i--)
    {
        joint = joints[i];
        segment = segments[i];

        joint_frame = exponential_coordinates_to_SE3(segment.get_coordinates()) * joint_frame;
        joint_frame = exponential_coordinates_to_SE3(joint.get_coordinates()) * joint_frame;
    }
    return joint_frame;
};

void Arm::inverse_kinematics(Coordinates coordinates){

};