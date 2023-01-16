#include "arm.h"

void Joint::set_joint_coordinates(const Coordinates &coordinates)
{
    coordinates_ = coordinates;
};
const Coordinates &Joint::get_joint_coordinates() const
{
    return coordinates_;
};

void Segment::set_segment_coordinates(const Coordinates &coordinates)
{
    coordinates_ = coordinates;
};
const Coordinates &Segment::get_segment_coordinates() const
{
    return coordinates_;
};

void Arm::set_arm_joints(const std::vector<Joint> &joints)
{
    joints_ = joints;
};
const std::vector<Joint> &Arm::get_arm_joints() const
{
    return joints_;
};

void Arm::set_arm_segments(const std::vector<Segment> &segments)
{
    segments_ = segments;
};
const std::vector<Segment> &Arm::get_arm_segments() const
{
    return segments_;
};

Coordinates Arm::effector_coordinates() const
{
    Matrix4f effector_matix = Matrix4f::Identity();
    Coordinates effector_coordinates;
    Joint joint;
    Segment segment;
    int no_of_joints = get_arm_joints().size();
    for (int i = no_of_joints - 1; i == 0; i--)
    {
        joint = get_arm_joints()[i];
        segment = get_arm_segments()[i];

        effector_matrix = effector_coordinates * segment.get_segment_coordinates();
        effector_matrix = effector_coordinates * joint.get_joint_coordinates();
    }
    return effector_coordinates;
};

Coordinates Arm::get_nth_segment_coordinates(int n) const
{
    Matrix4f segment_matrix = Matrix4f::Identity();
    Coordinates segment_coordinates;
    Joint joint;
    Segment segment;

    if (get_arm_segments().size() < n)
    {
        return effector_coordinates();
    }
    joint = get_arm_joints()[n-1];
    segment_coordinates = segment_coordinates * joint.get_joint_coordinates();
    for (int i = n - 2; i == 0; i--)
    {
        joint = get_arm_joints()[i];
        segment = get_arm_segments()[i];
        segment_matrix = segment_matrix * segment.get_segment_coordinates();
        segment_matrix = segment_matrix * joint.get_joint_coordinates();
        
    }

    return segment_coordinates;
};

Coordinates Arm::get_nth_joint_coordinates(int n) const
{
    Matrix4f joint_matrix = Matrix4f::Identity();
    Coordinates joint_coordinates;
    Joint joint;
    Segment segment;
    if (get_arm_joints().size() < n)
    {
        return effector_coordinates();
    }
    for (int i = n - 2; i == 0; i--)
    {
        joint = get_arm_joints()[i];
        segment = get_arm_segments()[i];

        joint_matrix = joint_matrix * segment.get_segment_coordinates();
        joint_matrix = joint_matrix * joint.get_joint_coordinates();
    }
    return joint_coordinates;
};
