#ifndef __ARM_H__
#define __ARM_H__

#include "inverse_kinematics.h"

enum JointType
{
    Revolut,
    Spherical,
    Prismatic,
};

class Joint
{
    Coordinates coordinates_; // element of se(3) describing how the joint is transforming a reference frame written as a twist vector
    JointType type_;

public:
    void set_coordinates(const Coordinates &coordinates);
    const Coordinates &get_coordinates() const;
    void set_type(const JointType &joint_type);
    const JointType &get_type() const;
};

class Segment
{
    Coordinates coordinates_; // element of se(3) describing how the segment is transforming a reference frame writen as a twist vector

public:
    void set_coordinates(const Coordinates &coordinates);
    const Coordinates &get_coordinates() const;
};

class Arm
{
    std::vector<Joint> joints_;     // First joint (with index 0 in the vector) is the one closest to the ground
    std::vector<Segment> segments_; // First segment (with index 0 in the vector) is the one closest to the ground

public:
    Matrix4f effector_frame() const; // for a given Arm configuration gives element of SE(3) describing effectors
                                     //  reference frame position and orientation
    Matrix4f get_nth_segment_frame(int n) const;
    Matrix4f get_nth_joint_frame(int n) const;
    void set_joints(const std::vector<Joint> &joints);
    const std::vector<Joint> &get_joints() const;
    void set_segments(const std::vector<Segment> &segments);
    const std::vector<Segment> &get_segments() const;

    void inverse_kinematics(Coordinates coordinates);
};

#endif