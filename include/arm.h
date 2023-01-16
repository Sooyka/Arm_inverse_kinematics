#ifndef __ARM_H__
#define __ARM_H__

#include "common.h"

enum JointType
{
    Revolut_pitch,
    Revolut_yawn,
    Revolut_roll,
    Spherical,
    Prismatic,
};

class Joint
{
    Matrix4f coordinates_; // element of SE(3) describing how the joint is transforming a reference frame
    JointType type;

public:
    void set_joint_coordinates(const Matrix4f &coordinates);
    const Matrix4f &get_joint_coordinates() const;
    void set_joint_type(const JointType &joint_type);
    const Matrix4f &get_joint_type() const;
};

class Segment
{
    Matrix4f coordinates_; // element of SE(3) describing how the segment is transforming a reference frame

public:
    void set_segment_coordinates(const Matrix4f &coordinates);
    const Matrix4f &get_segment_coordinates() const;
};

class Arm
{
    std::vector<Joint> joints_;    //First joint (with index 0 in the vector) is the one closest to the ground
    std::vector<Segment> segments_; //First segment (with index 0 in the vector) is the one closest to the ground

public:
    Matrix4f effector_coordinates() const; // for a given Arm configuration gives element of SE(3) describing effectors
                                                  //  reference frame position and orientation
    void set_arm_joints(const std::vector<Joint> &joints);
    const std::vector<Joint>& get_arm_joints() const;
    void set_arm_segments(const std::vector<Segment> &segments);
    const std::vector<Segment>& get_arm_segments() const;
    Matrix4f get_nth_segment_coordinates(int n) const;
    Matrix4f get_nth_joint_coordinates(int n) const;
};

#endif