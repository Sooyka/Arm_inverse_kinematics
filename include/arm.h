#ifndef __ARM_H__
#define __ARM_H__

#include "common.h"

enum JointType
{
    Revolut,
    Spherical,
    Prismatic,
};

class Joint
{
    Coordinates coordinates_; // element of SE(3) describing how the joint is transforming a reference frame
    JointType type;

public:
    void set_joint_coordinates(const Coordinates &coordinates);
    const Coordinates &get_joint_coordinates() const;
    void set_joint_type(const JointType &joint_type);
    const JointType &get_joint_type() const;
};

class Segment
{
    Coordinates coordinates_; // element of SE(3) describing how the segment is transforming a reference frame

public:
    void set_segment_coordinates(const Coordinates &coordinates);
    const Coordinates &get_segment_coordinates() const;
};

class Arm
{
    std::vector<Joint> joints_;    //First joint (with index 0 in the vector) is the one closest to the ground
    std::vector<Segment> segments_; //First segment (with index 0 in the vector) is the one closest to the ground

public:
    Coordinates effector_coordinates() const; // for a given Arm configuration gives element of SE(3) describing effectors
                                                  //  reference frame position and orientation
    void set_arm_joints(const std::vector<Joint> &joints);
    const std::vector<Joint>& get_arm_joints() const;
    void set_arm_segments(const std::vector<Segment> &segments);
    const std::vector<Segment>& get_arm_segments() const;
    Coordinates get_nth_segment_coordinates(int n) const;
    Coordinates get_nth_joint_coordinates(int n) const;
};

#endif