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
    float get_length() const; 
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

    void set_parameters(const std::vector<float>&); // Sets coordinates that serve as actual parameters.
    std::vector<float> get_parameters() const; // Extracts number of coordinates equal to th number of DoF.d

    void inverse_kinematics(Coordinates coordinates);
    std::vector<Sn_theta> get_Sn_theta() const; // Calculates screw matrices.
    int no_of_DoF() const;
    Matrix<float, 6, Dynamic> get_jacobian()const;
    
    Matrix4f Sn_product()const;
    Matrix4f Bn_product()const;
};



#endif