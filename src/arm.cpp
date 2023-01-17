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

float Segment::get_length() const
{
    Coordinates coordinates;
    coordinates = get_coordinates();
    float x_t, y_t, z_t;
    x_t = coordinates.x_t;
    y_t = coordinates.y_t;
    z_t = coordinates.z_t;
    return sqrt(x_t * x_t + y_t * y_t + z_t * z_t);
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

int Arm::no_of_DoF() const
{
    std::vector<Joint> joints = get_joints();
    Joint joint;
    int no_of_DoF = 0;
    int no_of_joints = joints.size();
    for (int i = 0; i < no_of_joints; i++)
    {
        joint = joints[i];
        switch (joint.get_type())
        {
        case Revolut_pitch:
            no_of_DoF += 1;
            break;
        case Revolut_yawn:
            no_of_DoF += 1;
            break;
        case Revolut_roll:
            no_of_DoF += 1;
            break;
        case Spherical:
            no_of_DoF += 3;
            break;
        case Prismatic:
            no_of_DoF += 1;
            break;
        default:
            no_of_DoF += 0;
        }
    }
    return no_of_DoF;
};
void Arm::set_parameters(const std::vector<float> &parameters)
{
    Joint old_joint;
    std::vector<Joint> old_joints;
    old_joints = get_joints();
    int no_of_joints = old_joints.size();

    Joint new_joint;
    std::vector<Joint> new_joints;
    Coordinates new_joint_coordinates;
    JointType old_joint_type;
    for (int i = 0; i < no_of_joints; i++)
    {
        old_joint = old_joints[i];
        old_joint_type = old_joint.get_type();
        switch (old_joint_type)
        {
        case Revolut_pitch:
            // parameters.push_back(joint_coordinates.x_r);
            new_joint.set_type(old_joint_type);
            new_joint_coordinates = {parameters[i], 0, 0, 0, 0, 0};
            break;
        case Revolut_yawn:
            // parameters.push_back(joint_coordinates.z_r);
            new_joint.set_type(old_joint_type);
            new_joint_coordinates = {0, 0, parameters[i], 0, 0, 0};
            break;
        case Revolut_roll:
            // parameters.push_back(joint_coordinates.y_r);
            new_joint.set_type(old_joint_type);
            new_joint_coordinates = {0, parameters[i], 0, 0, 0, 0};
            break;
        case Spherical:
            // parameters.push_back(joint_coordinates.x_r);
            // parameters.push_back(joint_coordinates.y_r);
            // parameters.push_back(joint_coordinates.z_r);
            new_joint.set_type(old_joint_type);
            new_joint_coordinates = {parameters[i], parameters[i + 1], parameters[i + 2], 0, 0, 0};
            i += 2;
            break;
        case Prismatic:
            // parameters.push_back(joint_coordinates.y_t);
            new_joint.set_type(old_joint_type);
            new_joint_coordinates = {0, 0, 0, 0, parameters[i], 0};
            break;
        default:
            break;
        }
        new_joint.set_coordinates(new_joint_coordinates);
        new_joints.push_back(new_joint);
    }
    set_joints(new_joints);
}

std::vector<float> Arm::get_parameters() const
{
    std::vector<float> parameters;
    Joint joint;
    std::vector<Joint> joints;
    Coordinates joint_coordinates;
    joints = get_joints();
    int no_of_joints = joints.size();
    JointType jointtype;
    for (int i = 0; i < no_of_joints; i++)
    {
        joint = joints[i];
        joint_coordinates = joint.get_coordinates();
        switch (joint.get_type())
        {
        case Revolut_pitch:
            parameters.push_back(joint_coordinates.x_r);
            break;
        case Revolut_yawn:
            parameters.push_back(joint_coordinates.z_r);
            break;
        case Revolut_roll:
            parameters.push_back(joint_coordinates.y_r);
            break;
        case Spherical:
            parameters.push_back(joint_coordinates.x_r);
            parameters.push_back(joint_coordinates.y_r);
            parameters.push_back(joint_coordinates.z_r);
            break;
        case Prismatic:
            parameters.push_back(joint_coordinates.y_t);
            break;
        default:
            break;
        }
    }
    return parameters;
};

void Arm::inverse_kinematics(Coordinates coordinates)
{
    const Matrix4f target_frame = exponential_coordinates_to_SE3(coordinates);
    Matrix4f current_frame = effector_frame();
    // log
    int no_of_DoF = Arm::no_of_DoF();
    Matrix<float, 6, Dynamic> Jacobian;
};

// Calculates screw matrices.
std::vector<Sn_theta> Arm::get_Sn_theta() const
{
    Segment segment;
    std::vector<Segment> segments;
    segments = get_segments();
    int no_of_segments = segments.size();
    std::vector<float> length_along_segments;
    float length_summation = 0;
    length_along_segments.push_back(length_summation);
    for (int i = 0; i < no_of_segments; i++)
    {
        segment = segments[i];
        length_summation += segment.get_length();
        length_along_segments.push_back(length_summation);
    }
    std::vector<Sn_theta> Sn_theta_vec;
    Sn_theta Sn_theta_i;
    Matrix4f Sn = Matrix4f::Zero();
    float theta;
    float theta_square;
    // for(int i = 1; )
    Joint joint;
    std::vector<Joint> joints;
    joints = get_joints();

    Sn_theta_i.Sn = Sn;
    Sn_theta_i.theta = theta;
    Sn_theta_vec.push_back(Sn_theta_i);
    Coordinates coordinates;
    float x_r, y_r, z_r;
    Matrix3f omega;
    float epsilon = 0.0000001;
    for (int i = 0; i <= no_of_segments; i++)
    {
        coordinates = joints[i].get_coordinates();
        x_r = coordinates.x_r;
        y_r = coordinates.y_r;
        z_r = coordinates.z_r;
        theta_square = x_r * x_r + y_r * y_r + z_r * z_r;

        if (theta_square)
        {
            Sn = Matrix4f::Zero();
            theta = 0;
            Sn_theta_i.Sn = Sn;
            Sn_theta_i.theta = theta;
            Sn_theta_vec.push_back(Sn_theta_i);
        }
        theta = sqrt(theta_square);
        x_r /= theta;
        y_r /= theta;
        z_r /= theta;

        omega << 0, -z_r, y_r,
            z_r, 0, -x_r,
            -y_r, x_r, 0;

        Sn.block(0, 0, 3, 3) = omega;
        Sn.block(0, 3, 3, 1) << 0, -length_along_segments[i], 0;
        Sn_theta_i.Sn = Sn;
        Sn_theta_i.theta = theta;
        Sn_theta_vec.push_back(Sn_theta_i);
    }

    return Sn_theta_vec;
};
