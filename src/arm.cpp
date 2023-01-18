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

    Matrix4f current_frame;
    Coordinates V_b_coordinates;
    // Matrix<float, 6, 1> V_b;
    MatrixXf V_b;
    V_b.resize(6, 1);
    bool precision_achieved = false;
    int no_of_DoF = Arm::no_of_DoF();

    std::vector<float> parameters;
    MatrixXf parameters_delta_vect;
    parameters_delta_vect.resize(no_of_DoF, 1);
    MatrixXf Jacobian;
    Jacobian.resize(6, no_of_DoF);
    parameters = get_parameters();
    int no_of_steps = 0;
    const int max_no_of_steps = 10;

    float epsilon = 0.05;
    current_frame = effector_frame();
    Matrix4f Identityhmm = current_frame.inverse() * target_frame;
    V_b_coordinates = SE3_to_exponential_coordinates(current_frame.inverse() * target_frame);
    V_b(0, 0) = V_b_coordinates.x_r;
    V_b(1, 0) = V_b_coordinates.y_r;
    V_b(2, 0) = V_b_coordinates.z_r;
    V_b(3, 0) = V_b_coordinates.x_t;
    V_b(4, 0) = V_b_coordinates.y_t;
    V_b(5, 0) = V_b_coordinates.z_t;
    while (!precision_achieved && no_of_steps < max_no_of_steps)
    {

        // transforming to V_b

        // Building Jacobian
        Jacobian = get_jacobian();
        MatrixXf Jacobian_pseudoinverse;
        Jacobian_pseudoinverse.resize(no_of_DoF, 6);
        Jacobian_pseudoinverse = Jacobian.completeOrthogonalDecomposition().pseudoInverse();
        parameters_delta_vect = Jacobian_pseudoinverse * V_b;

        // std::cout <<"Jacobian pseudoinverse:"<< std::endl<< Jacobian_pseudoinverse << std::endl << std::endl <<"Jacobian:" << std::endl << Jacobian << std::endl << std::endl << std::endl;
        float diff_norm;
        diff_norm = parameters_delta_vect.norm();
        std::cout << diff_norm << std::endl;
        if (diff_norm < epsilon)
        {
            precision_achieved = true;
        }

        for (int i = 0; i < no_of_DoF; i++)
        {
            parameters[i] += parameters_delta_vect(i, 0) ;
        }

        set_parameters(parameters);

        no_of_steps++;
    }
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
    for (int i = 0; i < no_of_segments - 1; i++)
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
    JointType joint_type;

    // Sn_theta_i.Sn = Sn;
    // Sn_theta_i.theta = theta;
    // Sn_theta_vec.push_back(Sn_theta_i);
    Coordinates coordinates;
    float x_r, y_r, z_r, x_t, y_t, z_t;
    Matrix3f omega;
    float epsilon = 0.001;
    for (int i = 0; i < no_of_segments; i++)
    {

        coordinates = joints[i].get_coordinates();
        joint_type = joints[i].get_type();
        x_r = coordinates.x_r;
        y_r = coordinates.y_r;
        z_r = coordinates.z_r;
        x_t = coordinates.x_t;
        y_t = coordinates.y_t;
        z_t = coordinates.z_t;
        theta_square = x_r * x_r + y_r * y_r + z_r * z_r;

        if (theta_square < epsilon)
        {
            Sn = Matrix4f::Zero();
            theta = 0;
            if (joint_type == Prismatic)
            {
                theta = sqrt(x_t * x_t + y_t * y_t + z_t * z_t);
                x_t /= theta;
                y_t /= theta;
                z_t /= theta;
                Sn.block(0, 3, 3, 1) << x_t, y_t, z_t;
            }
            x_r = 0;
            y_r = 0;
            z_r = 0;
            switch (joint_type)
            {
            case Revolut_pitch:
                x_r = 1;
                break;
            case Revolut_yawn:
                z_r = 1;
                break;
            case Revolut_roll:
                y_r = 1;
                break;
            default:
                break;
            }
            omega << 0, -z_r, y_r,
                z_r, 0, -x_r,
                -y_r, x_r, 0;

            Sn.block(0, 0, 3, 3) = omega;
            Sn_theta_i.Sn = Sn;
            Sn_theta_i.theta = theta;
            Sn_theta_vec.push_back(Sn_theta_i);
            // std::cout << joint_type << std::endl
            //           << "Sn:" << std::endl
            //           << Sn << std::endl
            //           << std::endl
            //           << "theta:" << theta << std::endl
            //           << std::endl;
            continue;
        }
        theta = sqrt(theta_square);
        x_r /= theta;
        y_r /= theta;
        z_r /= theta;
        Vector3f offset;
        if (joint_type == Spherical) // here the velocity is calculated wrong
        {
            std::vector<Matrix3f> omega_3;
            offset << 0, -length_along_segments[i], 0;
            omega << 0, 0, 0,
                0, 0, -x_r,
                0, x_r, 0;
            omega_3.push_back(omega);

            omega << 0, 0, y_r,
                0, 0, 0,
                -y_r, 0, 0;
            omega_3.push_back(omega);

            omega << 0, -z_r, 0,
                z_r, 0, 0,
                0, 0, 0;
            omega_3.push_back(omega);
            // not shure about theta here
            for (int j = 0; j < 3; j++)
            {

                Sn.block(0, 0, 3, 3) = omega_3[j];
                Sn.block(0, 3, 3, 1) << omega_3[i] * offset;
                Sn_theta_i.Sn = Sn;
                Sn_theta_i.theta = theta;
                Sn_theta_vec.push_back(Sn_theta_i);
            }
            continue;
        }
        omega << 0, -z_r, y_r,
            z_r, 0, -x_r,
            -y_r, x_r, 0;
        Sn = Matrix4f::Zero();
        Sn.block(0, 0, 3, 3) = omega;

        offset << 0, -length_along_segments[i], 0;

        Sn.block(0, 3, 3, 1) << omega * offset;

        Sn_theta_i.Sn = Sn;
        Sn_theta_i.theta = theta;
        Sn_theta_vec.push_back(Sn_theta_i);
        // spdlog::info("Sn: \n {},\n\n theta:\n {}", Sn,theta);
        // std::cout << joint_type << std::endl
        //           << "Sn:" << std::endl
        //           << Sn << std::endl
        //           << std::endl
        //           << "theta:" << theta << std::endl
        //           << std::endl;
    }

    return Sn_theta_vec;
};

Matrix<float, 6, Dynamic> Arm::get_jacobian() const
{
    Matrix4f M_temp = Matrix4f::Identity();
    Segment segment;
    std::vector<Segment> segments;
    segments = get_segments();
    int no_of_segments = segments.size();
    float length_summation = 0;
    for (int i = 0; i < no_of_segments; i++)
    {
        segment = segments[i];
        length_summation += segment.get_length();
    }
    M_temp(1, 3) = length_summation;
    const Matrix4f M = M_temp;

    Matrix<float, 6, Dynamic> Jacobian;
    int no_of_DoF = Arm::no_of_DoF();
    Jacobian.resize(6, no_of_DoF);

    std::vector<Sn_theta> Sn_theta_vec = get_Sn_theta();
    Sn_theta Sn_theta_i;

    std::vector<Bn_theta> Bn_theta_vec;
    Bn_theta Bn_theta_i;

    for (int i = 0; i < no_of_DoF; i++)
    {
        Sn_theta_i = Sn_theta_vec[i];
        Bn_theta_i.Bn = Sn_to_Bn(Sn_theta_i.Sn, M);
        Bn_theta_i.theta = Sn_theta_i.theta;

        Bn_theta_vec.push_back(Bn_theta_i);
        // std::cout << "Bn:" << std::endl
        //           << Bn_theta_i.Bn << std::endl
        //           << std::endl
        //           << "theta:" << Bn_theta_i.theta << std::endl
        //           << std::endl
        //           << "M:" << std::endl
        //           << M << std::endl
        //           << std::endl;
    }

    Matrix<float, 6, 1> Jacobian_column = Matrix<float, 6, 1>::Zero();
    // Matrix<float, 6, 1> B_i;
    Matrix4f B_i;
    Matrix4f B_j;
    float theta_j;
    Coordinates B_j_coordinates;
    Coordinates B_j_coordinates_m;
    for (int i = 0; i < no_of_DoF; i++)
    {
        B_i = Bn_theta_vec[i].Bn;

        for (int j = i + 1; j < no_of_DoF; j++)
        {
            B_j = Bn_theta_vec[j].Bn;
            theta_j = Bn_theta_vec[j].theta;
            B_j_coordinates = se3_to_exponential_coordinates(B_j * theta_j);
            B_j_coordinates_m = se3_to_exponential_coordinates(-B_j * theta_j);

            B_i = exponential_coordinates_to_SE3(B_j_coordinates_m) * B_i * exponential_coordinates_to_SE3(B_j_coordinates);
        }

        Jacobian_column(0) = B_i(2, 1);
        Jacobian_column(1) = B_i(0, 2);
        Jacobian_column(2) = B_i(1, 0);
        Jacobian_column(3) = B_i(0, 3);
        Jacobian_column(4) = B_i(1, 3);
        Jacobian_column(5) = B_i(2, 3);

        Jacobian.block(0, i, 6, 1) = Jacobian_column;
    }

    return Jacobian;
};

Matrix4f Arm::Sn_product() const
{
    Matrix4f M_temp = Matrix4f::Identity();
    Segment segment;
    std::vector<Segment> segments;
    segments = get_segments();
    int no_of_segments = segments.size();
    float length_summation = 0;
    for (int i = 0; i < no_of_segments; i++)
    {
        segment = segments[i];
        length_summation += segment.get_length();
    }
    M_temp(1, 3) = length_summation;
    const Matrix4f M = M_temp;

    int no_of_DoF = Arm::no_of_DoF();

    std::vector<Sn_theta> Sn_theta_vec = get_Sn_theta();
    Sn_theta Sn_theta_i;
    Matrix4f Sn;
    Matrix4f Sn_product = Matrix4f::Identity();
    float theta;
    for (int i = 0; i < no_of_DoF; i++)
    {
        Sn_theta_i = Sn_theta_vec[i];
        Sn = Sn_theta_i.Sn;
        theta = Sn_theta_i.theta;
        Sn_product = Sn_product * exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Sn_theta_vec[i].Sn * Sn_theta_vec[i].theta));
    }
    Sn_product = Sn_product * M;
    // std::cout<< "Sn prod"<<std::endl;
    // std::cout << "Sn:" << std::endl << Sn<<std::endl<<std::endl<< "theta:" << theta<<std::endl<<std::endl<<"M:"<<std::endl<<M<<std::endl<<std::endl;
    // Matrix4f prod1 = exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Sn * theta))*M;
    // Matrix4f prod2 = M*exponential_coordinates_to_SE3(se3_to_exponential_coordinates(M.inverse()*Sn*M * theta));
    // Matrix4f prod3 = M*exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Sn_to_Bn(Sn, M) * theta));
    // std::cout << "prod1:" << std::endl << prod1<< std::endl << std::endl <<"prod2:" << std::endl << prod2 << std::endl << std::endl << std::endl;
    // std::cout << "prod3:" << std::endl << prod3<< std::endl << std::endl;
    // std::cout << "sn_exp:" << std::endl
    //           << exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Sn_theta_vec[0].Sn * Sn_theta_vec[0].theta)) << std::endl;
    // std::cout << "S_n_product:" << std::endl
    //           << Sn_product << std::endl
    //           << std::endl
    //           << "Effector frame:" << std::endl
    //           << effector_frame() << std::endl
    //           << std::endl
    //           << std::endl;

    return Sn_product;
}
Matrix4f Arm::Bn_product() const
{
    Matrix4f M_temp = Matrix4f::Identity();
    Segment segment;
    std::vector<Segment> segments;
    segments = get_segments();
    int no_of_segments = segments.size();
    float length_summation = 0;
    for (int i = 0; i < no_of_segments; i++)
    {
        segment = segments[i];
        length_summation += segment.get_length();
    }
    M_temp(1, 3) = length_summation;
    const Matrix4f M = M_temp;

    int no_of_DoF = Arm::no_of_DoF();

    std::vector<Sn_theta> Sn_theta_vec = get_Sn_theta();
    Sn_theta Sn_theta_i;

    std::vector<Bn_theta> Bn_theta_vec;
    Bn_theta Bn_theta_i;
    Matrix4f Sn, Bn;
    float theta;
    for (int i = 0; i < no_of_DoF; i++)
    {
        Sn_theta_i = Sn_theta_vec[i];

        Sn = Sn_theta_i.Sn;
        theta = Sn_theta_i.theta;
        Bn_theta_i.Bn = Sn_to_Bn(Sn, M);
        Bn = Bn_theta_i.Bn;
        // std::cout <<"M inverse:"<<std::endl<< M.inverse()<<std::endl<<"M:"<<std::endl<< M<<std::endl<<"Sn:"<<std::endl<< Sn<<std::endl<<"Bn:"<<std::endl<< Bn<<std::endl;
        Bn_theta_i.theta = Sn_theta_i.theta;

        Bn_theta_vec.push_back(Bn_theta_i);
        // std::cout << i <<std::endl;
    }

    Matrix4f Bn_product = M;
    for (int i = 0; i < no_of_DoF; i++)
    {
        Bn_product = Bn_product * exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Bn_theta_vec[i].Bn * Bn_theta_vec[i].theta));
        // std::cout << i <<std::endl;
    }
    // std::cout<< "Bn prod"<<std::endl;
    // std::cout << "Sn:" << std::endl << Sn<<std::endl<<std::endl<< "theta:" << theta<<std::endl<<std::endl<<"M:"<<std::endl<<M<<std::endl<<std::endl;
    // Matrix4f prod1 = exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Sn * theta))*M;
    // Matrix4f prod2 = M*exponential_coordinates_to_SE3(se3_to_exponential_coordinates(M.inverse()*Sn*M * theta));
    // Matrix4f prod3 = M*exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Sn_to_Bn(Sn, M) * theta));
    // std::cout << "prod1:" << std::endl << prod1<< std::endl << std::endl <<"prod2:" << std::endl << prod2 << std::endl << std::endl << std::endl;
    // std::cout << "prod3:" << std::endl << prod3<< std::endl << std::endl;
    // std::cout<<"bn_exp:"<<std::endl<< exponential_coordinates_to_SE3(se3_to_exponential_coordinates(Bn_theta_vec[0].Bn * Bn_theta_vec[0].theta)) <<std::endl;
    // std::cout<<"M*M.inverse()*Sn*M"<<std::endl<<M*M.inverse()*Sn*M<<std::endl;
    // std::cout<<"M.inverse()*Sn*M"<<std::endl<<M.inverse()*Sn*M<<std::endl;
    // std::cout<<"M.inverse()*Sn*M"<<std::endl<<M.inverse()*Sn*M<<std::endl;
    // std::cout << "Effector frame:" << std::endl
    //           << effector_frame() << std::endl
    //           << std::endl;
    // std::cout<< "B_n product:" << std::endl
    // << Bn_product << std::endl
    // << std::endl
    // << std::endl;
    return Bn_product;
}