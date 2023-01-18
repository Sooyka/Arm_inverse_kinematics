
#include "common.h"
#include "ArmVis.h"
#include "vis_interface.h"

void generateMockData(std::vector<float> &transform_data);

int main(int argc, char *argv[])
{
	feenableexcept(FE_ALL_EXCEPT & ~FE_INEXACT);
	// spdlog::set_level(spdlog::level::debug);
	// MatrixXf M;
	// M.resize(2,3);
	// M << 2,3,4,
	// 	1,0,1;
	// MatrixXf M_1 = M.completeOrthogonalDecomposition().pseudoInverse();
	// std::cout<< M_1 << std::endl<<M_1*M<<std::endl<< M*M_1 <<std::endl;
	// return 0;
	// Manipulator
	// Matrix4f A, A2;
	// Coordinates test;
	// test = {2, 1, 4, -11, -3, 5};
	// A = exponential_coordinates_to_SE3(test);
	// std::cout << "A1:" << std::endl
	// 		  << A << std::endl
	// 		  << std::endl;
	// // A << 1,0,0,
	// // 0,1,0,
	// // 0,0,1;
	// Coordinates test2 = SE3_to_exponential_coordinates(A);
	// std::cout << test2.x_r << ", " << test2.y_r << ", " << test2.z_r << ", " << test2.x_t << ", " << test2.y_t << ", " << test2.z_t << std::endl;
	// A2 = exponential_coordinates_to_SE3(test2);
	// std::cout << "A2:" << std::endl
	// 		  << A2 << std::endl
	// 		  << std::endl;
	// return 0;



	Arm arm;
	
	std::vector<Joint> joints;
	std::vector<Segment> segments;
	Joint joint;
	Segment segment;
	std::vector<Coordinates> joints_coordinates;
	std::vector<Coordinates> segments_coordinates;
	std::vector<JointType> joints_types;
	Coordinates coordinates;
	// coordinates = {1,2,4,3,2,2};
	// coordinates = {0, 0.5 * M_PI, 0, 0, 0, 0};
	// std::cout << exponential_coordinates_to_SE3(coordinates) << std::endl;
	// return 0;
	// How to add joints and segments:
	// Coordinates are exponential coordinates on se(3) representing how the given joint will bend or how long will be given segment
	// Rotations are normalized to -pi, pi.
	// First three coordinates are for rotations and should be used only in joints.
	// For now. its up to the user to provide rotations that make sense regarding the joint type.
	// Second three of coordinates are for translations and should be used only for segments.
	// To properly integrate with visualisation segment length should be written as 5th coordinate.
	// coordinates = {0,0.1*M_PI,0,0,0,0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_roll);
	// coordinates = {0,0,0,0,0.1,0};
	// segments_coordinates.push_back(coordinates);

	// coordinates = {0,0.1*M_PI,0,0,0,0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_roll);
	// coordinates = {0,0,0,0,0.1,0};
	// segments_coordinates.push_back(coordinates);

	// coordinates = {0.2*M_PI,0,0,0,0,0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_pitch);
	// coordinates = {0,0,0,0,0.5,0};
	// segments_coordinates.push_back(coordinates);

	// coordinates = {0.3*M_PI,0,0,0,0,0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_pitch);
	// coordinates = {0,0,0,0,0.2,0};
	// segments_coordinates.push_back(coordinates);

	// coordinates = {0,0.2*M_PI,0,0,0,0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_roll);
	// coordinates = {0,0,0,0,0.2,0};
	// segments_coordinates.push_back(coordinates);

	// coordinates = {0.45*M_PI,0,0,0,0,0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_pitch);
	// coordinates = {0,0,0,0,0.2,0};
	// segments_coordinates.push_back(coordinates);

	// coordinates = {0.2*M_PI,0,0,0,0,0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_pitch);
	// coordinates = {0,0,0,0,0.2,0};
	// segments_coordinates.push_back(coordinates);

	// coordinates = {0, 0.1 * M_PI, 0, 0, 0, 0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_roll);
	// coordinates = {0, 0, 0, 0, 0.1, 0};
	// segments_coordinates.push_back(coordinates);

	coordinates = {0, 0, 0, 0, 0, 0};
	joints_coordinates.push_back(coordinates);
	joints_types.push_back(Revolut_roll);
	coordinates = {0, 0, 0, 0, 0.1, 0};
	segments_coordinates.push_back(coordinates);

	coordinates = {-0.2 * M_PI, 0, 0, 0, 0, 0};
	joints_coordinates.push_back(coordinates);
	joints_types.push_back(Revolut_pitch);
	coordinates = {0, 0, 0, 0, 0.5, 0};
	segments_coordinates.push_back(coordinates);

	// coordinates = {0, -0.2 * M_PI, 0, 0, 0, 0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_roll);
	// coordinates = {0, 0, 0, 0, 0.2, 0};
	// segments_coordinates.push_back(coordinates);

	coordinates = {0.7 * M_PI, 0, 0, 0, 0, 0};
	joints_coordinates.push_back(coordinates);
	joints_types.push_back(Revolut_pitch);
	coordinates = {0, 0, 0, 0, 0.2, 0};
	segments_coordinates.push_back(coordinates);

	// coordinates = {0, -0.2 * M_PI, 0, 0, 0, 0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_roll);
	// coordinates = {0, 0, 0, 0, 0.2, 0};
	// segments_coordinates.push_back(coordinates);

	coordinates = {0.1 * M_PI, 0, 0, 0, 0, 0};
	joints_coordinates.push_back(coordinates);
	joints_types.push_back(Revolut_pitch);
	coordinates = {0, 0, 0, 0, 0.2, 0};
	segments_coordinates.push_back(coordinates);

	// coordinates = {-0.1 * M_PI, 0, 0, 0, 0, 0};
	// joints_coordinates.push_back(coordinates);
	// joints_types.push_back(Revolut_pitch);
	// coordinates = {0, 0, 0, 0, 0.25, 0};
	// segments_coordinates.push_back(coordinates);

	int no_of_joints = joints_coordinates.size();
	for (int i = 0; i < no_of_joints; i++)
	{
		joint.set_coordinates(joints_coordinates[i]);
		joint.set_type(joints_types[i]);
		joints.push_back(joint);
		segment.set_coordinates(segments_coordinates[i]);
		segments.push_back(segment);
	}

	arm.set_joints(joints);
	arm.set_segments(segments);
std::vector<float> par ;

arm.set_parameters{}
	return 0;
	// Initialize visualization
	ArmVis viz;
	// Arguments are: window title, window width, window height, number of vertices per circle, radius of cylinder
	viz.Init("Arm Visualization", 1280, 720, 15, 0.1f);

	// Prepare transofrmation data
	std::vector<float> arm_coordinates;
	generateMockData(arm_coordinates);

	arm_coordinates = arm_to_float_vector(arm);
	// Set data pointer
	viz.setUserPointer(&arm_coordinates);

	// End position to be set by visualization gui
	ArmVis::vec3 end_pos;
	ArmVis::vec3 end_rot;
	Coordinates effector_coordinates;

	Matrix4f offset_frame = arm.effector_frame();

	Coordinates offset;

	offset = SE3_to_exponential_coordinates(offset_frame);

	std::cout << offset.x_r << std::endl
			  << offset.y_r << std::endl
			  << offset.z_r << std::endl
			  << offset.x_t << std::endl
			  << offset.y_t << std::endl
			  << offset.z_t << std::endl;

	float scale = 0.1;



	// Rendering loop
	while (!viz.WindowShouldClose())
	{
		// Render scene and process gui
		viz.OnUpdate();

		// Retrieve end pos from gui
		ArmVis::vec3 tmp1 = viz.getEndPos();
		ArmVis::vec3 tmp2 = viz.getEndRot();
		// Do something if position changed
		if (tmp1 != end_pos || tmp2 != end_rot)
		{std::cout << end_pos.x << ", " << end_pos.y << ", " << end_pos.z << ", " << end_rot.x << ", " << end_rot.y << ", " << end_rot.z << '\n';
			end_pos = tmp1;
			end_rot = tmp2;
			effector_coordinates = {offset.x_r + end_rot.x * scale, offset.y_r + end_rot.y * scale, offset.z_r + end_rot.z * scale, offset.x_t + end_pos.x * scale, offset.y_t + end_pos.y * scale, offset.z_t + end_pos.z * scale};
			arm.inverse_kinematics(effector_coordinates);
			std::vector<float> arm_temp;
			arm_temp = arm_to_float_vector(arm);
			std::vector<float> raw_matrix_float = matrix_to_float_vector(exponential_coordinates_to_SE3(effector_coordinates));
			for (int j = 0; j < 16; j++)
			{
				arm_temp.push_back(raw_matrix_float[j]);
			}
			arm_coordinates = arm_temp;
			
		}
		// sleep(1);
	}

	return 0;
}

void generateMockData(std::vector<float> &transform_data)
{
	const float angle_offset = 0.25f * 3.1415926535f;
	float translation[3] = {0.0f, 0.0f, 0.0f};
	float height = 1.0f;

	for (int i = 0; i < 10; i++)
	{
		float s = sin(angle_offset * float(i));
		float c = cos(angle_offset * float(i));

		float matrix[16] = {c, -s, 0.0f, 0.0f,
							height * s, height * c, 0.0f, 0.0f,
							0.0f, 0.0f, 1.0f, 0.0f,
							translation[0], translation[1], translation[2], 1.0f};

		translation[0] += height * s;
		translation[1] += height * c;
		translation[2] += 0.0f;

		height *= 0.66f;

		for (int j = 0; j < 16; j++)
		{
			transform_data.push_back(matrix[j]);
		}
	}
}