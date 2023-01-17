#include "spdlog/spdlog.h"
#include "common.h"
#include "ArmVis.h"
#include "vis_interface.h"

void generateMockData(std::vector<float> &transform_data);

int main(int argc, char *argv[])
{

	// Manipulator
	Arm arm;

	std::vector<Joint> joints;
	std::vector<Segment> segments;
	Joint joint;
	Segment segment;
	std::vector<Coordinates> joints_coordinates;
	std::vector<Coordinates> segments_coordinates;
	Coordinates coordinates; 
	// coordinates = {1,2,4,3,2,2};
	coordinates = {0,0.5*M_PI,0,0,0,0};
	// std::cout << exponential_coordinates_to_SE3(coordinates) << std::endl;
	// return 0;
	// How to add joints and segments:
	// Coordinates are exponential coordinates on se(3) representing how the given joint will bend or how long will be given segment
	// Rotations are normalized to -pi, pi. 
	// First three coordinates are for rotations and should be used only in joints.
	// For now. its up to the user to provide rotations that make sense regarding the joint type.
	// Second three of coordinates are for translations and should be used only for segments.
	// To properly integrate with visualisation segment length should be written as 5th coordinate.
	coordinates = {-0.1*M_PI,0,0,0,0,0};
	joints_coordinates.push_back(coordinates);
	coordinates = {0,0,0,0,0.5,0};
	segments_coordinates.push_back(coordinates);

	coordinates = {0.3*M_PI,0,0,0,0,0};
	joints_coordinates.push_back(coordinates);
	coordinates = {0,0,0,0,0.4,0};
	segments_coordinates.push_back(coordinates);

	coordinates = {0.45*M_PI,0,0,0,0,0};
	joints_coordinates.push_back(coordinates);
	coordinates = {0,0,0,0,0.25,0};
	segments_coordinates.push_back(coordinates);

	int no_of_joints = joints_coordinates.size();
	for (int i = 0; i < no_of_joints; i++)
	{	
		joint.set_coordinates(joints_coordinates[i]); 
		joint.set_type(Revolut_pitch);
		joints.push_back(joint); 
		segment.set_coordinates(segments_coordinates[i]); 
		segments.push_back(segment);

	}

	arm.set_joints(joints);
	arm.set_segments(segments);

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
		{
			end_pos = tmp1;
			end_rot = tmp2;
			effector_coordinates = {end_rot.x, end_rot.y, end_rot.z, end_pos.x, end_pos.y, end_pos.z};
			arm.inverse_kinematics(effector_coordinates);
			arm_coordinates = arm_to_float_vector(arm);
			std::cout << end_pos.x << ", " << end_pos.y << ", " << end_pos.z << ", " << end_rot.x << ", " << end_rot.y << ", " << end_rot.z << '\n';
		}
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