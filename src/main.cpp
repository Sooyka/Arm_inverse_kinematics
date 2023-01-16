#include "spdlog/spdlog.h"
#include "inverse_kinematics.h"
#include "common.h"
#include "ArmVis.h"

void generateMockData(std::vector <float>& transform_data);

int main(int argc, char *argv[])
{

	// Manipulator
	Arm arm; 

    //Initialize visualization
	ArmVis viz;
	//Arguments are: window title, window width, window height, number of vertices per circle, radius of cylinder
	viz.Init("Arm Visualization", 1280, 720, 15, 0.1f);

	//Prepare transofrmation data
	std::vector<float> arm_coordinates;
	generateMockData(arm_coordinates);

	//Set data pointer
	viz.setUserPointer(&arm_coordinates);

	//End position to be set by visualization gui
	ArmVis::vec3 end_pos;
	ArmVis::vec3 end_rot;

	//Rendering loop
	while (!viz.WindowShouldClose())
	{
		//Render scene and process gui
		viz.OnUpdate();

		//Retrieve end pos from gui
		ArmVis::vec3 tmp1 = viz.getEndPos();
		ArmVis::vec3 tmp2 = viz.getEndRot();
		//Do something if position changed
		if (tmp1 != end_pos || tmp2 != end_rot)
		{
			end_pos = tmp1;
			end_rot = tmp2;
			arm.set_arm_joints();
			std::cout << end_pos.x << ", " << end_pos.y << ", " << end_pos.z << '\n';
		}
	}

    return 0;
}




void generateMockData(std::vector <float>& transform_data)
{
	const float angle_offset = 0.25f * 3.1415926535f;
	float translation[3] = { 0.0f, 0.0f, 0.0f };
	float height = 1.0f;

	for (int i = 0; i <10 ; i++)
	{
		float s = sin(angle_offset * float(i));
		float c = cos(angle_offset * float(i));

		float matrix[16] = {             c,             -s,           0.0f, 0.0f,
							    height * s,     height * c,           0.0f, 0.0f,
									  0.0f,           0.0f,           1.0f, 0.0f,
							translation[0], translation[1], translation[2], 1.0f };

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