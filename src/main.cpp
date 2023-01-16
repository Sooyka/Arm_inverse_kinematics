#include "spdlog/spdlog.h"
#include "inverse_kinematics.h"

// using namespace ArmVis;

int main(int argc, char *argv[])
{
    // Hello hi;
    // hi.print();

    // takin the parameters for the arm
    
    const IOFormat fmt(2, DontAlignCols, "\t", " ", "", "", "", "");

    // visualisation initialisation
    spdlog::info("Welcome to spdlog!");
    bool condition = true;
    Vector3d  target_position(1, 0, 0);
    std::cout << target_position.format(fmt) << std::endl; 
    

    while (condition)
    {
        // take position parameters
        // parameters will have form of a point in 3D space 
        // target_position = take_parameters();

        bool parameters_change = true;
        if (parameters_change)
        {
            // calculate inverse kinematics

        }

        // push it to visualisation
    }


    // //Initialize visualization
	// ArmVis viz;
	// //Arguments are: window title, window width, window height, number of vertices per circle, radius of cylinder
	// viz.Init("Arm Visualization", 1280, 720, 15, 0.1f);

	// //Prepare transofrmation data
	// std::vector<float> transform_data;
	// generateMockData(transform_data);

	// //Set data pointer
	// viz.setUserPointer(&transform_data);

	// //End position to by set by visualization gui
	// ArmVis::vec3 end_pos;

	// //Rendering loop
	// while (!viz.WindowShouldClose())
	// {
	// 	//Render scene and process gui
	// 	viz.OnUpdate();

	// 	//Retrieve end pos from gui
	// 	ArmVis::vec3 tmp = viz.getEndPos();

	// 	//Do something if position changed
	// 	if (tmp != end_pos)
	// 	{
	// 		end_pos = tmp;
	// 		std::cout << end_pos.x << ", " << end_pos.y << ", " << end_pos.z << '\n';
	// 	}
	// }

    return 0;
}