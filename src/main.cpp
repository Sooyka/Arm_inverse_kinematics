#include "spdlog/spdlog.h"
#include "inverse_kinematics.h"

using namespace Eigen;    

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

    return 0;
}