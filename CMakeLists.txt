# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project (Arm_inverse_kinematics)

find_package(Eigen3 REQUIRED)

find_package(spdlog REQUIRED)

# Create a sources variable with a link to all cpp files to compile
set(SOURCES
    src/inverse_kinematics.cpp
    src/main.cpp
)

# Add an executable with the above sources
add_executable(Arm_inverse_kinematics ${SOURCES})

target_link_libraries(Arm_inverse_kinematics PUBLIC spdlog::spdlog  Eigen3::Eigen)

# include_directories(${SPDLOG_INCLUDE_DIR})

# message(${SPDLOG_INCLUDE_DIR}) 

# find_package (Eigen3 REQUIRED)

include_directories(${EIGEN_INCLUDE_DIR} ${SPDLOG_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/include )

# Set the directories that should be included in the build command for this target
# when running g++ these will be included as -I/directory/path/
# target_include_directories(Kinematics
#     PRIVATE 
#         ${PROJECT_SOURCE_DIR}/include
# )
