#pragma once

#include "arm.h"

std::vector<float> arm_to_float_vector(const Arm& arm);

std::vector<double> matrix_to_double_vector(const Matrix4d& matrix);

Matrix4d segment_scaling_for_drawing(const Segment& segment);