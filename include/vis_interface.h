#pragma once

#include "arm.h"

std::vector<float> arm_to_float_vector(const Arm& arm);

std::vector<float> matrix_to_float_vector(const Matrix4f& matrix);

Matrix4f segment_scaling_for_drawing(const Segment& segment);