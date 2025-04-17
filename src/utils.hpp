
#pragma once

#include "Matrix.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"

auto square(auto x) { return x * x; }

raylib::Matrix diagonal_matrix(const raylib::Vector3 &v);

raylib::Matrix cross_matrix(const raylib::Vector3 &v);

raylib::Matrix reorthonormalize(const raylib::Matrix &m);

raylib::Quaternion angular_velocity_to_rotation(
    const raylib::Vector3 &angular_velocity, const float delta
);
