#pragma once

#include "Matrix.hpp"
#include "Vector3.hpp"

auto square(auto x) { return x * x; }

raylib::Matrix diagonal_matrix(const raylib::Vector3 &v);
