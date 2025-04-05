#include "utils.hpp"

raylib::Matrix diagonal_matrix(const Vector3 &v) {
  return raylib::Matrix{
      v.x,
      0.f,
      0.f,
      0.f,
      0.f,
      v.y,
      0.f,
      0.f,
      0.f,
      0.f,
      v.z,
      0.f,
      0.f,
      0.f,
      0.f,
      1.f
  };
}
