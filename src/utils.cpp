#include "utils.hpp"

raylib::Matrix diagonal_matrix(const raylib::Vector3 &v) {
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

raylib::Matrix cross_matrix(const raylib::Vector3 &v) {
  return raylib::Matrix{
      0.f,
      v.z,
      -v.y,
      0.f,
      -v.z,
      0.f,
      v.x,
      0.f,
      v.y,
      -v.x,
      0.f,
      0.f,
      0.f,
      0.f,
      0.f,
      0.f
  };
}

raylib::Matrix reorthonormalize(const raylib::Matrix &m) {
  const auto x_row = raylib::Vector3{m.m0, m.m1, m.m2}.Normalize();
  const auto y_row = raylib::Vector3{m.m4, m.m5, m.m6}.Normalize();
  const auto z_row = raylib::Vector3{m.m8, m.m9, m.m10}.Normalize();

  const auto fixed_x = x_row * 0.5f * (3.f - x_row.DotProduct(x_row));
  const auto fixed_y = y_row * 0.5f * (3.f - y_row.DotProduct(y_row));
  const auto fixed_z = z_row * 0.5f * (3.f - z_row.DotProduct(z_row));

  return raylib::Matrix{
      fixed_x.x,
      fixed_y.x,
      fixed_z.x,
      0.f,
      fixed_x.y,
      fixed_y.y,
      fixed_z.y,
      0.f,
      fixed_x.z,
      fixed_y.z,
      fixed_z.z,
      0.f,
      0.f,
      0.f,
      0.f,
      1.f
  };
}
