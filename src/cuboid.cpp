#include "cuboid.hpp"
#include "consts.hpp"
#include <print>
#include <raymath.h>
#include <rlgl.h>

constexpr raylib::Vector3
aerodynamic_drag(const raylib::Vector3 &velocity, const float area) {
  return velocity * velocity * area * -DRAG_COEFFICIENT_CUBE * AIR_DENSITY /
         2.0f;
}

constexpr raylib::Vector3 friction(
    const raylib::Vector3 &velocity,
    const raylib::Vector3 &normal,
    const BodyMaterial &material
) {
  return raylib::Vector3::Zero();
  const auto velocity_magnitude = velocity.Length();

  const auto is_dynamic = velocity_magnitude >= 0.1f;

  const auto dynamic_friction = material.dynamic_friction();
  const auto static_friction = material.static_friction();

  const auto friction =
      normal.Length() * (is_dynamic ? dynamic_friction : static_friction);

  const auto friction_force = velocity.Scale(-friction / velocity_magnitude);

  std::println(
      "velocity: {}, normal: {}, friction: {}, friction_force: {}",
      velocity,
      normal,
      friction,
      friction_force
  );
  return friction_force;
}

void Cuboid::update(float delta) {
  // // iterate over vertices of the cuboid
  const auto half_size = _size / 2.f;

  const std::array<raylib::Vector3, 8> vertex_offsets = {
      raylib::Vector3{half_size.x, half_size.y, half_size.z},
      raylib::Vector3{-half_size.x, half_size.y, half_size.z},
      raylib::Vector3{half_size.x, -half_size.y, half_size.z},
      raylib::Vector3{-half_size.x, -half_size.y, half_size.z},
      raylib::Vector3{half_size.x, half_size.y, -half_size.z},
      raylib::Vector3{-half_size.x, half_size.y, -half_size.z},
      raylib::Vector3{half_size.x, -half_size.y, -half_size.z},
      raylib::Vector3{-half_size.x, -half_size.y, -half_size.z}
  };

  auto touching_ground = 0;

  for (auto &vertex_offset : vertex_offsets) {
    const auto vertex = _position + vertex_offset;

    if (vertex.y < 0) {
      touching_ground++;
    }
  }

  for (auto &vertex_offset : vertex_offsets) {
    const auto vertex = _position + vertex_offset;

    if (vertex.y < 0) {
      const auto vertex_velocity =
          _velocity + _angular_velocity * vertex_offset.Length();
      std::println(
          "vertex_velocity: {}, offset: {}, point: {} ({})",
          vertex_velocity,
          vertex_offset,
          vertex,
          touching_ground
      );

      // apply_force(-GRAVITY * _mass / touching_ground, vertex_offset);
    }
  }

  if (_position.y <= _size.y / 2) {
    _position.y = _size.y / 2;

    if (_velocity.y < 0.1f) {
      _velocity.y = 0.0f;
      _acceleration += friction(_velocity, GRAVITY * _mass, _material);
    } else {
      _velocity.y *= -BOUNCE_COEFFICIENT;
    }
  }

  body_update(delta);
}

float Cuboid::moment_of_inertia(const raylib::Vector3 &axis) {
  const auto x = _size.x;
  const auto y = _size.y;
  const auto z = _size.z;

  const float I_xx = 1.f / 12.f * _mass * (y * y + z * z);
  const float I_yy = 1.f / 12.f * _mass * (x * x + z * z);
  const float I_zz = 1.f / 12.f * _mass * (x * x + y * y);

  const raylib::Matrix inertia_tensor{
      I_xx,
      0.f,
      0.f,
      0.f,
      0.f,
      I_yy,
      0.f,
      0.f,
      0.f,
      0.f,
      I_zz,
      0.f,
      0.f,
      0.f,
      0.f,
      0.f
  };

  return axis.Transform(inertia_tensor).DotProduct(axis);
}
