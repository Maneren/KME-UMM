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
  const auto direction = velocity.Normalize();
  const auto is_dynamic = velocity.LengthSqr() >= 0.1f;

  const auto dynamic_friction = material.dynamic_friction();
  const auto static_friction = material.static_friction();

  return direction * normal.y *
         (is_dynamic ? dynamic_friction : static_friction);
}

void Cuboid::update(float delta) {
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
