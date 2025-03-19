#include "cube.hpp"
#include "consts.hpp"
#include <Model.hpp>
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

void Cube::update(float delta) {
  if (_position.y <= _size.y / 2) {
    _position.y = _size.y / 2;
    _velocity.y *= -BOUNCE_COEFFICIENT;

    if (_velocity.y < 0.1f) {
      _velocity.y = 0.0f;
      _acceleration += friction(_velocity, GRAVITY * _mass, _material);
    }
  }

  body_update(delta);
}
