#include "cube.hpp"
#include "Model.hpp"
#include "consts.hpp"
#include "defs.hpp"
#include "raylib.h"
#include "rlgl.h"
#include <print>
#include <raymath.h>

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

  return direction * normal.y *
         (is_dynamic ? material.dynamic_friction() : material.static_friction()
         );
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

// void Cube::draw() {
//   const auto mesh = GenMeshCube(_size.x, _size.y, _size.z);
//   auto model = raylib::Model(mesh);
//   model.transform = _orientation.ToMatrix();
//   DrawModel(model, _position, 1.0f, _color);
//   // _position.DrawCube(_size, _color);
//   // _position.DrawCubeWires(_size, raylib::Color::Black());
// }
