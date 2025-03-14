#pragma once

#include "Mesh.hpp"
#include "body.hpp"
#include "material.hpp"
#include <Color.hpp>
#include <Vector3.hpp>

class Cube final : public Body {
public:
  Cube(
      raylib::Vector3 size,
      raylib::Color color,
      BodyMaterial material,
      raylib::Vector3 position = raylib::Vector3::Zero(),
      raylib::Quaternion orientation = raylib::Quaternion::Identity()
  )
      : Body(position, orientation), _size(size), _material(material) {
    _color = color;
  };

  ~Cube() = default;

  void update(const float delta) override;

  float moment_of_inertia(const raylib::Vector3 &) override {
    return 1.f / 6.f * _mass * (_size.x * _size.x);
  }

protected:
  raylib::MeshUnmanaged create_mesh() override {
    return raylib::MeshUnmanaged::Cube(_size.x, _size.y, _size.z);
  }

private:
  raylib::Vector3 _size;
  const BodyMaterial _material;
};
