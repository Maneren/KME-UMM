#pragma once

#include "body.hpp"
#include "material.hpp"
#include "utils.hpp"
#include <Color.hpp>
#include <Mesh.hpp>
#include <Vector3.hpp>

class Cuboid final : public Body {
public:
  Cuboid(
      raylib::Vector3 size,
      raylib::Color color,
      BodyMaterial material,
      raylib::Vector3 position = raylib::Vector3::Zero(),
      raylib::Quaternion orientation = raylib::Quaternion::Identity()
  )
      : Body(position, orientation), _size(size), _material(material) {
    _color = color;
  };

  ~Cuboid() = default;

  void update(const float delta) override;

protected:
  const raylib::Matrix &inverse_body_inertia_tensor() const override {
    return _inverse_inertia_tensor;
  }

  raylib::MeshUnmanaged get_mesh() override {
    return raylib::MeshUnmanaged::Cube(_size.x, _size.y, _size.z);
  }

private:
  raylib::Vector3 _size;
  const BodyMaterial _material;
  raylib::Matrix _inverse_inertia_tensor;

  void update_inertia() {
    const auto x = _size.x;
    const auto y = _size.y;
    const auto z = _size.z;

    const raylib::Vector3 base_tensor{
        (y * y + z * z), (x * x + z * z), (x * x + y * y)
    };
    _inverse_inertia_tensor = diagonal_matrix(base_tensor * mass() / 12.f);
  }
};
