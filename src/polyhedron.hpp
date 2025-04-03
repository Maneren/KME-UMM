#pragma once

#include "body.hpp"
#include "material.hpp"
#include <Color.hpp>
#include <Mesh.hpp>
#include <Vector3.hpp>

class Polyhedron final : public Body {
public:
  Polyhedron(
      const raylib::MeshUnmanaged &mesh,
      const float density,
      const raylib::Color &color,
      const BodyMaterial &material,
      const raylib::Vector3 &position = raylib::Vector3::Zero(),
      const raylib::Quaternion &orientation = raylib::Quaternion::Identity()
  );

  ~Polyhedron() = default;

  float moment_of_inertia(const raylib::Vector3 &axis) override;

protected:
  raylib::MeshUnmanaged create_mesh() override { return _mesh; }

private:
  raylib::MeshUnmanaged _mesh;
  const BodyMaterial _material;

  const float _density;

  raylib::Matrix _inertia_tensor;
  raylib::Vector3 _center_of_mass;
  float _mass;

  std::vector<std::tuple<
      raylib::Vector3,
      raylib::Vector3,
      raylib::Vector3,
      raylib::Vector3>>
      _faces;
};
