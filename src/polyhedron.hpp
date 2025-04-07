#pragma once

#include "body.hpp"
#include "material.hpp"
#include <Color.hpp>
#include <Mesh.hpp>
#include <Vector3.hpp>

class Polyhedron final : public Body {
public:
  using Vertices = std::span<raylib::Vector3>;
  using Face = std::
      tuple<raylib::Vector3, raylib::Vector3, raylib::Vector3, raylib::Vector3>;

  Polyhedron(
      raylib::Mesh &&mesh,
      const float density,
      const raylib::Color &color,
      const BodyMaterial &material,
      const raylib::Vector3 &position = raylib::Vector3::Zero(),
      const raylib::Quaternion &orientation = raylib::Quaternion::Identity()
  );

  ~Polyhedron() = default;

  const raylib::Matrix &inverse_body_inertia_tensor() const override {
    return _inverse_inertia_tensor;
  }

  Vertices vertices() { return _vertices; }
  std::span<Face> faces() { return _faces; }

protected:
  raylib::MeshUnmanaged get_mesh() override { return std::move(_mesh); }

private:
  raylib::Mesh _mesh;
  const BodyMaterial _material;

  const float _density;

  raylib::Matrix _inverse_inertia_tensor;

  std::vector<raylib::Vector3> _vertices;
  std::vector<Face> _faces;
};
