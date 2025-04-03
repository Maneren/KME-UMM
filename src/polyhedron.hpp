#pragma once

#include "body.hpp"
#include "material.hpp"
#include <Color.hpp>
#include <Mesh.hpp>
#include <Vector3.hpp>
#include <print>
#include <ranges>

class Polyhedron final : public Body {
public:
  Polyhedron(
      const raylib::MeshUnmanaged &mesh,
      const raylib::Color &color,
      const BodyMaterial &material,
      const raylib::Vector3 &position = raylib::Vector3::Zero(),
      const raylib::Quaternion &orientation = raylib::Quaternion::Identity()
  )
      : Body(position, orientation), _mesh(mesh), _material(material) {
    _color = color;

    // calculate centroid
    const std::span<raylib::Vector3> vertices(
        reinterpret_cast<raylib::Vector3 *>(mesh.vertices), mesh.vertexCount
    );

    const std::span<unsigned short> indices(
        mesh.indices, mesh.triangleCount * 3
    );

    const auto triangles =
        indices |
        std::views::transform([&vertices](auto i) -> raylib::Vector3 & {
          return vertices[i];
        }) |
        std::views::chunk(3);

    // Initialize faces and compute normals
    _faces = {};
    _faces.reserve(mesh.triangleCount);
    for (const auto triangle : triangles) {
      auto iter = triangle.begin();
      const auto &a = *iter++;
      const auto &b = *iter++;
      const auto &c = *iter++;

      // Compute area-magnitude normal
      raylib::Vector3 n = (b - a).CrossProduct(c - a);

      _faces.emplace_back(a, b, c, n);
    }

    // Compute inertia tensor
    const auto square = [](auto x) { return x * x; };

    _mass = 0.f;
    _center_of_mass = raylib::Vector3::Zero();

    float Ia = 0.0, Ib = 0.0, Ic = 0.0, Iap = 0.0, Ibp = 0.0, Icp = 0.0;
    for (const auto &[a, b, c, _] : _faces) {
      const float determinant_J = a.DotProduct(b.CrossProduct(c));
      const float tetra_volume = determinant_J / 6.0;
      const float tetra_mass = _density * tetra_volume;

      const raylib::Vector3 tetra_center_mass = (a + b + c) / 4.f;

      const auto Tet3I = a * a + b * b + c * c + a * b + a * c + b * c;

      const float V100 = Tet3I.x;
      const float V010 = Tet3I.y;
      const float V001 = Tet3I.z;

      Ia += determinant_J * (V010 + V001);
      Ib += determinant_J * (V100 + V001);
      Ic += determinant_J * (V100 + V010);

      Iap += determinant_J * 2.f * a.y * a.z + 2.f * b.y * b.z +
             2.f * c.y * c.z + a.y * b.z + a.y * c.z + b.y * a.z + b.y * c.z +
             c.y * b.z + c.y * a.z;

      Ibp += determinant_J * 2.f * a.x * a.y + 2.f * b.x * b.y +
             2.f * c.x * c.y + a.x * b.y + a.x * c.y + b.x * a.y + b.x * c.y +
             c.x * b.y + c.x * a.y;

      Icp += determinant_J * 2.f * a.x * a.z + 2.f * b.x * b.z +
             2.f * c.x * c.z + a.x * b.z + a.x * c.z + b.x * a.z + b.x * c.z +
             c.x * b.z + c.x * a.z;

      _center_of_mass += tetra_center_mass * tetra_mass;
      _mass += tetra_mass;
    }

    _center_of_mass /= _mass;

    Ia = _density * Ia / 60.0 -
         _mass * (square(_center_of_mass.y) + square(_center_of_mass.z));
    Ib = _density * Ib / 60.0 -
         _mass * (square(_center_of_mass.x) + square(_center_of_mass.z));
    Ic = _density * Ic / 60.0 -
         _mass * (square(_center_of_mass.x) + square(_center_of_mass.y));
    Iap = _density * Iap / 120.0 -
          _mass * (_center_of_mass.y * _center_of_mass.z);
    Ibp = _density * Ibp / 120.0 -
          _mass * (_center_of_mass.x * _center_of_mass.y);
    Icp = _density * Icp / 120.0 -
          _mass * (_center_of_mass.x * _center_of_mass.z);

    _inertia_tensor = raylib::Matrix{
        Ia,
        Ib,
        Ic,
        0.f,
        -Ibp,
        -Iap,
        -Icp,
        0.f,
        -Icp,
        -Icp,
        0.f,
        0.f,
        -Ibp,
        -Iap,
        0.f,
        0.f
    };

    std::println("mass: {}, center of mass: {}", _mass, _center_of_mass);
    std::println("inertia tensor: {}", _inertia_tensor);
  };

  ~Polyhedron() = default;

  float moment_of_inertia(const raylib::Vector3 &axis) override;

protected:
  raylib::MeshUnmanaged create_mesh() override { return _mesh; }

private:
  raylib::MeshUnmanaged _mesh;
  const BodyMaterial _material;

  const float _density = 1000.0f;

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
