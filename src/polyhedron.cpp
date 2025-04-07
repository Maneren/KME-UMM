#include "polyhedron.hpp"
#include "utils.hpp"
#include <print>
#include <ranges>
#include <vector>

Polyhedron::Polyhedron(
    raylib::Mesh &&mesh,
    const float density,
    const raylib::Color &color,
    const BodyMaterial &material,
    const raylib::Vector3 &position,
    const raylib::Quaternion &orientation
)
    : Body(position, orientation), _mesh(std::forward<raylib::Mesh>(mesh)),
      _material(material), _density(density) {
  _color = color;

  const std::span<raylib::Vector3> mesh_vertices(
      reinterpret_cast<raylib::Vector3 *>(_mesh.vertices), _mesh.vertexCount
  );

  _vertices.assign(mesh_vertices.begin(), mesh_vertices.end());

  const std::span<unsigned short> indices(
      _mesh.indices, _mesh.triangleCount * 3
  );

  const auto triangles = indices |
                         std::views::transform([&mesh_vertices](auto i) {
                           return mesh_vertices[i];
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
    const raylib::Vector3 n = (b - a).CrossProduct(c - a);

    _faces.emplace_back(a, b, c, n);
  }

  // Compute inertia tensor
  //
  // Creates a tetrahedron for each triangle face in the mesh, computes its
  // properties and sums them up.

  _mass = 0.f;
  auto center_of_mass = raylib::Vector3::Zero();

  // Note that thorought the computations, it is assumed that the last
  // (not on the triangle face) vertex of the tetrahedron lies at the origin.

  raylib::Vector3 I{0.f, 0.f, 0.f};      // main diagonal of the tensor
  raylib::Vector3 Iprime{0.f, 0.f, 0.f}; // non diagonal elements

  for (const auto &[a, b, c, _] : _faces) {
    // |J| = a ⋅ (b × c) (essentialy a volume of cuboid around the tetrahedron)
    const float determinant_J = a.DotProduct(b.CrossProduct(c));

    // Vₜ = |J| / 6
    const float tetra_volume = determinant_J / 6.0;

    // mₜ = Vₜ * ρ
    const float tetra_mass = _density * tetra_volume;

    // Cₜ = (a + b + c + d) / 4, where d = (0, 0, 0)
    const raylib::Vector3 tetra_center_of_mass = (a + b + c) / 4.f;

    _mass += tetra_mass;

    // weighted sum
    center_of_mass += tetra_center_of_mass * tetra_mass;

    // inertia tensor of single tetrahedron
    const auto Tet3I = a * a + b * b + c * c + a * b + a * c + b * c;

    I +=
        raylib::Vector3{
            Tet3I.y + Tet3I.z, Tet3I.x + Tet3I.z, Tet3I.x + Tet3I.y
        } *
        determinant_J;

    Iprime +=
        raylib::Vector3{
            2.f * a.y * a.z + 2.f * b.y * b.z + 2.f * c.y * c.z + a.y * b.z +
                a.y * c.z + b.y * a.z + b.y * c.z + c.y * b.z + c.y * a.z,
            2.f * a.x * a.y + 2.f * b.x * b.y + 2.f * c.x * c.y + a.x * b.y +
                a.x * c.y + b.x * a.y + b.x * c.y + c.x * b.y + c.x * a.y,
            2.f * a.x * a.z + 2.f * b.x * b.z + 2.f * c.x * c.z + a.x * b.z +
                a.x * c.z + b.x * a.z + b.x * c.z + c.x * b.z + c.x * a.z
        } *
        determinant_J;
  }

  // Divide the weighted sum to get average
  center_of_mass /= _mass;

  // Scale tensor by factored constants
  I *= _density / 60.f;
  Iprime *= _density / 120.f;

  // Center of mass correction using Steiner's method
  const raylib::Vector3 com_distance{
      square(center_of_mass.y) + square(center_of_mass.z),
      square(center_of_mass.x) + square(center_of_mass.z),
      square(center_of_mass.x) + square(center_of_mass.y)
  };
  I -= com_distance * _mass;

  const raylib::Vector3 com_distance_prime{
      center_of_mass.y * center_of_mass.z,
      center_of_mass.x * center_of_mass.y,
      center_of_mass.x * center_of_mass.z
  };
  Iprime -= com_distance_prime * _mass;

  // Arrange the vector components into a matrix tensor
  const auto inertia_tensor = raylib::Matrix{
      I.x,
      -Iprime.y,
      -Iprime.z,
      0.f,
      -Iprime.y,
      I.y,
      -Iprime.x,
      0.f,
      -Iprime.z,
      -Iprime.x,
      I.z,
      0.f,
      0.f,
      0.f,
      0.f,
      1.f
  };

  _inverse_inertia_tensor = inertia_tensor.Invert();

  std::println("mass: {}, center of mass: {}", _mass, center_of_mass);
  std::println("inertia tensor: {}", inertia_tensor);
  std::println("inverse inertia tensor: {}", _inverse_inertia_tensor);

  if (center_of_mass.Length() >= EPSILON) {
    // Move mesh origin to the center of mass
    for (auto &vertex : mesh_vertices) {
      vertex -= center_of_mass;
    }
    for (auto &[a, b, c, n] : _faces) {
      a -= center_of_mass;
      b -= center_of_mass;
      c -= center_of_mass;
    }
  }
};
