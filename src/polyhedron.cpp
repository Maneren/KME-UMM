#include "polyhedron.hpp"

// const raylib::Matrix CANONICAL_TETRAHEDRON_INERTIA_TENSOR{
//     1.f, 0.5f, 0.5f, 0.f, 0.5f, 1.f, 0.5f, 0.f, 0.5f, 0.5f, 1.f, 0.f
// };

float Polyhedron::moment_of_inertia(const raylib::Vector3 &axis) {
  return axis.Transform(_inertia_tensor).DotProduct(axis);
}
