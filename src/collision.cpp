#include "collision.hpp"
#include <print>

std::optional<SupportPoint> find_support_point(
    raylib::Vector3 normal, raylib::Vector3 point, Polyhedron::Vertices vertices
) {
  float deepest_pen = 0.f;
  std::optional<SupportPoint> deepest_point;

  for (const auto &vertex : vertices) {
    const auto pen = -normal.DotProduct(vertex - point);
    if (pen > deepest_pen) {
      deepest_pen = pen;
      deepest_point = SupportPoint{vertex, pen};
    }
  }

  return deepest_point;
}

std::optional<CollisionManifold> find_collision_manifold(
    std::shared_ptr<Polyhedron> body_a, std::shared_ptr<Polyhedron> body_b
) {
  std::optional<CollisionManifold> manifold;

  float minimum_depth = std::numeric_limits<float>::max();

  for (const auto &[a, _b, _c, normal] : body_a->faces()) {
    const auto point = find_support_point(normal, a, body_b->vertices());

    if (!point)
      return std::nullopt;

    std::println(
        "for: {}, {}: point: {}, depth: {} | minimum depth: {}",
        a,
        normal,
        point->vertex,
        point->depth,
        minimum_depth
    );

    const auto depth = point->depth;
    if (depth < minimum_depth) {
      minimum_depth = depth;
      manifold = CollisionManifold{normal, point->vertex, depth};
    }
  }

  return manifold;
}

std::optional<CollisionManifold> detect_polyhedron_collision(
    std::shared_ptr<Polyhedron> body_a, std::shared_ptr<Polyhedron> body_b
) {
  const auto manifold_a = find_collision_manifold(body_a, body_b);
  if (!manifold_a)
    return std::nullopt;

  const auto manifold_b = find_collision_manifold(body_b, body_a);
  if (!manifold_b)
    return std::nullopt;

  std::println(
      "manifold_a: {}, manifold_b: {}", manifold_a.value(), manifold_b.value()
  );

  if (manifold_a->depth < manifold_b->depth) {
    return manifold_a;
  } else {
    return CollisionManifold{
        -manifold_b->normal, manifold_b->penetration_point, manifold_b->depth
    };
  }
}
