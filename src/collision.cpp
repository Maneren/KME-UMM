#include "collision.hpp"
#include <print>

std::optional<SupportPoint> find_support_point(
    raylib::Vector3 normal,
    raylib::Vector3 point,
    std::shared_ptr<Polyhedron> body
) {
  float max_depth = 0.f;
  std::optional<SupportPoint> deepest_point;

  for (const auto &vertex : body->vertices()) {
    const auto world_vertex = body->position() + body->transform_point(vertex);
    const auto depth = -normal.DotProduct(world_vertex - point);
    if (depth > max_depth) {
      max_depth = depth;
      deepest_point = SupportPoint{vertex, depth};
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
    const auto world_a = body_a->position() + body_a->transform_point(a);
    const auto world_normal = body_a->transform_point(normal);

    const auto point = find_support_point(world_normal, world_a, body_b);

    if (!point)
      return std::nullopt;

    const auto depth = point->depth;
    // const auto vertex = point->vertex;

    // std::println(
    //     "for: {} ({}), {} ({}): point: {}, depth: {} | minimum: {}",
    //     a,
    //     world_a,
    //     normal,
    //     world_normal,
    //     vertex,
    //     depth,
    //     minimum_depth
    // );

    if (depth < minimum_depth) {
      minimum_depth = depth;
      manifold = CollisionManifold{world_normal, point->vertex, depth};
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
