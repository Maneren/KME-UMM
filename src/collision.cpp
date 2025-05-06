#include "collision.hpp"
#include "consts.hpp"
#include "defs.hpp"
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
      deepest_point = SupportPoint{world_vertex, depth};
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

    if (depth < minimum_depth) {
      minimum_depth = depth;
      manifold = CollisionManifold{world_normal, point->vertex, depth};
    }
  }

  return manifold;
}

std::optional<Contact> detect_polyhedron_collision(
    std::shared_ptr<Polyhedron> body_a, std::shared_ptr<Polyhedron> body_b
) {
  const auto manifold_a = find_collision_manifold(body_a, body_b);
  if (!manifold_a)
    return std::nullopt;

  const auto manifold_b = find_collision_manifold(body_b, body_a);
  if (!manifold_b)
    return std::nullopt;

  if (manifold_a->depth < manifold_b->depth) {
    return Contact{
        body_a,
        body_b,
        Contact::FaceVertex{manifold_a->penetration_point, manifold_a->normal},
        manifold_a->depth
    };
  } else {
    return Contact{
        body_b,
        body_a,
        Contact::FaceVertex{
            manifold_b->penetration_point, manifold_b->normal

        },
        manifold_b->depth
    };
  }
}

void Contact::resolve() const {
  if (!is_face_vertex())
    throw std::runtime_error("not a face vertex collision");

  const auto &data = face_vertex();

  const auto point_a = data.point - body_a->position();
  const auto point_b = data.point - body_b->position();

  const auto point_a_velocity = body_a->point_velocity(point_a);
  const auto point_b_velocity = body_b->point_velocity(point_b);

  const auto &normal = data.normal;

  // vᵣ = (𝐯₂ - 𝐯₁) ⋅ 𝐧
  const auto relative_velocity =
      (point_b_velocity - point_a_velocity).DotProduct(normal);

  if (relative_velocity >= -EPSILON) {
    // moving apart already
    return;
  }

  std::println("relative_velocity: {}, depth: {}", relative_velocity, depth);

  const auto numerator =
      -(1.f + COEFFICIENT_OF_RESTITUTION) * relative_velocity;

  const auto inverse_masses = body_a->inverse_mass() + body_b->inverse_mass();

  const auto &inverse_inertia_tensor_a = body_a->inverse_inertia_tensor();
  const auto &inverse_inertia_tensor_b = body_b->inverse_inertia_tensor();

  // 𝐨 = 𝐫 × 𝐧
  const auto axis_a = point_a.CrossProduct(normal);
  const auto axis_b = point_b.CrossProduct(normal);

  // 𝐈⁻¹𝐨 × 𝐫
  const auto angular_a =
      axis_a.Transform(inverse_inertia_tensor_a).CrossProduct(point_a);
  const auto angular_b =
      axis_b.Transform(inverse_inertia_tensor_b).CrossProduct(point_b);

  const auto denominator =
      inverse_masses + (angular_a + angular_b).DotProduct(normal);

  std::println("numerator: {}", numerator);
  std::println(
      "denominator: {} ({} + {} + {})",
      denominator,
      inverse_masses,
      angular_a,
      angular_b
  );
  // j
  const auto impulse_magnitude = numerator / denominator;

  std::println("impulse: {}", impulse_magnitude);

  // 𝐣 = j𝐧
  const auto impulse = normal * impulse_magnitude;

  // 𝐏' = 𝐏 - 𝐣
  body_a->linear_momentum() -= impulse;
  body_b->linear_momentum() += impulse;

  // 𝐋' = 𝐋 - j𝐨
  body_a->angular_momentum() -= axis_a * impulse_magnitude;
  body_b->angular_momentum() += axis_b * impulse_magnitude;
}
