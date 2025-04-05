#pragma once

#include "polyhedron.hpp"
#include <Vector3.hpp>
#include <memory>
#include <optional>

struct SupportPoint {
public:
  raylib::Vector3 vertex;
  float depth;
};

template <> struct std::formatter<SupportPoint> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const SupportPoint &s, FormatContext &ctx) const {
    return std::format_to(ctx.out(), "Support({}, {})", s.vertex, s.depth);
  }
};

struct CollisionManifold {
public:
  raylib::Vector3 normal;
  raylib::Vector3 penetration_point;
  float depth;
};

template <> struct std::formatter<CollisionManifold> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const CollisionManifold &c, FormatContext &ctx) const {
    return std::format_to(
        ctx.out(),
        "Manifold({}, {}, {})",
        c.normal,
        c.penetration_point,
        c.depth
    );
  }
};

std::optional<SupportPoint> find_support_point(
    raylib::Vector3 normal, raylib::Vector3 point, Polyhedron::Vertices vertices
);

std::optional<CollisionManifold> find_collision_manifold(
    std::shared_ptr<Polyhedron> body_a, std::shared_ptr<Polyhedron> body_b
);

std::optional<CollisionManifold> detect_polyhedron_collision(
    std::shared_ptr<Polyhedron> body_a, std::shared_ptr<Polyhedron> body_b
);
