#pragma once

#include "polyhedron.hpp"
#include <Vector3.hpp>
#include <format>
#include <memory>
#include <optional>
#include <variant>

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

struct Contact {
  struct FaceVertex {
    raylib::Vector3 point, normal;
  };

  struct EdgeEdge {
    raylib::Vector3 edge_a, edge_b;
  };

  void resolve() const;

  bool is_face_vertex() const {
    return std::holds_alternative<FaceVertex>(data);
  }

  bool is_edge_edge() const { return std::holds_alternative<EdgeEdge>(data); }

  const FaceVertex &face_vertex() const { return std::get<FaceVertex>(data); }
  const EdgeEdge &edge_edge() const { return std::get<EdgeEdge>(data); }

  std::shared_ptr<Body> body_a, body_b;
  std::variant<FaceVertex, EdgeEdge> data;
  float depth;
};

template <> struct std::formatter<Contact::FaceVertex> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const Contact::FaceVertex &f, FormatContext &ctx) const {
    return std::format_to(ctx.out(), "FaceVertex({}, {})", f.point, f.normal);
  }
};

template <> struct std::formatter<Contact::EdgeEdge> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const Contact::EdgeEdge &e, FormatContext &ctx) const {
    return std::format_to(ctx.out(), "EdgeEdge({}, {})", e.edge_a, e.edge_b);
  }
};

template <> struct std::formatter<Contact> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const Contact &c, FormatContext &ctx) const {
    if (std::holds_alternative<Contact::FaceVertex>(c.data)) {
      return std::format_to(ctx.out(), "Contact::{}", c.face_vertex());
    } else if (std::holds_alternative<Contact::EdgeEdge>(c.data)) {
      return std::format_to(ctx.out(), "Contact::{}", c.edge_edge());
    } else {
      return std::format_to(ctx.out(), "Contact");
    }
  }
};

std::optional<SupportPoint> find_support_point(
    raylib::Vector3 normal, raylib::Vector3 point, Polyhedron::Vertices vertices
);

std::optional<CollisionManifold> find_collision_manifold(
    std::shared_ptr<Polyhedron> body_a, std::shared_ptr<Polyhedron> body_b
);

std::optional<Contact> detect_polyhedron_collision(
    std::shared_ptr<Polyhedron> body_a, std::shared_ptr<Polyhedron> body_b
);
