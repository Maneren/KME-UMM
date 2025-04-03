#pragma once

#include <Matrix.hpp>
#include <Color.hpp>
#include <Vector2.hpp>
#include <Vector3.hpp>
#include <Vector4.hpp>
#include <format>

template <> struct std::formatter<raylib::Vector2> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const raylib::Vector2 &c, FormatContext &ctx) const {
    return std::format_to(ctx.out(), "({}, {})", c.x, c.y);
  }
};

template <> struct std::formatter<raylib::Vector3> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const raylib::Vector3 &c, FormatContext &ctx) const {
    return std::format_to(ctx.out(), "({}, {}, {})", c.x, c.y, c.z);
  }
};

template <> struct std::formatter<raylib::Vector4> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const raylib::Vector4 &c, FormatContext &ctx) const {
    return std::format_to(ctx.out(), "({}, {}, {}, {})", c.x, c.y, c.z, c.w);
  }
};

template <> struct std::formatter<raylib::Color> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const raylib::Color &c, FormatContext &ctx) const {
    return std::format_to(ctx.out(), "({}, {}, {}, {})", c.r, c.g, c.b, c.a);
  }
};

template <> struct std::formatter<raylib::Matrix> {
  constexpr auto parse(std::format_parse_context const &ctx) const {
    return ctx.begin();
  }
  template <typename FormatContext>
  auto format(const raylib::Matrix &c, FormatContext &ctx) const {
    return std::format_to(
        ctx.out(),
        "({}, {}, {}, {} | {}, {}, {}, {} | {}, {}, {}, {} | {}, {}, {}, {})",
        c.m0,
        c.m1,
        c.m2,
        c.m3,
        c.m4,
        c.m5,
        c.m6,
        c.m7,
        c.m8,
        c.m9,
        c.m10,
        c.m11,
        c.m12,
        c.m13,
        c.m14,
        c.m15
    );
  }
};
