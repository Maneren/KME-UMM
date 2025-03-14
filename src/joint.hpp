#pragma once

#include "body.hpp"
#include "object.hpp"
#include <Color.hpp>
#include <Vector3.hpp>
#include <memory>

class Joint : public Object {
public:
  Joint(
      std::shared_ptr<Body> body_a,
      std::shared_ptr<Body> body_b,
      raylib::Color color
  )
      : _body_a(body_a), _body_b(body_b), color(color) {}

  Joint &anchor_a(const raylib::Vector3 &anchor) {
    _anchor_a = anchor;
    return *this;
  }
  Joint &anchor_b(const raylib::Vector3 &anchor) {
    _anchor_b = anchor;
    return *this;
  }

  const raylib::Vector3 &anchor_a() const { return _anchor_a; }
  const raylib::Vector3 &anchor_b() const { return _anchor_b; }

  void draw() const override {
    const auto offset_a = _body_a->transform_offset(_anchor_a);
    const auto offset_b = _body_b->transform_offset(_anchor_b);

    const auto position_a = _body_a->position() + offset_a;
    const auto position_b = _body_b->position() + offset_b;

    position_a.DrawLine3D(position_b, color);
    position_a.DrawSphere(0.1f, color);
    position_b.DrawSphere(0.1f, color);
  };

protected:
  std::shared_ptr<Body> _body_a;
  std::shared_ptr<Body> _body_b;

  raylib::Color color;

  raylib::Vector3 _anchor_a = raylib::Vector3::Zero();
  raylib::Vector3 _anchor_b = raylib::Vector3::Zero();
};
