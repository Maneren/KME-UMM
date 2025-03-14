#pragma once

#include "../joint.hpp"
#include <memory>

class ForceJoint final : public Joint {
public:
  ForceJoint(
      std::shared_ptr<Body> body_a,
      std::shared_ptr<Body> body_b,
      raylib::Color color,
      const float strength
  )
      : Joint(body_a, body_b, color), strength(strength) {}

  void update(const float dt) override;

protected:
  float strength;
};
