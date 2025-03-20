#pragma once

#include "../joint.hpp"
#include <memory>

class SpringJoint final : public Joint {
public:
  SpringJoint(
      std::shared_ptr<Body> body_a,
      std::shared_ptr<Body> body_b,
      raylib::Color color,
      const float strength,
      const float relaxed_length,
      const float damping = 1.0f
  )
      : Joint(body_a, body_b, color), strength(strength),
        relaxed_length(relaxed_length), damping(damping) {}

  void update(const float dt) override;

protected:
  float strength;
  float relaxed_length;
  float last_length = -1.f;
  float damping = 1.0f;
};
