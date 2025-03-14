#pragma once

class BodyMaterial final {
public:
  BodyMaterial(float static_friction, float dynamic_friction)
      : STATIC_FRICTION(static_friction), DYNAMIC_FRICTION(dynamic_friction) {}

  constexpr float static_friction() const { return STATIC_FRICTION; }
  constexpr float dynamic_friction() const { return DYNAMIC_FRICTION; }

  BodyMaterial operator+(const BodyMaterial &other) const {
    return BodyMaterial(
        static_friction() + other.static_friction(),
        dynamic_friction() + other.dynamic_friction()
    );
  }

private:
  const float STATIC_FRICTION;
  const float DYNAMIC_FRICTION;
};
