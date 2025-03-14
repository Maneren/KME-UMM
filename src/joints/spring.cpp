#include "spring.hpp"
#include <print>

void SpringJoint::update(const float) {
  const auto offset_a = _body_a->transform_offset(_anchor_a);
  const auto offset_b = _body_b->transform_offset(_anchor_b);

  const auto position_a = _body_a->position() + offset_a;
  const auto position_b = _body_b->position() + offset_b;

  const auto connection = position_b - position_a;

  const auto distance = connection.Length();
  const auto rest_distance = distance - relaxed_length;

  const auto force_magnitude = rest_distance / relaxed_length * strength;

  const auto force_a = connection.Scale(force_magnitude / distance);
  const auto force_b = -force_a;

  _body_a->apply_force(force_a, offset_a);
  _body_b->apply_force(force_b, offset_b);
}
