#include "spring.hpp"
#include <print>

void SpringJoint::update(const float) {
  const auto position_a = _body_a->position() + _anchor_a;
  const auto position_b = _body_b->position() + _anchor_b;
  const auto center = (position_a + position_b) / 2.f;

  const auto distance = position_a.Distance(position_b);
  const auto rest_distance = distance - relaxed_length;

  const auto force_magnitude =
      rest_distance * relaxed_length * strength * falloff;

  const auto direction_a = center - position_a;
  const auto direction_b = center - position_b;

  const auto direction_a_length = direction_a.Length();
  const auto direction_b_length = direction_b.Length();

  const auto force_a = direction_a.Scale(force_magnitude / direction_a_length);
  const auto force_b = direction_b.Scale(force_magnitude / direction_b_length);

  _body_a->apply_force(force_a, _anchor_a);
  _body_b->apply_force(force_b, _anchor_b);
}
