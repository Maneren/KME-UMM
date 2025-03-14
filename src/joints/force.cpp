#include "force.hpp"
#include <print>

void ForceJoint::update(const float) {
  const auto offset_a = _body_a->transform_offset(_anchor_a);
  const auto offset_b = _body_b->transform_offset(_anchor_b);

  const auto position_a = _body_a->position() + offset_a;
  const auto position_b = _body_b->position() + offset_b;

  if (Vector3DistanceSqr(position_a, position_b) <= EPSILON) {
    return;
  }

  const auto direction_a = position_b - position_a;
  const auto direction_b = position_a - position_b;

  const auto direction_a_length = direction_a.Length();
  const auto direction_b_length = direction_b.Length();

  const auto force_a = direction_a.Scale(strength / direction_a_length);
  const auto force_b = direction_b.Scale(strength / direction_b_length);

  _body_a->apply_force(force_a, offset_a);
  _body_b->apply_force(force_b, offset_b);
}
