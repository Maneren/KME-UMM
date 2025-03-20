#include "spring.hpp"
#include <Vector3.hpp>
#include <print>

void SpringJoint::update(const float delta) {
  const auto offset_a = _body_a->transform_offset(_anchor_a);
  const auto offset_b = _body_b->transform_offset(_anchor_b);

  const auto position_a = _body_a->position() + offset_a;
  const auto position_b = _body_b->position() + offset_b;

  const auto connection = position_b - position_a;

  std::println("position_a: {}, position_b: {}", position_a, position_b);
  const auto length = connection.Length();
  const auto extra_length = length - relaxed_length;

  std::println("relaxed_length: {}, distance: {}", relaxed_length, length);
  auto force_magnitude = extra_length / relaxed_length * strength;

  if (damping > 0.f && last_length >= 0.f) {
    const auto length_delta = last_length - length;
    const auto velocity_magnitude = length_delta / delta;

    std::println(
        "delta: {}, length_delta: {}, magnitude: {}, force: {}, "
        "damping: {}",
        delta,
        length_delta,
        velocity_magnitude,
        force_magnitude,
        damping * velocity_magnitude
    );
    force_magnitude -= damping * velocity_magnitude;
  }

  last_length = length;

  const auto force_a = connection.Scale(force_magnitude / length);
  const auto force_b = -force_a;
  std::println("force_a: {}, force_b: {}", force_a, force_b);

  _body_a->apply_force(force_a, offset_a);
  _body_b->apply_force(force_b, offset_b);
}
