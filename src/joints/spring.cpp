#include "spring.hpp"
#include <Vector3.hpp>
#include <print>

void SpringJoint::update(const float delta) {
  const auto offset_a = _body_a->transform_point(_anchor_a);
  const auto offset_b = _body_b->transform_point(_anchor_b);

  const auto position_a = _body_a->position() + offset_a;
  const auto position_b = _body_b->position() + offset_b;

  // 𝕩 = b - a
  const auto connection = position_b - position_a;

  std::println("position_a: {}, position_b: {}", position_a, position_b);

  // x = |𝕩|
  const auto length = connection.Length();

  // Δx = x - x0
  const auto length_delta = length - relaxed_length;

  color = (length_delta >= 0.f) ? raylib::Color::Green()
                                : raylib::Color::DarkGreen();

  std::println("relaxed_length: {}, distance: {}", relaxed_length, length);

  // |F| = k Δx
  auto force_magnitude = stiffness * length_delta;

  std::println("force_magnitude: {}", force_magnitude);

  if (std::abs(force_magnitude) <= EPSILON)
    return;

  if (damping > 0.f && last_length >= 0.f) {
    // dx
    const auto length_delta = last_length - length;
    // v = dx / dt
    const auto velocity = length_delta / delta;

    // |F'| = |F| - μv
    const auto damped = force_magnitude - damping * velocity;

    std::println(
        "delta: {}, length_delta: {}, velocity: {}, force: {}, "
        "damping: {} -> {}",
        delta,
        length_delta,
        velocity,
        force_magnitude,
        damping * velocity,
        damped
    );

    force_magnitude = damped;
  }

  last_length = length;

  // F = |F| 𝕩 / |𝕩|
  const auto force_a = connection.Scale(force_magnitude / length);
  const auto force_b = -force_a;
  std::println("force_a: {}, force_b: {}", force_a, force_b);
  std::println();

  _body_a->apply_force(force_a, offset_a);
  _body_b->apply_force(force_b, offset_b);
}
