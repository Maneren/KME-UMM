#include "spring.hpp"
#include <print>

void SpringJoint::update(const float) {
  const auto offset_a = _body_a->transform_point(_anchor_a);
  const auto offset_b = _body_b->transform_point(_anchor_b);

  const auto position_a = _body_a->position() + offset_a;
  const auto position_b = _body_b->position() + offset_b;

  // Hooke's law: F = k (|𝐱| - x₀) - μvᵣ

  // 𝐱 = 𝐛 - 𝐚
  const auto connection = position_b - position_a;

  // x = |𝐱|
  const auto length = connection.Length();

  // 𝐱' = 𝐱 / |𝐱|
  const auto direction = connection.Scale(1.f / length);

  // vᵣ = (𝐯₂ - 𝐯₁) ⋅ 𝐱'
  const auto relative_velocity =
      (_body_b->velocity() - _body_a->velocity()).DotProduct(direction);

  // Δx = x - x₀
  const auto length_delta = length - relaxed_length;

  if (std::abs(length_delta) <= 10e-3f)
    color = raylib::Color::Blue();
  else if (length_delta > 0.f)
    color = raylib::Color::Green();
  else
    color = raylib::Color::DarkGreen();

  // |F| = k Δx
  auto force_magnitude = stiffness * length_delta;

  // ignore small forces
  if (std::abs(force_magnitude) <= EPSILON)
    return;

  if (damping > 0.f) {
    // d = μvᵣ
    const auto damping_magnitude = damping * relative_velocity;
    // |F'| = |F| - μv
    const auto damped = force_magnitude - damping_magnitude;

    force_magnitude = damped;
  }

  // F = |F| 𝐱'
  const auto force_a = direction * force_magnitude;
  const auto force_b = -force_a;

  _body_a->apply_force(force_a, offset_a);
  _body_b->apply_force(force_b, offset_b);
}

float SpringJoint::potential_energy() const {
  const auto offset_a = _body_a->transform_point(_anchor_a);
  const auto offset_b = _body_b->transform_point(_anchor_b);

  const auto position_a = _body_a->position() + offset_a;
  const auto position_b = _body_b->position() + offset_b;

  // 𝐱 = 𝐛 - 𝐚
  const auto connection = position_b - position_a;

  // x = |𝐱|
  const auto length = connection.Length();

  // Δx = x - x₀
  const auto length_delta = length - relaxed_length;

  // U = ½ k Δx²
  return 0.5f * stiffness * length_delta * length_delta;
}
