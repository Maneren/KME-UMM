#include "body.hpp"
#include "consts.hpp"
#include "utils.hpp"

raylib::Vector3 Body::transform_point(const raylib::Vector3 &point) const {
  // 𝐑 𝐱
  return point.Transform(_model.transform);
}

raylib::Vector3 Body::point_velocity(const raylib::Vector3 &point) const {
  // 𝐯 + 𝛚 × 𝐱
  return _velocity + _angular_velocity.CrossProduct(point);
}

void Body::update_velocity() {
  // 𝐯 = 𝐏 / m
  _velocity = _linear_momentum * _inverse_mass;
}
void Body::update_angular_velocity() {
  // 𝛚 = 𝐈⁻¹𝐋
  _angular_velocity = _angular_momentum.Transform(_inverse_inertia_tensor);
}
void Body::update_inverse_inertia_tensor() {
  const raylib::Matrix &R = _model.transform;
  // 𝐈⁻¹ = 𝐑 𝐈⁻¹₀ 𝐑ᵀ
  _inverse_inertia_tensor = R * inverse_body_inertia_tensor() * R.Transpose();
}

void Body::body_apply_force(
    const raylib::Vector3 &force, const raylib::Vector3 &offset
) {
  // ignore small forces
  if (force.Length() <= EPSILON)
    return;

  _net_force += force;

  // only consider off center forces for torque
  if (offset.Length() <= EPSILON)
    return;

  // 𝛕 = 𝐫 × 𝐅
  const auto torque =
      transform_point(offset).CrossProduct(transform_point(force));

  _net_torque += torque;
};

void Body::body_update(const float delta) {
  update_position(delta);
  update_orientation(delta);
};

void Body::update_position(const float delta) {
  // d𝐏 = 𝐅 ⋅ dt
  _linear_momentum += _net_force * delta;
  _net_force = raylib::Vector3::Zero();

  // Friction and other environmental forces
  // 𝐏' = (1 - μ)ᵈᵗ 𝐏
  _linear_momentum *= std::pow(1.f - ENVIRONMENT_FRICTION_COEFFICIENT, delta);

  update_velocity();

  // d𝐱 = 𝐯 ⋅ dt
  _position += _velocity * delta;
}

void Body::update_orientation(const float delta) {
  // d𝐋 = 𝛕 dt
  _angular_momentum += _net_torque * delta;
  _net_torque = raylib::Vector3::Zero();

  // Friction and other environmental forces
  // 𝐋' = (1 - μ)ᵈᵗ 𝐋
  _angular_momentum *= std::pow(1.f - ENVIRONMENT_FRICTION_COEFFICIENT, delta);

  update_angular_velocity();

  // ignore small angular velocities
  if (_angular_velocity.Length() <= EPSILON)
    return;

  const auto delta_q = angular_velocity_to_rotation(_angular_velocity, delta);

  // q' = q * dq
  const auto rotated = (_orientation * delta_q).Normalize();

  _orientation = rotated;
  _model.transform = rotated.ToMatrix();

  update_inverse_inertia_tensor();
}
