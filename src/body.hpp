#pragma once

#include "consts.hpp"
#include "defs.hpp"
#include "object.hpp"
#include "ode.hpp"
#include "utils.hpp"
#include <Image.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <Model.hpp>
#include <Texture.hpp>
#include <Vector3.hpp>
#include <Vector4.hpp>
#include <cmath>
#include <print>
#include <raymath.h>

class Body : public Object {
public:
  Body(const raylib::Vector3 &position, const raylib::Quaternion &orientation)
      : _position(position), _orientation(orientation) {}

  ~Body() {
    if (_texture.IsValid())
      _texture.Unload();

    if (_mesh.IsValid())
      _mesh.Unload();
  };

  Body &position(const raylib::Vector3 &position) {
    this->_position = position;
    return *this;
  }
  Body &linear_momentum(const raylib::Vector3 &linear_momentum) {
    this->_linear_momentum = linear_momentum;
    update_velocity();
    return *this;
  }
  Body &orientation(const raylib::Quaternion &orientation) {
    this->_orientation = orientation;
    const auto R = _orientation.ToMatrix();
    _model.transform = R;
    update_inverse_inertia_tensor();
    update_angular_velocity();
    return *this;
  }
  Body &angular_momentum(const raylib::Vector3 &angular_momentum) {
    this->_angular_momentum = angular_momentum;
    update_angular_velocity();
    return *this;
  }
  Body &texture(raylib::Texture &&texture) {
    this->_texture = std::move(texture);
    return *this;
  }

  struct State {
    const float inverse_mass;
    const raylib::Matrix inverse_inertia_tensor;
    const raylib::Vector3 net_force, net_torque;

    raylib::Vector3 position, linear_momentum;
    raylib::Quaternion orientation;
    raylib::Vector3 angular_momentum;

    const raylib::Vector3 velocity() const {
      return linear_momentum * inverse_mass;
    }
    const raylib::Vector3 angular_velocity() const {
      return angular_momentum.Transform(inverse_inertia_tensor);
    }

    // State operator-(const State &other) const {
    //   return {
    //       .inverse_mass = inverse_mass,
    //       .inverse_inertia_tensor = inverse_inertia_tensor,
    //       .net_force = net_force - other.net_force,
    //       .net_torque = net_torque - other.net_torque,
    //       .position = position - other.position,
    //       .linear_momentum = linear_momentum - other.linear_momentum,
    //       .orientation = orientation - other.orientation,
    //       .angular_momentum = angular_momentum - other.angular_momentum
    //   };
    // };
  };

  State state() {
    return {
        .inverse_mass = inverse_mass(),
        .inverse_inertia_tensor = inverse_inertia_tensor(),
        .position = position(),
        .linear_momentum = linear_momentum(),
        .orientation = orientation(),
        .angular_momentum = angular_momentum(),
    };
  }
  void state(const State &state) {
    _position = state.position;
    _linear_momentum = state.linear_momentum;
    _orientation = state.orientation;
    _angular_momentum = state.angular_momentum;
  }

  float mass() const { return 1.f / _inverse_mass; }
  float inverse_mass() const { return _inverse_mass; }

  const raylib::Matrix &inverse_inertia_tensor() const {
    return _inverse_inertia_tensor;
  }

  raylib::Vector3 &position() { return _position; };
  raylib::Vector3 &linear_momentum() { return _linear_momentum; }
  const raylib::Vector3 &velocity() const { return _velocity; }
  const raylib::Vector3 &force() const { return _net_force; }

  raylib::Quaternion &orientation() { return _orientation; }
  raylib::Vector3 &angular_momentum() { return _angular_momentum; }
  const raylib::Vector3 &angular_velocity() const { return _angular_velocity; }
  const raylib::Vector3 &torque() const { return _net_torque; }

  const raylib::Vector3 point_velocity(const raylib::Vector3 &point) const {
    return _velocity + _angular_velocity.CrossProduct(point);
  }

  const raylib::Texture &texture() const { return _texture; }

  virtual void initialize() override {
    raylib::Image image = raylib::Image::Color(1, 1, _color);
    _texture = raylib::Texture(image);

    _mesh = get_mesh();
    _model.Load(_mesh);
    _model.transform = _orientation.ToMatrix();
    _model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = _texture;

    update_velocity();
    update_angular_velocity();
    update_inverse_inertia_tensor();
  }

  virtual void draw() const override {
    _model.Draw(_position, 1.0f, raylib::Color::White());
    _model.DrawWires(_position, 1.0f, raylib::Color::Black());
  }

  raylib::Vector3 transform_point(const raylib::Vector3 &point) const {
    return point.Transform(_model.transform);
  }

  virtual void apply_force(
      const raylib::Vector3 &force,
      raylib::Vector3 offset = raylib::Vector3::Zero()
  ) {
    body_apply_force(force, offset);
  };
  virtual void update(const float delta) override { body_update(delta); };

protected:
  raylib::Vector3 _position;
  raylib::Vector3 _linear_momentum = raylib::Vector3::Zero();
  raylib::Vector3 _net_force = raylib::Vector3::Zero();

  raylib::Quaternion _orientation;
  raylib::Vector3 _angular_momentum = raylib::Vector3::Zero();
  raylib::Vector3 _net_torque = raylib::Vector3::Zero();

  raylib::Model _model;
  raylib::MeshUnmanaged _mesh;
  raylib::Color _color = raylib::Color::White();
  raylib::Texture2D _texture;

  // cached
  raylib::Vector3 _velocity = raylib::Vector3::Zero();
  raylib::Vector3 _angular_velocity = raylib::Vector3::Zero();
  raylib::Matrix _inverse_inertia_tensor = raylib::Matrix::Identity();
  float _inverse_mass = 1.0f;

  virtual raylib::MeshUnmanaged get_mesh() = 0;
  virtual const raylib::Matrix &inverse_body_inertia_tensor() const = 0;

  void update_velocity() {
    // 𝐯 = 𝐏 / m
    _velocity = _linear_momentum * _inverse_mass;
  }
  void update_angular_velocity() {
    // 𝛚 = 𝐈⁻¹𝐋
    _angular_velocity = _angular_momentum.Transform(_inverse_inertia_tensor);
  }
  void update_inverse_inertia_tensor() {
    const raylib::Matrix &R = _model.transform;
    // 𝐈⁻¹ = 𝐑 𝐈⁻¹₀ 𝐑ᵀ
    _inverse_inertia_tensor = R * inverse_body_inertia_tensor() * R.Transpose();
  }

  void body_apply_force(
      const raylib::Vector3 &force, const raylib::Vector3 &offset
  ) {
    // std::println("Applied force: {} at offset: {}", force, offset);

    // ignore very small forces
    if (force.Length() <= EPSILON)
      return;

    _net_force += force;

    // only consider off center forces for torque
    if (offset.Length() <= EPSILON)
      return;

    // 𝛕 = 𝐫 × 𝐅
    const auto torque =
        transform_point(offset).CrossProduct(transform_point(force));

    // std::println("Resulting in torque: {}", torque);

    _net_torque += torque;
    // std::println();
  };

  static State state_change(const float delta, const State &state) {
    auto new_state = state;

    // dP = F ⋅ dt
    new_state.linear_momentum = state.linear_momentum + state.net_force * delta;
    // dx = v ⋅ dt
    new_state.position = state.position + new_state.velocity() * delta;

    // dL = τ ⋅ dt
    new_state.angular_momentum =
        state.angular_momentum + state.net_torque * delta;
    // dq = ω ⋅ dt
    new_state.orientation =
        (state.orientation *
         angular_velocity_to_rotation(new_state.angular_velocity(), delta))
            .Normalize();

    return new_state;
  }

  void body_update(const float delta) {
    update_position(delta);
    update_orientation(delta);

    // std::println(
    //     "linear_momentum: {} kg m s⁻¹, angular_momentum: {} kg m² s⁻¹",
    //     _linear_momentum.Length(),
    //     _angular_momentum.Length()
    // );

    // // Eₖ = |𝐏|² / 2m (equivalent to Eₖ = ½ m |𝐯|², where 𝐯 = 𝐏 / m)
    // const auto linear_energy =
    //     _linear_momentum.LengthSqr() * 0.5f * _inverse_mass;
    //
    // // Eₘ = ½ 𝐋 ⋅ 𝛚 (equivalent to Eₘ = ½ 𝐈𝛚², since 𝛚 = 𝐈⁻¹𝐋)
    // const auto angular_energy =
    //     _angular_velocity.DotProduct(_angular_momentum) * 0.5f;
    //
    //   std::println(
    //       "linear_energy: {} J + angular_energy: {} J = {} J",
    //       linear_energy,
    //       angular_energy,
    //       linear_energy + angular_energy
    //   );
    //   std::println();
  };

private:
  void update_position(const float delta) {
    // d𝐏 = 𝐅 ⋅ dt
    _linear_momentum += _net_force * delta;

    // Friction and other environmental forces
    // 𝐏' = (1 - μ)ᵈᵗ 𝐏
    _linear_momentum *= std::pow(1.f - ENVIRONMENT_FRICTION_COEFFICIENT, delta);

    update_velocity();

    // d𝐱 = 𝐯 ⋅ dt
    _position += _velocity * delta;

    // std::println(
    //     "position: {}, linear_momentum: {}, velocity: {}",
    //     _position,
    //     _linear_momentum,
    //     _velocity
    // );

    _net_force = raylib::Vector3::Zero();
    // std::println();
  }

  void update_orientation(const float delta) {
    // d𝐋 = 𝛕 dt
    _angular_momentum += _net_torque * delta;

    // Friction and other environmental forces
    // 𝐋' = (1 - μ)ᵈᵗ 𝐋
    _angular_momentum *=
        std::pow(1.f - ENVIRONMENT_FRICTION_COEFFICIENT, delta);

    update_angular_velocity();

    // std::println(
    //     "torque: {}, angular_momentum: {}, angular_velocity: {}",
    //     _net_torque,
    //     _angular_momentum,
    //     _angular_velocity
    // );

    _net_torque = raylib::Vector3::Zero();

    // ignore very small angular velocities
    if (_angular_velocity.Length() <= EPSILON)
      return;

    const auto delta_q = angular_velocity_to_rotation(_angular_velocity, delta);

    // q' = q * dq
    const auto rotated = (_orientation * delta_q).Normalize();

    // std::println("orientation: {}, rotation: {}", _orientation, delta_q);
    // std::println("rotated: {}", rotated);
    // std::println();

    _orientation = rotated;
    _model.transform = rotated.ToMatrix();

    update_inverse_inertia_tensor();
  }
};
