#pragma once

#include "consts.hpp"
#include "defs.hpp"
#include "object.hpp"
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
  Body &velocity(const raylib::Vector3 &velocity) {
    this->_linear_momentum = velocity;
    return *this;
  }
  Body &acceleration(const raylib::Vector3 &acceleration) {
    this->_net_force = acceleration;
    return *this;
  }
  Body &mass(const float mass) {
    this->_mass = mass;
    return *this;
  }
  Body &orientation(const raylib::Quaternion &orientation) {
    this->_orientation = orientation;
    _model.transform = _orientation.ToMatrix();
    return *this;
  }
  Body &angular_acceleration(const raylib::Vector3 &angular_acceleration) {
    this->_net_torque = angular_acceleration;
    return *this;
  }
  Body &texture(raylib::Texture &&texture) {
    this->_texture = std::move(texture);
    return *this;
  }

  float mass() const { return _mass; }

  const raylib::Vector3 &position() const { return _position; };
  const raylib::Vector3 &linear_momentum() const { return _linear_momentum; }
  const raylib::Vector3 velocity() const {
    // v = P / m
    return _linear_momentum / _mass;
  }
  const raylib::Vector3 &force() const { return _net_force; }

  const raylib::Quaternion &orientation() const { return _orientation; }
  const raylib::Vector3 &angular_momentum() const { return _angular_momentum; }
  const raylib::Vector3 angular_velocity() const {
    // ω = I⁻¹L
    return _angular_momentum.Transform(inverse_inertia_tensor());
  }
  const raylib::Vector3 &torque() const { return _net_torque; }

  const raylib::Texture &texture() const { return _texture; }

  virtual void initialize() override {
    raylib::Image image = raylib::Image::Color(1, 1, _color);
    _texture = raylib::Texture(image);

    _mesh = get_mesh();
    _model.Load(_mesh);
    _model.transform = _orientation.ToMatrix();
    _model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = _texture;
  }

  virtual void draw() const override {
    _model.Draw(_position, 1.0f, raylib::Color::White());
    _model.DrawWires(_position, 1.0f, raylib::Color::Black());

    // _position.DrawLine3D(
    //     _position + angular_momentum() * 10, raylib::Color::Green()
    // );
    // _position.DrawLine3D(
    //     _position + transform_point(angular_velocity() * 500),
    //     raylib::Color::Blue()
    // );
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

  float _mass = 1.0f;

  raylib::Quaternion _orientation;
  raylib::Vector3 _angular_momentum = raylib::Vector3::Zero();
  raylib::Vector3 _net_torque = raylib::Vector3::Zero();

  raylib::Model _model;
  raylib::MeshUnmanaged _mesh;
  raylib::Color _color = raylib::Color::White();
  raylib::Texture2D _texture;

  virtual raylib::MeshUnmanaged get_mesh() = 0;

  virtual const raylib::Matrix &inverse_body_inertia_tensor() const = 0;

  const raylib::Matrix inverse_inertia_tensor() const {
    // I⁻¹ = RI₀⁻¹Rᵀ
    const raylib::Matrix R = _model.transform;
    return R * inverse_body_inertia_tensor() * R.Transpose();
  }

  void body_apply_force(
      const raylib::Vector3 &force, const raylib::Vector3 &offset
  ) {
    std::println("Applied force: {} at offset: {}", force, offset);

    // ignore very small forces
    if (force.Length() <= EPSILON)
      return;

    _net_force += force;

    // only consider off center forces for torque
    if (offset.Length() <= EPSILON)
      return;

    // τ = r × F
    const auto torque =
        transform_point(offset).CrossProduct(transform_point(force));

    std::println("Resulting in torque: {}", torque);

    _net_torque += torque;
    std::println();
  };

  void body_update(const float delta) {
    update_position(delta);
    update_orientation(delta);

    std::println(
        "linear_momentum: {} kg m s⁻¹, angular_momentum: {} kg m² s⁻¹",
        _linear_momentum.Length(),
        _angular_momentum.Length()
    );

    // Eₖ = |P|² / 2m (equivalent to Eₖ = ½ m |v|², where v = P / m)
    const auto linear_energy = _linear_momentum.LengthSqr() / (2.0f * _mass);

    // Eₘ = ½ L ⋅ ω (equivalent to Eₘ = ½Iω², since ω = I⁻¹L)
    const auto angular_velocity = this->angular_velocity();
    const auto angular_energy =
        angular_velocity.DotProduct(_angular_momentum) * 0.5f;

    std::println(
        "linear_energy: {} J + angular_energy: {} J = {}",
        linear_energy,
        angular_energy,
        linear_energy + angular_energy
    );
    std::println();
  };

private:
  void update_position(const float delta) {
    // dP = F ⋅ dt
    _linear_momentum += _net_force * delta;

    // Friction and other environmental forces
    // P' = (1 - μ)ᵈᵗ P
    _linear_momentum *= std::pow(1.f - ENVIRONMENT_FRICTION_COEFFICIENT, delta);

    const auto velocity = this->velocity();

    // dx = v ⋅ dt
    _position += velocity * delta;
    std::println(
        "position: {}, linear_momentum: {}, velocity: {}",
        _position,
        _linear_momentum,
        velocity
    );

    _net_force = raylib::Vector3::Zero();
    std::println();
  }

  void update_orientation(const float delta) {
    // dL = τ dt
    _angular_momentum += _net_torque * delta;

    // Friction and other environmental forces
    // L' = (1 - μ)ᵈᵗ L
    _angular_momentum *=
        std::pow(1.f - ENVIRONMENT_FRICTION_COEFFICIENT, delta);

    // ω = I⁻¹L
    auto omega = this->angular_velocity();

    std::println(
        "torque: {}, angular_momentum: {}, angular_velocity: {}",
        _net_torque,
        _angular_momentum,
        omega
    );

    _net_torque = raylib::Vector3::Zero();

    // ignore very small angular velocities
    if (omega.Length() <= EPSILON)
      return;

    // dq = ½ ⋅ ω * q
    // omega *= 0.5f;
    // const raylib::Quaternion delta_q(omega.x, omega.y, omega.z, 1.f);

    const auto delta_q =
        raylib::Quaternion::FromAxisAngle(omega, omega.Length() * delta);

    // q' = q * dq
    const auto rotated = (_orientation * delta_q).Normalize();

    std::println("orientation: {}, rotation: {}", _orientation, delta_q);
    std::println("rotated: {}", rotated);
    std::println();

    _orientation = rotated;
    _model.transform = rotated.ToMatrix();
  }
};
