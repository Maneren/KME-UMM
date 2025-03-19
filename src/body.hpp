#pragma once

#include "consts.hpp"
#include "defs.hpp"
#include "object.hpp"
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

  Body &position(const raylib::Vector3 &position) {
    this->_position = position;
    return *this;
  }
  Body &velocity(const raylib::Vector3 &velocity) {
    this->_velocity = velocity;
    return *this;
  }
  Body &acceleration(const raylib::Vector3 &acceleration) {
    this->_acceleration = acceleration;
    return *this;
  }
  Body &mass(const float mass) {
    this->_mass = mass;
    return *this;
  }
  Body &orientation(const raylib::Quaternion &orientation) {
    this->_orientation = orientation;
    return *this;
  }
  Body &angular_velocity(const raylib::Vector3 &angular_velocity) {
    this->_angular_velocity = angular_velocity;
    return *this;
  }
  Body &angular_acceleration(const raylib::Vector3 &angular_acceleration) {
    this->_angular_acceleration = angular_acceleration;
    return *this;
  }
  Body &texture(raylib::Texture &&texture) {
    this->_texture = std::move(texture);
    return *this;
  }

  const raylib::Vector3 &position() const { return _position; };
  const raylib::Vector3 &velocity() const { return _velocity; }
  const raylib::Vector3 &acceleration() const { return _acceleration; };
  float mass() const { return _mass; }
  const raylib::Quaternion &orientation() const { return _orientation; }
  const raylib::Vector3 &angular_velocity() const { return _angular_velocity; }
  const raylib::Vector3 &angular_acceleration() const {
    return _angular_acceleration;
  }
  const raylib::Texture &texture() const { return _texture; }

  virtual void initialize() override {
    raylib::Image image = raylib::Image::Color(1, 1, _color);
    _texture = raylib::Texture(image);

    _mesh = create_mesh();
    _model.Load(_mesh);
    _model.transform = _orientation.ToMatrix();
    _model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = _texture;
  }

  virtual void draw() const override {
    _model.Draw(_position, 1.0f, raylib::Color::White());
  }

  raylib::Vector3 transform_point(const raylib::Vector3 &point) {
    return point.Transform(_model.transform);
  }

  virtual float moment_of_inertia(const raylib::Vector3 &axis) = 0;

  virtual void apply_force(
      const raylib::Vector3 &force,
      raylib::Vector3 offset = raylib::Vector3::Zero()
  ) {
    body_apply_force(force, offset);
  };
  virtual void update(const float delta) override { body_update(delta); };

  virtual raylib::Vector3 transform_offset(const raylib::Vector3 &offset
  ) const {
    return offset.Transform(_model.transform);
  }

protected:
  raylib::Vector3 _position;
  raylib::Vector3 _velocity = raylib::Vector3::Zero();
  raylib::Vector3 _acceleration = raylib::Vector3::Zero();

  float _mass = 1.0f;

  raylib::Quaternion _orientation;
  raylib::Vector3 _angular_velocity = raylib::Vector3::Zero();
  raylib::Vector3 _angular_acceleration = raylib::Vector3::Zero();

  raylib::Model _model;
  raylib::MeshUnmanaged _mesh;
  raylib::Color _color = raylib::Color::White();
  raylib::Texture2D _texture;

  virtual raylib::MeshUnmanaged create_mesh() = 0;

  void body_apply_force(
      const raylib::Vector3 &force, const raylib::Vector3 &offset
  ) {
    _acceleration += force / _mass;

    const auto length = offset.Length();
    if (length <= EPSILON)
      return;

    const auto torque =
        transform_offset(offset).CrossProduct(transform_offset(force));
    const float moment = moment_of_inertia(torque);
    _angular_acceleration += torque / moment;
  };

  void body_update(const float delta) {
    update_position(delta);
    update_orientation(delta);
  };

private:
  void update_position(const float delta) {
    _velocity += _acceleration * delta;
    _position += _velocity * delta;
    _acceleration = raylib::Vector3::Zero();
  }

  void update_orientation(const float delta) {
    _angular_velocity += _angular_acceleration * delta;
    _angular_acceleration = raylib::Vector3::Zero();

    const auto angle = _angular_velocity.Length();

    if (angle <= EPSILON)
      return;

    _angular_velocity *= std::pow(ENVIRONMENT_FRICTION_COEFFICIENT, delta);

    const auto axis = _angular_velocity.Normalize();
    const auto rotation = raylib::Quaternion::FromAxisAngle(axis, angle);

    std::println("orientation: {}, rotation: {}", _orientation, rotation);
    _orientation = (_orientation * rotation).Normalize();
    _model.transform = _orientation.ToMatrix();
  }
};
