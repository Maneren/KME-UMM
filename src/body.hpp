#pragma once

#include "object.hpp"
#include "ode.hpp"
#include <Image.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <Model.hpp>
#include <Texture.hpp>
#include <Vector3.hpp>
#include <Vector4.hpp>
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

  /// Description of the state of the Body
  struct State {
    // body constants
    const float inverse_mass;
    const raylib::Matrix inverse_body_inertia_tensor;

    // state
    raylib::Vector3 position, linear_momentum;
    raylib::Quaternion orientation;
    raylib::Vector3 angular_momentum;

    // derived quantities
    raylib::Matrix inverse_inertia_tensor;
    raylib::Matrix rotation_matrix;
    raylib::Vector3 velocity, angular_velocity;

    // computed
    raylib::Vector3 net_force, net_torque;

    /// Derivative of the State
    struct Derivative {
      // position, orientation
      raylib::Vector3 velocity, angular_velocity;
      // linear_momentum, angular_momentum
      raylib::Vector3 force, torque;
    };

    Derivative dydt(const double t, const State &state) const {
      return {

      };
    }
  };

  // struct Change {
  //   raylib::Vector3 velocity, angular_velocity;
  //   raylib::Vector3 force, torque;
  // };
  //
  // static const State update(const float delta, const State &state) {
  //   return {
  //       .inverse_mass = state.inverse_mass,
  //       .inverse_inertia_tensor = state.inverse_inertia_tensor,
  //       .net_force = state.net_force,
  //       .net_torque = state.net_torque,
  //
  //       .position = state.position + state.velocity() * delta,
  //       .linear_momentum = state.linear_momentum + state.net_force * delta,
  //       .orientation =
  //           (state.orientation *
  //            angular_velocity_to_rotation(state.angular_velocity(), delta))
  //               .Normalize(),
  //       .angular_momentum = state.angular_momentum + state.net_torque *
  //       delta,
  //   };
  // }
  //
  // State operator+(const State &other) const {
  //   if (inverse_mass != other.inverse_mass ||
  //       inverse_inertia_tensor != other.inverse_inertia_tensor ||
  //       net_force != other.net_force || net_torque != other.net_torque)
  //     throw std::runtime_error("incompatible state can't be added");
  //
  //   return {
  //       .inverse_mass = inverse_mass,
  //       .inverse_inertia_tensor = inverse_inertia_tensor,
  //       .net_force = net_force,
  //       .net_torque = net_torque,
  //       .position = position + other.position,
  //       .linear_momentum = linear_momentum + other.linear_momentum,
  //       .orientation = orientation * other.orientation,
  //       .angular_momentum = angular_momentum + other.angular_momentum
  //   };
  // }
  //
  // State operator-(const State &other) const {
  //   if (inverse_mass != other.inverse_mass ||
  //       inverse_inertia_tensor != other.inverse_inertia_tensor ||
  //       net_force != other.net_force || net_torque != other.net_torque)
  //     throw std::runtime_error("incompatible state can't be added");
  //
  //   return {
  //       .inverse_mass = inverse_mass,
  //       .inverse_inertia_tensor = inverse_inertia_tensor,
  //       .net_force = net_force,
  //       .net_torque = net_torque,
  //       .position = position - other.position,
  //       .linear_momentum = linear_momentum - other.linear_momentum,
  //       .orientation = orientation * other.orientation.Invert(),
  //       .angular_momentum = angular_momentum + other.angular_momentum,
  //   };
  // }

  // State state() {
  //   return {
  //       .inverse_mass = _inverse_mass,
  //       .inverse_inertia_tensor = _inverse_inertia_tensor,
  //       .net_force = _net_force,
  //       .net_torque = _net_torque,
  //       .position = _position,
  //       .linear_momentum = _linear_momentum,
  //       .orientation = _orientation,
  //       .angular_momentum = _angular_momentum,
  //   };
  // }

  // void state(const State &state) {
  //   _position = state.position;
  //   _linear_momentum = state.linear_momentum;
  //   _orientation = state.orientation;
  //   _angular_momentum = state.angular_momentum;
  //   update_velocity();
  //   update_angular_velocity();
  //   update_inverse_inertia_tensor();
  // }

  inline float mass() const { return 1.f / _inverse_mass; }
  inline float inverse_mass() const { return _inverse_mass; }

  inline const raylib::Matrix &inverse_inertia_tensor() const {
    return _inverse_inertia_tensor;
  }

  inline raylib::Vector3 &position() { return _position; };
  inline raylib::Vector3 &linear_momentum() { return _linear_momentum; }
  inline const raylib::Vector3 &velocity() const { return _velocity; }
  inline const raylib::Vector3 &force() const { return _net_force; }

  inline raylib::Quaternion &orientation() { return _orientation; }
  inline raylib::Vector3 &angular_momentum() { return _angular_momentum; }
  inline const raylib::Vector3 &angular_velocity() const {
    return _angular_velocity;
  }
  inline const raylib::Vector3 &torque() const { return _net_torque; }

  inline const raylib::Texture &texture() const { return _texture; }

  inline float linear_energy() const {
    // Eₖ = ½ |𝐏|² m⁻¹ (equivalent to Eₖ = ½ m |𝐯|², where 𝐯 = 𝐏 / m)
    return 0.5f * _linear_momentum.LengthSqr() * _inverse_mass;
  }

  inline float angular_energy() const {
    // Eₘ = ½ 𝐋 ⋅ 𝛚 (equivalent to Eₘ = ½ 𝐈𝛚², since 𝛚 = 𝐈⁻¹𝐋)
    return _angular_velocity.DotProduct(_angular_momentum) * 0.5f;
  }

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

  raylib::Vector3 transform_point(const raylib::Vector3 &point) const;
  raylib::Vector3 point_velocity(const raylib::Vector3 &point) const;

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

  void update_velocity();
  void update_angular_velocity();
  void update_inverse_inertia_tensor();

  void
  body_apply_force(const raylib::Vector3 &force, const raylib::Vector3 &offset);

  // static State state_change(const float delta, const State &state) {
  //   auto new_state = state;
  //
  //   // dP = F ⋅ dt
  //   new_state.linear_momentum = state.linear_momentum + state.net_force *
  //   delta;
  //   // dx = v ⋅ dt
  //   new_state.position = state.position + new_state.velocity() * delta;
  //
  //   // dL = τ ⋅ dt
  //   new_state.angular_momentum =
  //       state.angular_momentum + state.net_torque * delta;
  //   // dq = ω ⋅ dt
  //   new_state.orientation =
  //       (state.orientation *
  //        angular_velocity_to_rotation(new_state.angular_velocity(), delta))
  //           .Normalize();
  //
  //   return new_state;
  // }

  void body_update(const float delta);

private:
  void update_position(const float delta);

  void update_orientation(const float delta);
};
