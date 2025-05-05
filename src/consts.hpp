#pragma once

#include "material.hpp"
#include <Vector3.hpp>

const raylib::Vector3 GRAVITY{0.0f, -9.81f, 0.0f};

const BodyMaterial GROUND_MATERIAL{0.4f, 0.25f};

constexpr float AIR_DENSITY = 1.293e-3;

constexpr float BOUNCE_COEFFICIENT = 0.4f;
constexpr float DRAG_COEFFICIENT_CUBE = 2.1f;

constexpr float ENVIRONMENT_FRICTION_COEFFICIENT = 0.2f;

constexpr float COEFFICIENT_OF_RESTITUTION = 0.8f;
