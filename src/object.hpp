#pragma once

#include "drawable.hpp"

class Object : public Drawable {
public:
  virtual void initialize() {};
  virtual void update(const float dt) = 0;
};
