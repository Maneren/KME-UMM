#include <functional>

template <typename Y>
Y derivative(
    const std::function<Y(float, const Y &)> f, const Y y, const float h
) {
  // central difference method
  return (f(h, y) - f(-h, y)) / (2 * h);
}
