
#include "derivative.hpp"
#include <functional>

template <typename Y> using ODE = std::function<Y(float, const Y &)>;

enum ODE_Method { EULER, HEUNE, MODIFIED_EULER, RUNGE_KUTTA_4 };

template <typename Y, const ODE_Method METHOD = ODE_Method::EULER>
Y ode(const ODE<Y> &f, Y y, double t0, double t1, float h) {
  const auto dt = t1 - t0;
  if constexpr (METHOD == ODE_Method::EULER) {
    return y + derivative(f, y, h) * h;
  } else if constexpr (METHOD == ODE_Method::HEUNE) {
    const auto df = derivative(f, y, h);
    const auto guess = y + df * h;
    return y + h / 2.f * (df + derivative(f, guess, h)) * h;
  } else if constexpr (METHOD == ODE_Method::MODIFIED_EULER) {
    const auto df = derivative(f, y, h);
    const auto guess = y + df * h / 2.f;
    return y + derivative(f, guess, h / 2.f) * h;
  } else if constexpr (METHOD == ODE_Method::RUNGE_KUTTA_4) {
    const auto k1 = derivative(f, y, h);
    const auto k2 = derivative(f, y + k1 * h / 2.f, h / 2.f);
    const auto k3 = derivative(f, y + k2 * h / 2.f, h / 2.f);
    const auto k4 = derivative(f, y + k3 * h, h);
    return y + (k1 + 2.f * k2 + 2.f * k3 + k4) * h / 6.f;
  } else {
    static_assert(false, "Unimplemented ODE method");
  }
}
