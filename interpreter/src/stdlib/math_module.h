#ifndef ASTRA_MATH_MODULE_H
#define ASTRA_MATH_MODULE_H

#include <cmath>
#include <random>
#include "../vm/vm.h"
#include "../vm/value.h"

namespace astra {
namespace stdlib {
namespace math {

/**
 * Initialize the Math module in the VM
 */
void initialize(VM& vm);

// Math constants
constexpr double PI = M_PI;
constexpr double E = M_E;
constexpr double SQRT2 = M_SQRT2;
constexpr double INFINITY_VALUE = INFINITY;
constexpr double NAN_VALUE = NAN;

// Basic math functions
Value abs(const std::vector<Value>& args);
Value sign(const std::vector<Value>& args);
Value min(const std::vector<Value>& args);
Value max(const std::vector<Value>& args);
Value floor(const std::vector<Value>& args);
Value ceil(const std::vector<Value>& args);
Value round(const std::vector<Value>& args);
Value trunc(const std::vector<Value>& args);

// Exponential and logarithmic functions
Value exp(const std::vector<Value>& args);
Value log(const std::vector<Value>& args);
Value log10(const std::vector<Value>& args);
Value log2(const std::vector<Value>& args);
Value pow(const std::vector<Value>& args);
Value sqrt(const std::vector<Value>& args);
Value cbrt(const std::vector<Value>& args);

// Trigonometric functions
Value sin(const std::vector<Value>& args);
Value cos(const std::vector<Value>& args);
Value tan(const std::vector<Value>& args);
Value asin(const std::vector<Value>& args);
Value acos(const std::vector<Value>& args);
Value atan(const std::vector<Value>& args);
Value atan2(const std::vector<Value>& args);

// Hyperbolic functions
Value sinh(const std::vector<Value>& args);
Value cosh(const std::vector<Value>& args);
Value tanh(const std::vector<Value>& args);
Value asinh(const std::vector<Value>& args);
Value acosh(const std::vector<Value>& args);
Value atanh(const std::vector<Value>& args);

// Angle conversion
Value degrees(const std::vector<Value>& args);
Value radians(const std::vector<Value>& args);

// Random number generation
Value random(const std::vector<Value>& args);
Value randomInt(const std::vector<Value>& args);
Value randomFloat(const std::vector<Value>& args);
Value setSeed(const std::vector<Value>& args);

// Additional functions
Value clamp(const std::vector<Value>& args);
Value lerp(const std::vector<Value>& args);
Value smoothStep(const std::vector<Value>& args);
Value isNaN(const std::vector<Value>& args);
Value isInfinite(const std::vector<Value>& args);
Value isFinite(const std::vector<Value>& args);

} // namespace math
} // namespace stdlib
} // namespace astra

#endif // ASTRA_MATH_MODULE_H