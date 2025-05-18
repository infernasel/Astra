#ifndef ASTRA_VECTOR_MODULE_H
#define ASTRA_VECTOR_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace vector {

/**
 * Initialize the Vector module in the VM
 */
void initialize(VM& vm);

// Vector2 class methods
Value vector2Constructor(const std::vector<Value>& args);
Value vector2Add(const std::vector<Value>& args);
Value vector2Subtract(const std::vector<Value>& args);
Value vector2Multiply(const std::vector<Value>& args);
Value vector2Divide(const std::vector<Value>& args);
Value vector2Dot(const std::vector<Value>& args);
Value vector2Cross(const std::vector<Value>& args); // Returns scalar for 2D vectors
Value vector2Magnitude(const std::vector<Value>& args);
Value vector2MagnitudeSquared(const std::vector<Value>& args);
Value vector2Normalize(const std::vector<Value>& args);
Value vector2Distance(const std::vector<Value>& args);
Value vector2DistanceSquared(const std::vector<Value>& args);
Value vector2Lerp(const std::vector<Value>& args);
Value vector2Angle(const std::vector<Value>& args);
Value vector2Rotate(const std::vector<Value>& args);
Value vector2ToString(const std::vector<Value>& args);
Value vector2Equals(const std::vector<Value>& args);

// Vector3 class methods
Value vector3Constructor(const std::vector<Value>& args);
Value vector3Add(const std::vector<Value>& args);
Value vector3Subtract(const std::vector<Value>& args);
Value vector3Multiply(const std::vector<Value>& args);
Value vector3Divide(const std::vector<Value>& args);
Value vector3Dot(const std::vector<Value>& args);
Value vector3Cross(const std::vector<Value>& args);
Value vector3Magnitude(const std::vector<Value>& args);
Value vector3MagnitudeSquared(const std::vector<Value>& args);
Value vector3Normalize(const std::vector<Value>& args);
Value vector3Distance(const std::vector<Value>& args);
Value vector3DistanceSquared(const std::vector<Value>& args);
Value vector3Lerp(const std::vector<Value>& args);
Value vector3Angle(const std::vector<Value>& args);
Value vector3RotateX(const std::vector<Value>& args);
Value vector3RotateY(const std::vector<Value>& args);
Value vector3RotateZ(const std::vector<Value>& args);
Value vector3ToString(const std::vector<Value>& args);
Value vector3Equals(const std::vector<Value>& args);

// Vector4 class methods
Value vector4Constructor(const std::vector<Value>& args);
Value vector4Add(const std::vector<Value>& args);
Value vector4Subtract(const std::vector<Value>& args);
Value vector4Multiply(const std::vector<Value>& args);
Value vector4Divide(const std::vector<Value>& args);
Value vector4Dot(const std::vector<Value>& args);
Value vector4Magnitude(const std::vector<Value>& args);
Value vector4MagnitudeSquared(const std::vector<Value>& args);
Value vector4Normalize(const std::vector<Value>& args);
Value vector4Distance(const std::vector<Value>& args);
Value vector4DistanceSquared(const std::vector<Value>& args);
Value vector4Lerp(const std::vector<Value>& args);
Value vector4ToString(const std::vector<Value>& args);
Value vector4Equals(const std::vector<Value>& args);

// Helper functions
ObjectPtr createVector2(double x, double y);
ObjectPtr createVector3(double x, double y, double z);
ObjectPtr createVector4(double x, double y, double z, double w);

bool isVector2(const Value& value);
bool isVector3(const Value& value);
bool isVector4(const Value& value);

} // namespace vector
} // namespace stdlib
} // namespace astra

#endif // ASTRA_VECTOR_MODULE_H