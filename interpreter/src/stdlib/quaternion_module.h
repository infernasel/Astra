#ifndef ASTRA_QUATERNION_MODULE_H
#define ASTRA_QUATERNION_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace quaternion {

/**
 * Initialize the Quaternion module in the VM
 */
void initialize(VM& vm);

// Quaternion class methods
Value quaternionConstructor(const std::vector<Value>& args);
Value quaternionIdentity(const std::vector<Value>& args);
Value quaternionFromAxisAngle(const std::vector<Value>& args);
Value quaternionFromEuler(const std::vector<Value>& args);
Value quaternionFromMatrix(const std::vector<Value>& args);
Value quaternionToMatrix(const std::vector<Value>& args);
Value quaternionToEuler(const std::vector<Value>& args);
Value quaternionAdd(const std::vector<Value>& args);
Value quaternionSubtract(const std::vector<Value>& args);
Value quaternionMultiply(const std::vector<Value>& args);
Value quaternionMultiplyScalar(const std::vector<Value>& args);
Value quaternionDivide(const std::vector<Value>& args);
Value quaternionConjugate(const std::vector<Value>& args);
Value quaternionInverse(const std::vector<Value>& args);
Value quaternionNormalize(const std::vector<Value>& args);
Value quaternionMagnitude(const std::vector<Value>& args);
Value quaternionMagnitudeSquared(const std::vector<Value>& args);
Value quaternionDot(const std::vector<Value>& args);
Value quaternionSlerp(const std::vector<Value>& args);
Value quaternionRotateVector(const std::vector<Value>& args);
Value quaternionGetAxis(const std::vector<Value>& args);
Value quaternionGetAngle(const std::vector<Value>& args);
Value quaternionToString(const std::vector<Value>& args);
Value quaternionEquals(const std::vector<Value>& args);

// Helper functions
ObjectPtr createQuaternion(double x, double y, double z, double w);
bool isQuaternion(const Value& value);

} // namespace quaternion
} // namespace stdlib
} // namespace astra

#endif // ASTRA_QUATERNION_MODULE_H