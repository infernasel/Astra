#ifndef ASTRA_MATRIX_MODULE_H
#define ASTRA_MATRIX_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace matrix {

/**
 * Initialize the Matrix module in the VM
 */
void initialize(VM& vm);

// Matrix2 class methods
Value matrix2Constructor(const std::vector<Value>& args);
Value matrix2Identity(const std::vector<Value>& args);
Value matrix2Add(const std::vector<Value>& args);
Value matrix2Subtract(const std::vector<Value>& args);
Value matrix2Multiply(const std::vector<Value>& args);
Value matrix2MultiplyScalar(const std::vector<Value>& args);
Value matrix2Determinant(const std::vector<Value>& args);
Value matrix2Inverse(const std::vector<Value>& args);
Value matrix2Transpose(const std::vector<Value>& args);
Value matrix2GetElement(const std::vector<Value>& args);
Value matrix2SetElement(const std::vector<Value>& args);
Value matrix2ToString(const std::vector<Value>& args);
Value matrix2Equals(const std::vector<Value>& args);

// Matrix3 class methods
Value matrix3Constructor(const std::vector<Value>& args);
Value matrix3Identity(const std::vector<Value>& args);
Value matrix3Add(const std::vector<Value>& args);
Value matrix3Subtract(const std::vector<Value>& args);
Value matrix3Multiply(const std::vector<Value>& args);
Value matrix3MultiplyScalar(const std::vector<Value>& args);
Value matrix3Determinant(const std::vector<Value>& args);
Value matrix3Inverse(const std::vector<Value>& args);
Value matrix3Transpose(const std::vector<Value>& args);
Value matrix3GetElement(const std::vector<Value>& args);
Value matrix3SetElement(const std::vector<Value>& args);
Value matrix3ToString(const std::vector<Value>& args);
Value matrix3Equals(const std::vector<Value>& args);
Value matrix3RotationX(const std::vector<Value>& args);
Value matrix3RotationY(const std::vector<Value>& args);
Value matrix3RotationZ(const std::vector<Value>& args);
Value matrix3Scale(const std::vector<Value>& args);
Value matrix3Translation(const std::vector<Value>& args);

// Matrix4 class methods
Value matrix4Constructor(const std::vector<Value>& args);
Value matrix4Identity(const std::vector<Value>& args);
Value matrix4Add(const std::vector<Value>& args);
Value matrix4Subtract(const std::vector<Value>& args);
Value matrix4Multiply(const std::vector<Value>& args);
Value matrix4MultiplyScalar(const std::vector<Value>& args);
Value matrix4Determinant(const std::vector<Value>& args);
Value matrix4Inverse(const std::vector<Value>& args);
Value matrix4Transpose(const std::vector<Value>& args);
Value matrix4GetElement(const std::vector<Value>& args);
Value matrix4SetElement(const std::vector<Value>& args);
Value matrix4ToString(const std::vector<Value>& args);
Value matrix4Equals(const std::vector<Value>& args);
Value matrix4RotationX(const std::vector<Value>& args);
Value matrix4RotationY(const std::vector<Value>& args);
Value matrix4RotationZ(const std::vector<Value>& args);
Value matrix4Scale(const std::vector<Value>& args);
Value matrix4Translation(const std::vector<Value>& args);
Value matrix4LookAt(const std::vector<Value>& args);
Value matrix4Perspective(const std::vector<Value>& args);
Value matrix4Orthographic(const std::vector<Value>& args);

// Helper functions
ObjectPtr createMatrix2(const std::vector<double>& elements);
ObjectPtr createMatrix3(const std::vector<double>& elements);
ObjectPtr createMatrix4(const std::vector<double>& elements);

bool isMatrix2(const Value& value);
bool isMatrix3(const Value& value);
bool isMatrix4(const Value& value);

} // namespace matrix
} // namespace stdlib
} // namespace astra

#endif // ASTRA_MATRIX_MODULE_H