#include "vector_module.h"
#include "../vm/object.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace astra {
namespace stdlib {
namespace vector {

// Constants
constexpr const char* VECTOR2_CLASS = "Vector2";
constexpr const char* VECTOR3_CLASS = "Vector3";
constexpr const char* VECTOR4_CLASS = "Vector4";

// Helper function to validate arguments
void validateArgs(const std::vector<Value>& args, size_t expected, const char* funcName) {
    if (args.size() != expected) {
        throw std::runtime_error(std::string(funcName) + " expects " + 
                                std::to_string(expected) + " arguments, got " + 
                                std::to_string(args.size()));
    }
}

// Helper function to validate numeric arguments
void validateNumeric(const Value& value, const char* funcName) {
    if (!value.isNumber()) {
        throw std::runtime_error(std::string(funcName) + " expects numeric arguments");
    }
}

void initialize(VM& vm) {
    // Create Vector2 class
    ObjectPtr vector2Class = vm.createObject();
    
    // Vector2 constructor
    vector2Class->set("constructor", Value::createNativeFunction(vector2Constructor));
    
    // Vector2 prototype
    ObjectPtr vector2Prototype = vm.createObject();
    vector2Prototype->set("add", Value::createNativeFunction(vector2Add));
    vector2Prototype->set("subtract", Value::createNativeFunction(vector2Subtract));
    vector2Prototype->set("multiply", Value::createNativeFunction(vector2Multiply));
    vector2Prototype->set("divide", Value::createNativeFunction(vector2Divide));
    vector2Prototype->set("dot", Value::createNativeFunction(vector2Dot));
    vector2Prototype->set("cross", Value::createNativeFunction(vector2Cross));
    vector2Prototype->set("magnitude", Value::createNativeFunction(vector2Magnitude));
    vector2Prototype->set("magnitudeSquared", Value::createNativeFunction(vector2MagnitudeSquared));
    vector2Prototype->set("normalize", Value::createNativeFunction(vector2Normalize));
    vector2Prototype->set("distance", Value::createNativeFunction(vector2Distance));
    vector2Prototype->set("distanceSquared", Value::createNativeFunction(vector2DistanceSquared));
    vector2Prototype->set("lerp", Value::createNativeFunction(vector2Lerp));
    vector2Prototype->set("angle", Value::createNativeFunction(vector2Angle));
    vector2Prototype->set("rotate", Value::createNativeFunction(vector2Rotate));
    vector2Prototype->set("toString", Value::createNativeFunction(vector2ToString));
    vector2Prototype->set("equals", Value::createNativeFunction(vector2Equals));
    
    vector2Class->set("prototype", Value(vector2Prototype));
    
    // Create Vector3 class
    ObjectPtr vector3Class = vm.createObject();
    
    // Vector3 constructor
    vector3Class->set("constructor", Value::createNativeFunction(vector3Constructor));
    
    // Vector3 prototype
    ObjectPtr vector3Prototype = vm.createObject();
    vector3Prototype->set("add", Value::createNativeFunction(vector3Add));
    vector3Prototype->set("subtract", Value::createNativeFunction(vector3Subtract));
    vector3Prototype->set("multiply", Value::createNativeFunction(vector3Multiply));
    vector3Prototype->set("divide", Value::createNativeFunction(vector3Divide));
    vector3Prototype->set("dot", Value::createNativeFunction(vector3Dot));
    vector3Prototype->set("cross", Value::createNativeFunction(vector3Cross));
    vector3Prototype->set("magnitude", Value::createNativeFunction(vector3Magnitude));
    vector3Prototype->set("magnitudeSquared", Value::createNativeFunction(vector3MagnitudeSquared));
    vector3Prototype->set("normalize", Value::createNativeFunction(vector3Normalize));
    vector3Prototype->set("distance", Value::createNativeFunction(vector3Distance));
    vector3Prototype->set("distanceSquared", Value::createNativeFunction(vector3DistanceSquared));
    vector3Prototype->set("lerp", Value::createNativeFunction(vector3Lerp));
    vector3Prototype->set("angle", Value::createNativeFunction(vector3Angle));
    vector3Prototype->set("rotateX", Value::createNativeFunction(vector3RotateX));
    vector3Prototype->set("rotateY", Value::createNativeFunction(vector3RotateY));
    vector3Prototype->set("rotateZ", Value::createNativeFunction(vector3RotateZ));
    vector3Prototype->set("toString", Value::createNativeFunction(vector3ToString));
    vector3Prototype->set("equals", Value::createNativeFunction(vector3Equals));
    
    vector3Class->set("prototype", Value(vector3Prototype));
    
    // Create Vector4 class
    ObjectPtr vector4Class = vm.createObject();
    
    // Vector4 constructor
    vector4Class->set("constructor", Value::createNativeFunction(vector4Constructor));
    
    // Vector4 prototype
    ObjectPtr vector4Prototype = vm.createObject();
    vector4Prototype->set("add", Value::createNativeFunction(vector4Add));
    vector4Prototype->set("subtract", Value::createNativeFunction(vector4Subtract));
    vector4Prototype->set("multiply", Value::createNativeFunction(vector4Multiply));
    vector4Prototype->set("divide", Value::createNativeFunction(vector4Divide));
    vector4Prototype->set("dot", Value::createNativeFunction(vector4Dot));
    vector4Prototype->set("magnitude", Value::createNativeFunction(vector4Magnitude));
    vector4Prototype->set("magnitudeSquared", Value::createNativeFunction(vector4MagnitudeSquared));
    vector4Prototype->set("normalize", Value::createNativeFunction(vector4Normalize));
    vector4Prototype->set("distance", Value::createNativeFunction(vector4Distance));
    vector4Prototype->set("distanceSquared", Value::createNativeFunction(vector4DistanceSquared));
    vector4Prototype->set("lerp", Value::createNativeFunction(vector4Lerp));
    vector4Prototype->set("toString", Value::createNativeFunction(vector4ToString));
    vector4Prototype->set("equals", Value::createNativeFunction(vector4Equals));
    
    vector4Class->set("prototype", Value(vector4Prototype));
    
    // Register the Vector classes in the global scope
    vm.setGlobal(VECTOR2_CLASS, Value(vector2Class));
    vm.setGlobal(VECTOR3_CLASS, Value(vector3Class));
    vm.setGlobal(VECTOR4_CLASS, Value(vector4Class));
}

// Helper functions
ObjectPtr createVector2(double x, double y) {
    ObjectPtr vector = std::make_shared<Object>();
    vector->set("x", Value(x));
    vector->set("y", Value(y));
    vector->setClassName(VECTOR2_CLASS);
    return vector;
}

ObjectPtr createVector3(double x, double y, double z) {
    ObjectPtr vector = std::make_shared<Object>();
    vector->set("x", Value(x));
    vector->set("y", Value(y));
    vector->set("z", Value(z));
    vector->setClassName(VECTOR3_CLASS);
    return vector;
}

ObjectPtr createVector4(double x, double y, double z, double w) {
    ObjectPtr vector = std::make_shared<Object>();
    vector->set("x", Value(x));
    vector->set("y", Value(y));
    vector->set("z", Value(z));
    vector->set("w", Value(w));
    vector->setClassName(VECTOR4_CLASS);
    return vector;
}

bool isVector2(const Value& value) {
    if (!value.isObject()) {
        return false;
    }
    
    ObjectPtr obj = value.asObject();
    return obj->getClassName() == VECTOR2_CLASS;
}

bool isVector3(const Value& value) {
    if (!value.isObject()) {
        return false;
    }
    
    ObjectPtr obj = value.asObject();
    return obj->getClassName() == VECTOR3_CLASS;
}

bool isVector4(const Value& value) {
    if (!value.isObject()) {
        return false;
    }
    
    ObjectPtr obj = value.asObject();
    return obj->getClassName() == VECTOR4_CLASS;
}

// Vector2 methods
Value vector2Constructor(const std::vector<Value>& args) {
    if (args.size() < 2) {
        throw std::runtime_error("Vector2 constructor expects at least 2 arguments");
    }
    
    validateNumeric(args[0], "Vector2.constructor");
    validateNumeric(args[1], "Vector2.constructor");
    
    double x = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double y = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    return Value(createVector2(x, y));
}

Value vector2Add(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.add");
    
    if (!isVector2(args[0])) {
        throw std::runtime_error("Vector2.add expects a Vector2 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    return Value(createVector2(x1 + x2, y1 + y2));
}

Value vector2Subtract(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.subtract");
    
    if (!isVector2(args[0])) {
        throw std::runtime_error("Vector2.subtract expects a Vector2 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    return Value(createVector2(x1 - x2, y1 - y2));
}

Value vector2Multiply(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.multiply");
    validateNumeric(args[0], "Vector2.multiply");
    
    ObjectPtr self = args.at(0).asObject();
    double scalar = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    return Value(createVector2(x * scalar, y * scalar));
}

Value vector2Divide(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.divide");
    validateNumeric(args[0], "Vector2.divide");
    
    ObjectPtr self = args.at(0).asObject();
    double scalar = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    if (scalar == 0.0) {
        throw std::runtime_error("Vector2.divide: Division by zero");
    }
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    return Value(createVector2(x / scalar, y / scalar));
}

Value vector2Dot(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.dot");
    
    if (!isVector2(args[0])) {
        throw std::runtime_error("Vector2.dot expects a Vector2 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    return Value(x1 * x2 + y1 * y2);
}

Value vector2Cross(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.cross");
    
    if (!isVector2(args[0])) {
        throw std::runtime_error("Vector2.cross expects a Vector2 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    // 2D cross product returns a scalar (the z component of the 3D cross product)
    return Value(x1 * y2 - y1 * x2);
}

Value vector2Magnitude(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector2.magnitude");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    return Value(std::sqrt(x * x + y * y));
}

Value vector2MagnitudeSquared(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector2.magnitudeSquared");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    return Value(x * x + y * y);
}

Value vector2Normalize(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector2.normalize");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    double magnitude = std::sqrt(x * x + y * y);
    
    if (magnitude < 1e-10) {
        throw std::runtime_error("Vector2.normalize: Cannot normalize zero vector");
    }
    
    return Value(createVector2(x / magnitude, y / magnitude));
}

Value vector2Distance(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.distance");
    
    if (!isVector2(args[0])) {
        throw std::runtime_error("Vector2.distance expects a Vector2 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    return Value(std::sqrt(dx * dx + dy * dy));
}

Value vector2DistanceSquared(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.distanceSquared");
    
    if (!isVector2(args[0])) {
        throw std::runtime_error("Vector2.distanceSquared expects a Vector2 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    return Value(dx * dx + dy * dy);
}

Value vector2Lerp(const std::vector<Value>& args) {
    validateArgs(args, 2, "Vector2.lerp");
    
    if (!isVector2(args[0])) {
        throw std::runtime_error("Vector2.lerp expects a Vector2 as first argument");
    }
    
    validateNumeric(args[1], "Vector2.lerp");
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    double t = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    return Value(createVector2(x1 + t * (x2 - x1), y1 + t * (y2 - y1)));
}

Value vector2Angle(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector2.angle");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    return Value(std::atan2(y, x));
}

Value vector2Rotate(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.rotate");
    validateNumeric(args[0], "Vector2.rotate");
    
    ObjectPtr self = args.at(0).asObject();
    double angle = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    
    double newX = x * cosAngle - y * sinAngle;
    double newY = x * sinAngle + y * cosAngle;
    
    return Value(createVector2(newX, newY));
}

Value vector2ToString(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector2.toString");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Vector2(" << x << ", " << y << ")";
    
    return Value(ss.str());
}

Value vector2Equals(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector2.equals");
    
    if (!isVector2(args[0])) {
        return Value(false);
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    
    // Use epsilon for floating point comparison
    constexpr double epsilon = 1e-10;
    return Value(std::abs(x1 - x2) < epsilon && std::abs(y1 - y2) < epsilon);
}

// Vector3 methods
Value vector3Constructor(const std::vector<Value>& args) {
    if (args.size() < 3) {
        throw std::runtime_error("Vector3 constructor expects at least 3 arguments");
    }
    
    validateNumeric(args[0], "Vector3.constructor");
    validateNumeric(args[1], "Vector3.constructor");
    validateNumeric(args[2], "Vector3.constructor");
    
    double x = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double y = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    double z = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
    
    return Value(createVector3(x, y, z));
}

Value vector3Add(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.add");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.add expects a Vector3 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    return Value(createVector3(x1 + x2, y1 + y2, z1 + z2));
}

Value vector3Subtract(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.subtract");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.subtract expects a Vector3 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    return Value(createVector3(x1 - x2, y1 - y2, z1 - z2));
}

Value vector3Multiply(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.multiply");
    validateNumeric(args[0], "Vector3.multiply");
    
    ObjectPtr self = args.at(0).asObject();
    double scalar = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    return Value(createVector3(x * scalar, y * scalar, z * scalar));
}

Value vector3Divide(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.divide");
    validateNumeric(args[0], "Vector3.divide");
    
    ObjectPtr self = args.at(0).asObject();
    double scalar = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    if (scalar == 0.0) {
        throw std::runtime_error("Vector3.divide: Division by zero");
    }
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    return Value(createVector3(x / scalar, y / scalar, z / scalar));
}

Value vector3Dot(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.dot");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.dot expects a Vector3 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    return Value(x1 * x2 + y1 * y2 + z1 * z2);
}

Value vector3Cross(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.cross");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.cross expects a Vector3 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    double newX = y1 * z2 - z1 * y2;
    double newY = z1 * x2 - x1 * z2;
    double newZ = x1 * y2 - y1 * x2;
    
    return Value(createVector3(newX, newY, newZ));
}

Value vector3Magnitude(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector3.magnitude");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    return Value(std::sqrt(x * x + y * y + z * z));
}

Value vector3MagnitudeSquared(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector3.magnitudeSquared");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    return Value(x * x + y * y + z * z);
}

Value vector3Normalize(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector3.normalize");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    double magnitude = std::sqrt(x * x + y * y + z * z);
    
    if (magnitude < 1e-10) {
        throw std::runtime_error("Vector3.normalize: Cannot normalize zero vector");
    }
    
    return Value(createVector3(x / magnitude, y / magnitude, z / magnitude));
}

Value vector3Distance(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.distance");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.distance expects a Vector3 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    
    return Value(std::sqrt(dx * dx + dy * dy + dz * dz));
}

Value vector3DistanceSquared(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.distanceSquared");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.distanceSquared expects a Vector3 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    
    return Value(dx * dx + dy * dy + dz * dz);
}

Value vector3Lerp(const std::vector<Value>& args) {
    validateArgs(args, 2, "Vector3.lerp");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.lerp expects a Vector3 as first argument");
    }
    
    validateNumeric(args[1], "Vector3.lerp");
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    double t = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    return Value(createVector3(
        x1 + t * (x2 - x1),
        y1 + t * (y2 - y1),
        z1 + t * (z2 - z1)
    ));
}

Value vector3Angle(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.angle");
    
    if (!isVector3(args[0])) {
        throw std::runtime_error("Vector3.angle expects a Vector3 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    double dot = x1 * x2 + y1 * y2 + z1 * z2;
    double mag1 = std::sqrt(x1 * x1 + y1 * y1 + z1 * z1);
    double mag2 = std::sqrt(x2 * x2 + y2 * y2 + z2 * z2);
    
    if (mag1 < 1e-10 || mag2 < 1e-10) {
        throw std::runtime_error("Vector3.angle: Cannot compute angle with zero vector");
    }
    
    double cosAngle = dot / (mag1 * mag2);
    
    // Clamp to [-1, 1] to handle floating point errors
    cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
    
    return Value(std::acos(cosAngle));
}

Value vector3RotateX(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.rotateX");
    validateNumeric(args[0], "Vector3.rotateX");
    
    ObjectPtr self = args.at(0).asObject();
    double angle = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    
    double newY = y * cosAngle - z * sinAngle;
    double newZ = y * sinAngle + z * cosAngle;
    
    return Value(createVector3(x, newY, newZ));
}

Value vector3RotateY(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.rotateY");
    validateNumeric(args[0], "Vector3.rotateY");
    
    ObjectPtr self = args.at(0).asObject();
    double angle = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    
    double newX = x * cosAngle + z * sinAngle;
    double newZ = -x * sinAngle + z * cosAngle;
    
    return Value(createVector3(newX, y, newZ));
}

Value vector3RotateZ(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.rotateZ");
    validateNumeric(args[0], "Vector3.rotateZ");
    
    ObjectPtr self = args.at(0).asObject();
    double angle = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    
    double newX = x * cosAngle - y * sinAngle;
    double newY = x * sinAngle + y * cosAngle;
    
    return Value(createVector3(newX, newY, z));
}

Value vector3ToString(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector3.toString");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Vector3(" << x << ", " << y << ", " << z << ")";
    
    return Value(ss.str());
}

Value vector3Equals(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector3.equals");
    
    if (!isVector3(args[0])) {
        return Value(false);
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    
    // Use epsilon for floating point comparison
    constexpr double epsilon = 1e-10;
    return Value(
        std::abs(x1 - x2) < epsilon && 
        std::abs(y1 - y2) < epsilon && 
        std::abs(z1 - z2) < epsilon
    );
}

// Vector4 methods
Value vector4Constructor(const std::vector<Value>& args) {
    if (args.size() < 4) {
        throw std::runtime_error("Vector4 constructor expects at least 4 arguments");
    }
    
    validateNumeric(args[0], "Vector4.constructor");
    validateNumeric(args[1], "Vector4.constructor");
    validateNumeric(args[2], "Vector4.constructor");
    validateNumeric(args[3], "Vector4.constructor");
    
    double x = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double y = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    double z = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
    double w = args[3].isInt() ? static_cast<double>(args[3].asInt()) : args[3].asFloat();
    
    return Value(createVector4(x, y, z, w));
}

Value vector4Add(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.add");
    
    if (!isVector4(args[0])) {
        throw std::runtime_error("Vector4.add expects a Vector4 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    double w1 = self->get("w").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    double w2 = other->get("w").asFloat();
    
    return Value(createVector4(x1 + x2, y1 + y2, z1 + z2, w1 + w2));
}

Value vector4Subtract(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.subtract");
    
    if (!isVector4(args[0])) {
        throw std::runtime_error("Vector4.subtract expects a Vector4 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    double w1 = self->get("w").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    double w2 = other->get("w").asFloat();
    
    return Value(createVector4(x1 - x2, y1 - y2, z1 - z2, w1 - w2));
}

Value vector4Multiply(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.multiply");
    validateNumeric(args[0], "Vector4.multiply");
    
    ObjectPtr self = args.at(0).asObject();
    double scalar = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    double w = self->get("w").asFloat();
    
    return Value(createVector4(x * scalar, y * scalar, z * scalar, w * scalar));
}

Value vector4Divide(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.divide");
    validateNumeric(args[0], "Vector4.divide");
    
    ObjectPtr self = args.at(0).asObject();
    double scalar = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    if (scalar == 0.0) {
        throw std::runtime_error("Vector4.divide: Division by zero");
    }
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    double w = self->get("w").asFloat();
    
    return Value(createVector4(x / scalar, y / scalar, z / scalar, w / scalar));
}

Value vector4Dot(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.dot");
    
    if (!isVector4(args[0])) {
        throw std::runtime_error("Vector4.dot expects a Vector4 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    double w1 = self->get("w").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    double w2 = other->get("w").asFloat();
    
    return Value(x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2);
}

Value vector4Magnitude(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector4.magnitude");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    double w = self->get("w").asFloat();
    
    return Value(std::sqrt(x * x + y * y + z * z + w * w));
}

Value vector4MagnitudeSquared(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector4.magnitudeSquared");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    double w = self->get("w").asFloat();
    
    return Value(x * x + y * y + z * z + w * w);
}

Value vector4Normalize(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector4.normalize");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    double w = self->get("w").asFloat();
    
    double magnitude = std::sqrt(x * x + y * y + z * z + w * w);
    
    if (magnitude < 1e-10) {
        throw std::runtime_error("Vector4.normalize: Cannot normalize zero vector");
    }
    
    return Value(createVector4(x / magnitude, y / magnitude, z / magnitude, w / magnitude));
}

Value vector4Distance(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.distance");
    
    if (!isVector4(args[0])) {
        throw std::runtime_error("Vector4.distance expects a Vector4 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    double w1 = self->get("w").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    double w2 = other->get("w").asFloat();
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    double dw = w2 - w1;
    
    return Value(std::sqrt(dx * dx + dy * dy + dz * dz + dw * dw));
}

Value vector4DistanceSquared(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.distanceSquared");
    
    if (!isVector4(args[0])) {
        throw std::runtime_error("Vector4.distanceSquared expects a Vector4 argument");
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    double w1 = self->get("w").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    double w2 = other->get("w").asFloat();
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    double dw = w2 - w1;
    
    return Value(dx * dx + dy * dy + dz * dz + dw * dw);
}

Value vector4Lerp(const std::vector<Value>& args) {
    validateArgs(args, 2, "Vector4.lerp");
    
    if (!isVector4(args[0])) {
        throw std::runtime_error("Vector4.lerp expects a Vector4 as first argument");
    }
    
    validateNumeric(args[1], "Vector4.lerp");
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    double t = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    double w1 = self->get("w").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    double w2 = other->get("w").asFloat();
    
    return Value(createVector4(
        x1 + t * (x2 - x1),
        y1 + t * (y2 - y1),
        z1 + t * (z2 - z1),
        w1 + t * (w2 - w1)
    ));
}

Value vector4ToString(const std::vector<Value>& args) {
    validateArgs(args, 0, "Vector4.toString");
    
    ObjectPtr self = args.at(0).asObject();
    
    double x = self->get("x").asFloat();
    double y = self->get("y").asFloat();
    double z = self->get("z").asFloat();
    double w = self->get("w").asFloat();
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Vector4(" << x << ", " << y << ", " << z << ", " << w << ")";
    
    return Value(ss.str());
}

Value vector4Equals(const std::vector<Value>& args) {
    validateArgs(args, 1, "Vector4.equals");
    
    if (!isVector4(args[0])) {
        return Value(false);
    }
    
    ObjectPtr self = args.at(0).asObject();
    ObjectPtr other = args.at(1).asObject();
    
    double x1 = self->get("x").asFloat();
    double y1 = self->get("y").asFloat();
    double z1 = self->get("z").asFloat();
    double w1 = self->get("w").asFloat();
    
    double x2 = other->get("x").asFloat();
    double y2 = other->get("y").asFloat();
    double z2 = other->get("z").asFloat();
    double w2 = other->get("w").asFloat();
    
    // Use epsilon for floating point comparison
    constexpr double epsilon = 1e-10;
    return Value(
        std::abs(x1 - x2) < epsilon && 
        std::abs(y1 - y2) < epsilon && 
        std::abs(z1 - z2) < epsilon &&
        std::abs(w1 - w2) < epsilon
    );
}

} // namespace vector
} // namespace stdlib
} // namespace astra