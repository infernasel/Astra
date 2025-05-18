#include "math_module.h"
#include "../vm/object.h"
#include <stdexcept>
#include <algorithm>
#include <chrono>

namespace astra {
namespace stdlib {
namespace math {

// Random number generator
static std::mt19937 rng;
static bool rng_initialized = false;

void initialize(VM& vm) {
    // Initialize random number generator if not already initialized
    if (!rng_initialized) {
        auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        rng.seed(static_cast<unsigned int>(seed));
        rng_initialized = true;
    }

    // Create Math module object
    ObjectPtr mathModule = vm.createObject();
    
    // Add constants
    mathModule->set("PI", Value(PI));
    mathModule->set("E", Value(E));
    mathModule->set("SQRT2", Value(SQRT2));
    mathModule->set("INFINITY", Value(INFINITY_VALUE));
    mathModule->set("NAN", Value(NAN_VALUE));
    
    // Add basic math functions
    mathModule->set("abs", Value::createNativeFunction(abs));
    mathModule->set("sign", Value::createNativeFunction(sign));
    mathModule->set("min", Value::createNativeFunction(min));
    mathModule->set("max", Value::createNativeFunction(max));
    mathModule->set("floor", Value::createNativeFunction(floor));
    mathModule->set("ceil", Value::createNativeFunction(ceil));
    mathModule->set("round", Value::createNativeFunction(round));
    mathModule->set("trunc", Value::createNativeFunction(trunc));
    
    // Add exponential and logarithmic functions
    mathModule->set("exp", Value::createNativeFunction(exp));
    mathModule->set("log", Value::createNativeFunction(log));
    mathModule->set("log10", Value::createNativeFunction(log10));
    mathModule->set("log2", Value::createNativeFunction(log2));
    mathModule->set("pow", Value::createNativeFunction(pow));
    mathModule->set("sqrt", Value::createNativeFunction(sqrt));
    mathModule->set("cbrt", Value::createNativeFunction(cbrt));
    
    // Add trigonometric functions
    mathModule->set("sin", Value::createNativeFunction(sin));
    mathModule->set("cos", Value::createNativeFunction(cos));
    mathModule->set("tan", Value::createNativeFunction(tan));
    mathModule->set("asin", Value::createNativeFunction(asin));
    mathModule->set("acos", Value::createNativeFunction(acos));
    mathModule->set("atan", Value::createNativeFunction(atan));
    mathModule->set("atan2", Value::createNativeFunction(atan2));
    
    // Add hyperbolic functions
    mathModule->set("sinh", Value::createNativeFunction(sinh));
    mathModule->set("cosh", Value::createNativeFunction(cosh));
    mathModule->set("tanh", Value::createNativeFunction(tanh));
    mathModule->set("asinh", Value::createNativeFunction(asinh));
    mathModule->set("acosh", Value::createNativeFunction(acosh));
    mathModule->set("atanh", Value::createNativeFunction(atanh));
    
    // Add angle conversion functions
    mathModule->set("degrees", Value::createNativeFunction(degrees));
    mathModule->set("radians", Value::createNativeFunction(radians));
    
    // Add random number generation functions
    mathModule->set("random", Value::createNativeFunction(random));
    mathModule->set("randomInt", Value::createNativeFunction(randomInt));
    mathModule->set("randomFloat", Value::createNativeFunction(randomFloat));
    mathModule->set("setSeed", Value::createNativeFunction(setSeed));
    
    // Add additional functions
    mathModule->set("clamp", Value::createNativeFunction(clamp));
    mathModule->set("lerp", Value::createNativeFunction(lerp));
    mathModule->set("smoothStep", Value::createNativeFunction(smoothStep));
    mathModule->set("isNaN", Value::createNativeFunction(isNaN));
    mathModule->set("isInfinite", Value::createNativeFunction(isInfinite));
    mathModule->set("isFinite", Value::createNativeFunction(isFinite));
    
    // Register the Math module in the global scope
    vm.setGlobal("Math", Value(mathModule));
}

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

// Basic math functions
Value abs(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.abs");
    validateNumeric(args[0], "Math.abs");
    
    if (args[0].isInt()) {
        return Value(std::abs(args[0].asInt()));
    } else {
        return Value(std::abs(args[0].asFloat()));
    }
}

Value sign(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.sign");
    validateNumeric(args[0], "Math.sign");
    
    if (args[0].isInt()) {
        int64_t val = args[0].asInt();
        return Value(val > 0 ? 1 : (val < 0 ? -1 : 0));
    } else {
        double val = args[0].asFloat();
        return Value(val > 0.0 ? 1.0 : (val < 0.0 ? -1.0 : 0.0));
    }
}

Value min(const std::vector<Value>& args) {
    if (args.size() < 1) {
        throw std::runtime_error("Math.min expects at least 1 argument");
    }
    
    Value result = args[0];
    for (size_t i = 1; i < args.size(); ++i) {
        validateNumeric(args[i], "Math.min");
        
        if (result.isInt() && args[i].isInt()) {
            if (args[i].asInt() < result.asInt()) {
                result = args[i];
            }
        } else {
            double a = result.isInt() ? static_cast<double>(result.asInt()) : result.asFloat();
            double b = args[i].isInt() ? static_cast<double>(args[i].asInt()) : args[i].asFloat();
            if (b < a) {
                result = Value(b);
            }
        }
    }
    
    return result;
}

Value max(const std::vector<Value>& args) {
    if (args.size() < 1) {
        throw std::runtime_error("Math.max expects at least 1 argument");
    }
    
    Value result = args[0];
    for (size_t i = 1; i < args.size(); ++i) {
        validateNumeric(args[i], "Math.max");
        
        if (result.isInt() && args[i].isInt()) {
            if (args[i].asInt() > result.asInt()) {
                result = args[i];
            }
        } else {
            double a = result.isInt() ? static_cast<double>(result.asInt()) : result.asFloat();
            double b = args[i].isInt() ? static_cast<double>(args[i].asInt()) : args[i].asFloat();
            if (b > a) {
                result = Value(b);
            }
        }
    }
    
    return result;
}

Value floor(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.floor");
    validateNumeric(args[0], "Math.floor");
    
    if (args[0].isInt()) {
        return args[0]; // Integer already floored
    } else {
        return Value(std::floor(args[0].asFloat()));
    }
}

Value ceil(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.ceil");
    validateNumeric(args[0], "Math.ceil");
    
    if (args[0].isInt()) {
        return args[0]; // Integer already ceiled
    } else {
        return Value(std::ceil(args[0].asFloat()));
    }
}

Value round(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.round");
    validateNumeric(args[0], "Math.round");
    
    if (args[0].isInt()) {
        return args[0]; // Integer already rounded
    } else {
        return Value(std::round(args[0].asFloat()));
    }
}

Value trunc(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.trunc");
    validateNumeric(args[0], "Math.trunc");
    
    if (args[0].isInt()) {
        return args[0]; // Integer already truncated
    } else {
        return Value(std::trunc(args[0].asFloat()));
    }
}

// Exponential and logarithmic functions
Value exp(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.exp");
    validateNumeric(args[0], "Math.exp");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::exp(value));
}

Value log(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.log");
    validateNumeric(args[0], "Math.log");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::log(value));
}

Value log10(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.log10");
    validateNumeric(args[0], "Math.log10");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::log10(value));
}

Value log2(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.log2");
    validateNumeric(args[0], "Math.log2");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::log2(value));
}

Value pow(const std::vector<Value>& args) {
    validateArgs(args, 2, "Math.pow");
    validateNumeric(args[0], "Math.pow");
    validateNumeric(args[1], "Math.pow");
    
    double base = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double exponent = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    return Value(std::pow(base, exponent));
}

Value sqrt(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.sqrt");
    validateNumeric(args[0], "Math.sqrt");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::sqrt(value));
}

Value cbrt(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.cbrt");
    validateNumeric(args[0], "Math.cbrt");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::cbrt(value));
}

// Trigonometric functions
Value sin(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.sin");
    validateNumeric(args[0], "Math.sin");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::sin(value));
}

Value cos(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.cos");
    validateNumeric(args[0], "Math.cos");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::cos(value));
}

Value tan(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.tan");
    validateNumeric(args[0], "Math.tan");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::tan(value));
}

Value asin(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.asin");
    validateNumeric(args[0], "Math.asin");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::asin(value));
}

Value acos(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.acos");
    validateNumeric(args[0], "Math.acos");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::acos(value));
}

Value atan(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.atan");
    validateNumeric(args[0], "Math.atan");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::atan(value));
}

Value atan2(const std::vector<Value>& args) {
    validateArgs(args, 2, "Math.atan2");
    validateNumeric(args[0], "Math.atan2");
    validateNumeric(args[1], "Math.atan2");
    
    double y = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double x = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    return Value(std::atan2(y, x));
}

// Hyperbolic functions
Value sinh(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.sinh");
    validateNumeric(args[0], "Math.sinh");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::sinh(value));
}

Value cosh(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.cosh");
    validateNumeric(args[0], "Math.cosh");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::cosh(value));
}

Value tanh(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.tanh");
    validateNumeric(args[0], "Math.tanh");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::tanh(value));
}

Value asinh(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.asinh");
    validateNumeric(args[0], "Math.asinh");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::asinh(value));
}

Value acosh(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.acosh");
    validateNumeric(args[0], "Math.acosh");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::acosh(value));
}

Value atanh(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.atanh");
    validateNumeric(args[0], "Math.atanh");
    
    double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(std::atanh(value));
}

// Angle conversion
Value degrees(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.degrees");
    validateNumeric(args[0], "Math.degrees");
    
    double radians = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(radians * 180.0 / PI);
}

Value radians(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.radians");
    validateNumeric(args[0], "Math.radians");
    
    double degrees = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    return Value(degrees * PI / 180.0);
}

// Random number generation
Value random(const std::vector<Value>& args) {
    validateArgs(args, 0, "Math.random");
    
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return Value(dist(rng));
}

Value randomInt(const std::vector<Value>& args) {
    validateArgs(args, 2, "Math.randomInt");
    validateNumeric(args[0], "Math.randomInt");
    validateNumeric(args[1], "Math.randomInt");
    
    int64_t min = args[0].isInt() ? args[0].asInt() : static_cast<int64_t>(args[0].asFloat());
    int64_t max = args[1].isInt() ? args[1].asInt() : static_cast<int64_t>(args[1].asFloat());
    
    if (min > max) {
        std::swap(min, max);
    }
    
    std::uniform_int_distribution<int64_t> dist(min, max);
    return Value(dist(rng));
}

Value randomFloat(const std::vector<Value>& args) {
    validateArgs(args, 2, "Math.randomFloat");
    validateNumeric(args[0], "Math.randomFloat");
    validateNumeric(args[1], "Math.randomFloat");
    
    double min = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double max = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    
    if (min > max) {
        std::swap(min, max);
    }
    
    std::uniform_real_distribution<double> dist(min, max);
    return Value(dist(rng));
}

Value setSeed(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.setSeed");
    validateNumeric(args[0], "Math.setSeed");
    
    unsigned int seed = args[0].isInt() ? 
                        static_cast<unsigned int>(args[0].asInt()) : 
                        static_cast<unsigned int>(args[0].asFloat());
    
    rng.seed(seed);
    rng_initialized = true;
    
    return Value(); // Return undefined
}

// Additional functions
Value clamp(const std::vector<Value>& args) {
    validateArgs(args, 3, "Math.clamp");
    validateNumeric(args[0], "Math.clamp");
    validateNumeric(args[1], "Math.clamp");
    validateNumeric(args[2], "Math.clamp");
    
    if (args[0].isInt() && args[1].isInt() && args[2].isInt()) {
        int64_t value = args[0].asInt();
        int64_t min = args[1].asInt();
        int64_t max = args[2].asInt();
        
        return Value(std::clamp(value, min, max));
    } else {
        double value = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
        double min = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
        double max = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
        
        return Value(std::clamp(value, min, max));
    }
}

Value lerp(const std::vector<Value>& args) {
    validateArgs(args, 3, "Math.lerp");
    validateNumeric(args[0], "Math.lerp");
    validateNumeric(args[1], "Math.lerp");
    validateNumeric(args[2], "Math.lerp");
    
    double a = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double b = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    double t = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
    
    return Value(a + t * (b - a));
}

Value smoothStep(const std::vector<Value>& args) {
    validateArgs(args, 3, "Math.smoothStep");
    validateNumeric(args[0], "Math.smoothStep");
    validateNumeric(args[1], "Math.smoothStep");
    validateNumeric(args[2], "Math.smoothStep");
    
    double edge0 = args[0].isInt() ? static_cast<double>(args[0].asInt()) : args[0].asFloat();
    double edge1 = args[1].isInt() ? static_cast<double>(args[1].asInt()) : args[1].asFloat();
    double x = args[2].isInt() ? static_cast<double>(args[2].asInt()) : args[2].asFloat();
    
    // Clamp x to [0, 1]
    x = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    
    // Evaluate polynomial
    return Value(x * x * (3 - 2 * x));
}

Value isNaN(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.isNaN");
    
    if (!args[0].isFloat()) {
        return Value(false);
    }
    
    return Value(std::isnan(args[0].asFloat()));
}

Value isInfinite(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.isInfinite");
    
    if (!args[0].isFloat()) {
        return Value(false);
    }
    
    return Value(std::isinf(args[0].asFloat()));
}

Value isFinite(const std::vector<Value>& args) {
    validateArgs(args, 1, "Math.isFinite");
    
    if (args[0].isInt()) {
        return Value(true); // Integers are always finite
    }
    
    if (!args[0].isFloat()) {
        return Value(false);
    }
    
    return Value(std::isfinite(args[0].asFloat()));
}

} // namespace math
} // namespace stdlib
} // namespace astra