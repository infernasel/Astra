/**
 * ASTRA Programming Language Interpreter
 * Value class definition
 */

#ifndef ASTRA_VALUE_H
#define ASTRA_VALUE_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <stdexcept>
#include <cmath>

namespace astra {

// Forward declarations
class Object;
class Array;
class Function;
class NativeFunction;
class Task;
class VM;
class Environment;
class Error;
class Vector2;
class Vector3;
class Vector4;
class Quaternion;
class Matrix;
class Value;

// Type aliases
using ObjectPtr = std::shared_ptr<Object>;

/**
 * Value types
 */
enum class ValueType {
    Null,
    Boolean,
    Integer,
    Float,
    String,
    Array,
    Object,
    Function,
    NativeFunction,
    Task,
    Vector2,
    Vector3,
    Vector4,
    Quaternion,
    Matrix,
    Error
};

/**
 * Value class
 */
class Value {
private:
    ValueType type;
    union {
        bool boolValue;
        int64_t intValue;
        double floatValue;
        void* ptrValue;
    };

public:
    // Constructors
    Value() : type(ValueType::Null), intValue(0) {}
    Value(bool value) : type(ValueType::Boolean), boolValue(value) {}
    Value(int value) : type(ValueType::Integer), intValue(value) {}
    Value(int64_t value) : type(ValueType::Integer), intValue(value) {}
    Value(double value) : type(ValueType::Float), floatValue(value) {}
    
    // String constructor
    Value(const std::string& value) : type(ValueType::String) {
        ptrValue = new std::string(value);
    }
    
    Value(const char* value) : type(ValueType::String) {
        ptrValue = new std::string(value);
    }
    
    // Forward declarations of other constructors
    Value(const Error& error);
    Value(const Function& func);
    Value(const NativeFunction& func);
    Value(const Object& obj);
    Value(const Array& arr);
    Value(const Vector2& vec);
    Value(const Vector3& vec);
    Value(const Vector4& vec);
    Value(const Quaternion& quat);
    Value(const Matrix& mat);
    
    // Destructor
    ~Value();
    
    // Copy constructor
    Value(const Value& other);
    
    // Assignment operator
    Value& operator=(const Value& other);
    
    // Type checking
    bool isNull() const { return type == ValueType::Null; }
    bool isBoolean() const { return type == ValueType::Boolean; }
    bool isInteger() const { return type == ValueType::Integer; }
    bool isFloat() const { return type == ValueType::Float; }
    bool isNumber() const { return isInteger() || isFloat(); }
    bool isString() const { return type == ValueType::String; }
    bool isArray() const { return type == ValueType::Array; }
    bool isObject() const { return type == ValueType::Object; }
    bool isFunction() const { return type == ValueType::Function; }
    bool isNativeFunction() const { return type == ValueType::NativeFunction; }
    bool isTask() const { return type == ValueType::Task; }
    bool isVector2() const { return type == ValueType::Vector2; }
    bool isVector3() const { return type == ValueType::Vector3; }
    bool isVector4() const { return type == ValueType::Vector4; }
    bool isQuaternion() const { return type == ValueType::Quaternion; }
    bool isMatrix() const { return type == ValueType::Matrix; }
    bool isError() const { return type == ValueType::Error; }
    
    // Value getters
    bool asBool() const { 
        if (!isBoolean()) throw std::runtime_error("Value is not a boolean");
        return boolValue; 
    }
    
    int asInt() const { 
        if (!isInteger()) throw std::runtime_error("Value is not an integer");
        return static_cast<int>(intValue); 
    }
    
    int64_t asInt64() const { 
        if (!isInteger()) throw std::runtime_error("Value is not an integer");
        return intValue; 
    }
    
    double asFloat() const { 
        if (isInteger()) return static_cast<double>(intValue);
        if (!isFloat()) throw std::runtime_error("Value is not a number");
        return floatValue; 
    }
    
    std::string asString() const { 
        if (!isString()) throw std::runtime_error("Value is not a string");
        return *reinterpret_cast<std::string*>(ptrValue); 
    }
    
    // Forward declarations of other getters
    Object& asObject() const;
    Array& asArray() const;
    Function& asFunction() const;
    NativeFunction& asNativeFunction() const;
    Error& asError() const;
    Vector2& asVector2() const;
    Vector3& asVector3() const;
    Vector4& asVector4() const;
    Quaternion& asQuaternion() const;
    Matrix& asMatrix() const;
    
    std::string toString() const;
};

/**
 * Vector2 class
 */
class Vector2 {
public:
    double x, y;
    
    Vector2() : x(0), y(0) {}
    Vector2(double x_, double y_) : x(x_), y(y_) {}
    
    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }
    
    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }
    
    Vector2 operator*(double scalar) const {
        return Vector2(x * scalar, y * scalar);
    }
    
    Vector2 operator/(double scalar) const {
        return Vector2(x / scalar, y / scalar);
    }
    
    double length() const {
        return sqrt(x * x + y * y);
    }
    
    Vector2 normalize() const {
        double len = length();
        if (len == 0) return Vector2();
        return Vector2(x / len, y / len);
    }
    
    double dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }
    
    std::string toString() const {
        return "Vector2(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }
};

/**
 * Vector3 class
 */
class Vector3 {
public:
    double x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3 operator*(double scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    
    Vector3 operator/(double scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }
    
    double length() const {
        return sqrt(x * x + y * y + z * z);
    }
    
    Vector3 normalize() const {
        double len = length();
        if (len == 0) return Vector3();
        return Vector3(x / len, y / len, z / len);
    }
    
    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    
    std::string toString() const {
        return "Vector3(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
    }
};

/**
 * Vector4 class
 */
class Vector4 {
public:
    double x, y, z, w;
    
    Vector4() : x(0), y(0), z(0), w(0) {}
    Vector4(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
    
    Vector4 operator+(const Vector4& other) const {
        return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
    }
    
    Vector4 operator-(const Vector4& other) const {
        return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
    }
    
    Vector4 operator*(double scalar) const {
        return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
    }
    
    Vector4 operator/(double scalar) const {
        return Vector4(x / scalar, y / scalar, z / scalar, w / scalar);
    }
    
    double length() const {
        return sqrt(x * x + y * y + z * z + w * w);
    }
    
    Vector4 normalize() const {
        double len = length();
        if (len == 0) return Vector4();
        return Vector4(x / len, y / len, z / len, w / len);
    }
    
    double dot(const Vector4& other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }
    
    std::string toString() const {
        return "Vector4(" + std::to_string(x) + ", " + std::to_string(y) + ", " + 
               std::to_string(z) + ", " + std::to_string(w) + ")";
    }
};

/**
 * Quaternion class
 */
class Quaternion {
public:
    double w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }
    
    Vector3 rotate(const Vector3& v) const {
        Quaternion p(0, v.x, v.y, v.z);
        Quaternion q = *this;
        Quaternion qInv = q.conjugate();
        Quaternion result = q * p * qInv;
        return Vector3(result.x, result.y, result.z);
    }
    
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    double length() const {
        return sqrt(w * w + x * x + y * y + z * z);
    }
    
    Quaternion normalize() const {
        double len = length();
        if (len == 0) return Quaternion();
        return Quaternion(w / len, x / len, y / len, z / len);
    }
    
    static Quaternion fromEuler(double pitch, double yaw, double roll) {
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        
        return Quaternion(
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr
        );
    }
    
    std::string toString() const {
        return "Quaternion(" + std::to_string(w) + ", " + std::to_string(x) + ", " + 
               std::to_string(y) + ", " + std::to_string(z) + ")";
    }
};

/**
 * Matrix class (4x4)
 */
class Matrix {
public:
    double m[16];
    
    Matrix() {
        for (int i = 0; i < 16; i++) {
            m[i] = (i % 5 == 0) ? 1.0 : 0.0; // Identity matrix
        }
    }
    
    Matrix operator*(const Matrix& other) const {
        Matrix result;
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                result.m[col * 4 + row] = 0;
                for (int i = 0; i < 4; i++) {
                    result.m[col * 4 + row] += m[i * 4 + row] * other.m[col * 4 + i];
                }
            }
        }
        return result;
    }
    
    Vector4 operator*(const Vector4& v) const {
        return Vector4(
            m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12] * v.w,
            m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13] * v.w,
            m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14] * v.w,
            m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15] * v.w
        );
    }
    
    static Matrix translation(double x, double y, double z) {
        Matrix result;
        result.m[12] = x;
        result.m[13] = y;
        result.m[14] = z;
        return result;
    }
    
    static Matrix rotation(const Quaternion& q) {
        Matrix result;
        double xx = q.x * q.x;
        double xy = q.x * q.y;
        double xz = q.x * q.z;
        double xw = q.x * q.w;
        double yy = q.y * q.y;
        double yz = q.y * q.z;
        double yw = q.y * q.w;
        double zz = q.z * q.z;
        double zw = q.z * q.w;
        
        result.m[0] = 1 - 2 * (yy + zz);
        result.m[1] = 2 * (xy - zw);
        result.m[2] = 2 * (xz + yw);
        
        result.m[4] = 2 * (xy + zw);
        result.m[5] = 1 - 2 * (xx + zz);
        result.m[6] = 2 * (yz - xw);
        
        result.m[8] = 2 * (xz - yw);
        result.m[9] = 2 * (yz + xw);
        result.m[10] = 1 - 2 * (xx + yy);
        
        return result;
    }
    
    static Matrix scale(double x, double y, double z) {
        Matrix result;
        result.m[0] = x;
        result.m[5] = y;
        result.m[10] = z;
        return result;
    }
    
    static Matrix perspective(double fov, double aspect, double near, double far) {
        Matrix result;
        double f = 1.0 / tan(fov * 0.5);
        double nf = 1.0 / (near - far);
        
        result.m[0] = f / aspect;
        result.m[5] = f;
        result.m[10] = (far + near) * nf;
        result.m[11] = -1;
        result.m[14] = 2 * far * near * nf;
        result.m[15] = 0;
        
        return result;
    }
    
    std::string toString() const {
        std::string result = "Matrix(\n";
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                result += std::to_string(m[col * 4 + row]) + " ";
            }
            result += "\n";
        }
        result += ")";
        return result;
    }
};

/**
 * Error class
 */
class Error {
private:
    std::string message;
    
public:
    Error(const std::string& msg) : message(msg) {}
    
    std::string getMessage() const {
        return message;
    }
};

/**
 * Environment class for variable scoping
 */
class Environment {
public:
    std::unordered_map<std::string, Value> variables;
    std::shared_ptr<Environment> outer;
    
    Environment() : outer(nullptr) {}
    Environment(std::shared_ptr<Environment> outer_) : outer(outer_) {}
    
    void set(const std::string& name, const Value& value);
    bool has(const std::string& name) const;
    Value get(const std::string& name) const;
};

/**
 * Function class
 */
class Function {
private:
    std::string name;
    std::string body;
    std::shared_ptr<Environment> env;
    
public:
    Function(const std::string& name_, const std::string& body_, std::shared_ptr<Environment> env_)
        : name(name_), body(body_), env(env_) {}
    
    std::string getName() const {
        return name;
    }
    
    std::string getBody() const {
        return body;
    }
    
    std::shared_ptr<Environment> getEnvironment() const {
        return env;
    }
};

/**
 * NativeFunction class
 */
class NativeFunction {
private:
    std::string name;
    std::function<Value(const std::vector<Value>&)> func;
    
public:
    NativeFunction(const std::string& name_, std::function<Value(const std::vector<Value>&)> func_)
        : name(name_), func(func_) {}
    
    std::string getName() const {
        return name;
    }
    
    Value call(const std::vector<Value>& args) const {
        return func(args);
    }
};

/**
 * Object class
 */
class Object {
private:
    std::unordered_map<std::string, Value> properties;
    
public:
    Object() {}
    
    void set(const std::string& key, const Value& value) {
        properties[key] = value;
    }
    
    Value get(const std::string& key) const {
        auto it = properties.find(key);
        if (it != properties.end()) {
            return it->second;
        }
        return Value(); // Return null
    }
    
    bool has(const std::string& key) const {
        return properties.find(key) != properties.end();
    }
    
    std::unordered_map<std::string, Value>& getProperties() {
        return properties;
    }
};

/**
 * Array class
 */
class Array {
private:
    std::vector<Value> elements;
    
public:
    Array() {}
    Array(const std::vector<Value>& elements_) : elements(elements_) {}
    
    void push(const Value& value) {
        elements.push_back(value);
    }
    
    Value pop() {
        if (elements.empty()) {
            return Value(); // Return null
        }
        Value last = elements.back();
        elements.pop_back();
        return last;
    }
    
    Value get(size_t index) const {
        if (index >= elements.size()) {
            return Value(); // Return null
        }
        return elements[index];
    }
    
    void set(size_t index, const Value& value) {
        if (index >= elements.size()) {
            elements.resize(index + 1);
        }
        elements[index] = value;
    }
    
    size_t size() const {
        return elements.size();
    }
    
    std::vector<Value>& getElements() {
        return elements;
    }
};

/**
 * Task class
 */
class Task {
private:
    std::string name;
    std::function<void()> callback;
    bool isRunning;
    
public:
    Task(const std::string& name_, std::function<void()> callback_)
        : name(name_), callback(callback_), isRunning(false) {}
    
    std::string getName() const {
        return name;
    }
    
    void run() {
        callback();
    }
    
    bool getIsRunning() const {
        return isRunning;
    }
    
    void setIsRunning(bool running) {
        isRunning = running;
    }
};

// Implementation of Value constructors that depend on other classes
inline Value::Value(const Error& error) : type(ValueType::Error) {
    ptrValue = new Error(error);
}

inline Value::Value(const Function& func) : type(ValueType::Function) {
    ptrValue = new Function(func);
}

inline Value::Value(const NativeFunction& func) : type(ValueType::NativeFunction) {
    ptrValue = new NativeFunction(func);
}

inline Value::Value(const Object& obj) : type(ValueType::Object) {
    ptrValue = new Object(obj);
}

inline Value::Value(const Array& arr) : type(ValueType::Array) {
    ptrValue = new Array(arr);
}

inline Value::Value(const Vector2& vec) : type(ValueType::Vector2) {
    ptrValue = new Vector2(vec);
}

inline Value::Value(const Vector3& vec) : type(ValueType::Vector3) {
    ptrValue = new Vector3(vec);
}

inline Value::Value(const Vector4& vec) : type(ValueType::Vector4) {
    ptrValue = new Vector4(vec);
}

inline Value::Value(const Quaternion& quat) : type(ValueType::Quaternion) {
    ptrValue = new Quaternion(quat);
}

inline Value::Value(const Matrix& mat) : type(ValueType::Matrix) {
    ptrValue = new Matrix(mat);
}

// Implementation of Value destructor
inline Value::~Value() {
    if (type == ValueType::String) {
        delete reinterpret_cast<std::string*>(ptrValue);
    } else if (type == ValueType::Object) {
        delete reinterpret_cast<Object*>(ptrValue);
    } else if (type == ValueType::Array) {
        delete reinterpret_cast<Array*>(ptrValue);
    } else if (type == ValueType::Function) {
        delete reinterpret_cast<Function*>(ptrValue);
    } else if (type == ValueType::NativeFunction) {
        delete reinterpret_cast<NativeFunction*>(ptrValue);
    } else if (type == ValueType::Error) {
        delete reinterpret_cast<Error*>(ptrValue);
    } else if (type == ValueType::Vector2) {
        delete reinterpret_cast<Vector2*>(ptrValue);
    } else if (type == ValueType::Vector3) {
        delete reinterpret_cast<Vector3*>(ptrValue);
    } else if (type == ValueType::Vector4) {
        delete reinterpret_cast<Vector4*>(ptrValue);
    } else if (type == ValueType::Quaternion) {
        delete reinterpret_cast<Quaternion*>(ptrValue);
    } else if (type == ValueType::Matrix) {
        delete reinterpret_cast<Matrix*>(ptrValue);
    }
}

// Implementation of Value copy constructor
inline Value::Value(const Value& other) : type(other.type) {
    switch (type) {
        case ValueType::Null:
            intValue = 0;
            break;
        case ValueType::Boolean:
            boolValue = other.boolValue;
            break;
        case ValueType::Integer:
            intValue = other.intValue;
            break;
        case ValueType::Float:
            floatValue = other.floatValue;
            break;
        case ValueType::String:
            ptrValue = new std::string(*reinterpret_cast<std::string*>(other.ptrValue));
            break;
        case ValueType::Object:
            ptrValue = new Object(*reinterpret_cast<Object*>(other.ptrValue));
            break;
        case ValueType::Array:
            ptrValue = new Array(*reinterpret_cast<Array*>(other.ptrValue));
            break;
        case ValueType::Function:
            ptrValue = new Function(*reinterpret_cast<Function*>(other.ptrValue));
            break;
        case ValueType::NativeFunction:
            ptrValue = new NativeFunction(*reinterpret_cast<NativeFunction*>(other.ptrValue));
            break;
        case ValueType::Error:
            ptrValue = new Error(*reinterpret_cast<Error*>(other.ptrValue));
            break;
        case ValueType::Vector2:
            ptrValue = new Vector2(*reinterpret_cast<Vector2*>(other.ptrValue));
            break;
        case ValueType::Vector3:
            ptrValue = new Vector3(*reinterpret_cast<Vector3*>(other.ptrValue));
            break;
        case ValueType::Vector4:
            ptrValue = new Vector4(*reinterpret_cast<Vector4*>(other.ptrValue));
            break;
        case ValueType::Quaternion:
            ptrValue = new Quaternion(*reinterpret_cast<Quaternion*>(other.ptrValue));
            break;
        case ValueType::Matrix:
            ptrValue = new Matrix(*reinterpret_cast<Matrix*>(other.ptrValue));
            break;
        default:
            ptrValue = nullptr;
            break;
    }
}

// Implementation of Value assignment operator
inline Value& Value::operator=(const Value& other) {
    if (this != &other) {
        // Clean up old value
        if (type == ValueType::String) {
            delete reinterpret_cast<std::string*>(ptrValue);
        } else if (type == ValueType::Object) {
            delete reinterpret_cast<Object*>(ptrValue);
        } else if (type == ValueType::Array) {
            delete reinterpret_cast<Array*>(ptrValue);
        } else if (type == ValueType::Function) {
            delete reinterpret_cast<Function*>(ptrValue);
        } else if (type == ValueType::NativeFunction) {
            delete reinterpret_cast<NativeFunction*>(ptrValue);
        } else if (type == ValueType::Error) {
            delete reinterpret_cast<Error*>(ptrValue);
        } else if (type == ValueType::Vector2) {
            delete reinterpret_cast<Vector2*>(ptrValue);
        } else if (type == ValueType::Vector3) {
            delete reinterpret_cast<Vector3*>(ptrValue);
        } else if (type == ValueType::Vector4) {
            delete reinterpret_cast<Vector4*>(ptrValue);
        } else if (type == ValueType::Quaternion) {
            delete reinterpret_cast<Quaternion*>(ptrValue);
        } else if (type == ValueType::Matrix) {
            delete reinterpret_cast<Matrix*>(ptrValue);
        }
        
        // Copy new value
        type = other.type;
        switch (type) {
            case ValueType::Null:
                intValue = 0;
                break;
            case ValueType::Boolean:
                boolValue = other.boolValue;
                break;
            case ValueType::Integer:
                intValue = other.intValue;
                break;
            case ValueType::Float:
                floatValue = other.floatValue;
                break;
            case ValueType::String:
                ptrValue = new std::string(*reinterpret_cast<std::string*>(other.ptrValue));
                break;
            case ValueType::Object:
                ptrValue = new Object(*reinterpret_cast<Object*>(other.ptrValue));
                break;
            case ValueType::Array:
                ptrValue = new Array(*reinterpret_cast<Array*>(other.ptrValue));
                break;
            case ValueType::Function:
                ptrValue = new Function(*reinterpret_cast<Function*>(other.ptrValue));
                break;
            case ValueType::NativeFunction:
                ptrValue = new NativeFunction(*reinterpret_cast<NativeFunction*>(other.ptrValue));
                break;
            case ValueType::Error:
                ptrValue = new Error(*reinterpret_cast<Error*>(other.ptrValue));
                break;
            case ValueType::Vector2:
                ptrValue = new Vector2(*reinterpret_cast<Vector2*>(other.ptrValue));
                break;
            case ValueType::Vector3:
                ptrValue = new Vector3(*reinterpret_cast<Vector3*>(other.ptrValue));
                break;
            case ValueType::Vector4:
                ptrValue = new Vector4(*reinterpret_cast<Vector4*>(other.ptrValue));
                break;
            case ValueType::Quaternion:
                ptrValue = new Quaternion(*reinterpret_cast<Quaternion*>(other.ptrValue));
                break;
            case ValueType::Matrix:
                ptrValue = new Matrix(*reinterpret_cast<Matrix*>(other.ptrValue));
                break;
            default:
                ptrValue = nullptr;
                break;
        }
    }
    return *this;
}

// Implementation of Value getters that depend on other classes
inline Object& Value::asObject() const {
    if (!isObject()) throw std::runtime_error("Value is not an object");
    return *reinterpret_cast<Object*>(ptrValue);
}

inline Array& Value::asArray() const {
    if (!isArray()) throw std::runtime_error("Value is not an array");
    return *reinterpret_cast<Array*>(ptrValue);
}

inline Function& Value::asFunction() const {
    if (!isFunction()) throw std::runtime_error("Value is not a function");
    return *reinterpret_cast<Function*>(ptrValue);
}

inline NativeFunction& Value::asNativeFunction() const {
    if (!isNativeFunction()) throw std::runtime_error("Value is not a native function");
    return *reinterpret_cast<NativeFunction*>(ptrValue);
}

inline Error& Value::asError() const {
    if (!isError()) throw std::runtime_error("Value is not an error");
    return *reinterpret_cast<Error*>(ptrValue);
}

inline Vector2& Value::asVector2() const {
    if (!isVector2()) throw std::runtime_error("Value is not a Vector2");
    return *reinterpret_cast<Vector2*>(ptrValue);
}

inline Vector3& Value::asVector3() const {
    if (!isVector3()) throw std::runtime_error("Value is not a Vector3");
    return *reinterpret_cast<Vector3*>(ptrValue);
}

inline Vector4& Value::asVector4() const {
    if (!isVector4()) throw std::runtime_error("Value is not a Vector4");
    return *reinterpret_cast<Vector4*>(ptrValue);
}

inline Quaternion& Value::asQuaternion() const {
    if (!isQuaternion()) throw std::runtime_error("Value is not a Quaternion");
    return *reinterpret_cast<Quaternion*>(ptrValue);
}

inline Matrix& Value::asMatrix() const {
    if (!isMatrix()) throw std::runtime_error("Value is not a Matrix");
    return *reinterpret_cast<Matrix*>(ptrValue);
}

// Implementation of Value::toString()
inline std::string Value::toString() const {
    switch (type) {
        case ValueType::Null: return "null";
        case ValueType::Boolean: return boolValue ? "true" : "false";
        case ValueType::Integer: return std::to_string(intValue);
        case ValueType::Float: return std::to_string(floatValue);
        case ValueType::String: return asString();
        case ValueType::Object: return "[object Object]";
        case ValueType::Array: return "[array]";
        case ValueType::Function: return "[function " + asFunction().getName() + "]";
        case ValueType::NativeFunction: return "[native function " + asNativeFunction().getName() + "]";
        case ValueType::Error: return "Error: " + asError().getMessage();
        case ValueType::Vector2: return asVector2().toString();
        case ValueType::Vector3: return asVector3().toString();
        case ValueType::Vector4: return asVector4().toString();
        case ValueType::Quaternion: return asQuaternion().toString();
        case ValueType::Matrix: return asMatrix().toString();
        default: return "[unknown]";
    }
}

} // namespace astra

#endif // ASTRA_VALUE_H