/**
 * ASTRA Programming Language Interpreter
 * Standard library
 */

#ifndef ASTRA_STDLIB_H
#define ASTRA_STDLIB_H

#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <iostream>

#include "../vm/vm.h"
#include "network_module.h"

namespace astra {
namespace stdlib {

/**
 * Math module
 */
void loadMathModule(VM& vm) {
    auto mathObj = Object();
    
    // Constants
    mathObj.set("PI", Value(M_PI));
    mathObj.set("E", Value(M_E));
    mathObj.set("SQRT2", Value(M_SQRT2));
    mathObj.set("SQRT1_2", Value(M_SQRT1_2));
    mathObj.set("LN2", Value(M_LN2));
    mathObj.set("LN10", Value(M_LN10));
    mathObj.set("LOG2E", Value(M_LOG2E));
    mathObj.set("LOG10E", Value(M_LOG10E));
    
    // Functions
    mathObj.set("abs", Value(NativeFunction("abs", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("abs requires a number argument"));
        }
        
        if (args[0].isInteger()) {
            return Value(std::abs(args[0].asInt()));
        } else {
            return Value(std::abs(args[0].asFloat()));
        }
    })));
    
    mathObj.set("sin", Value(NativeFunction("sin", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("sin requires a number argument"));
        }
        
        return Value(std::sin(args[0].asFloat()));
    })));
    
    mathObj.set("cos", Value(NativeFunction("cos", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("cos requires a number argument"));
        }
        
        return Value(std::cos(args[0].asFloat()));
    })));
    
    mathObj.set("tan", Value(NativeFunction("tan", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("tan requires a number argument"));
        }
        
        return Value(std::tan(args[0].asFloat()));
    })));
    
    mathObj.set("asin", Value(NativeFunction("asin", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("asin requires a number argument"));
        }
        
        return Value(std::asin(args[0].asFloat()));
    })));
    
    mathObj.set("acos", Value(NativeFunction("acos", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("acos requires a number argument"));
        }
        
        return Value(std::acos(args[0].asFloat()));
    })));
    
    mathObj.set("atan", Value(NativeFunction("atan", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("atan requires a number argument"));
        }
        
        return Value(std::atan(args[0].asFloat()));
    })));
    
    mathObj.set("atan2", Value(NativeFunction("atan2", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isNumber() || !args[1].isNumber()) {
            return Value(Error("atan2 requires two number arguments"));
        }
        
        return Value(std::atan2(args[0].asFloat(), args[1].asFloat()));
    })));
    
    mathObj.set("sqrt", Value(NativeFunction("sqrt", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("sqrt requires a number argument"));
        }
        
        return Value(std::sqrt(args[0].asFloat()));
    })));
    
    mathObj.set("pow", Value(NativeFunction("pow", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isNumber() || !args[1].isNumber()) {
            return Value(Error("pow requires two number arguments"));
        }
        
        return Value(std::pow(args[0].asFloat(), args[1].asFloat()));
    })));
    
    mathObj.set("exp", Value(NativeFunction("exp", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("exp requires a number argument"));
        }
        
        return Value(std::exp(args[0].asFloat()));
    })));
    
    mathObj.set("log", Value(NativeFunction("log", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("log requires a number argument"));
        }
        
        return Value(std::log(args[0].asFloat()));
    })));
    
    mathObj.set("log10", Value(NativeFunction("log10", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("log10 requires a number argument"));
        }
        
        return Value(std::log10(args[0].asFloat()));
    })));
    
    mathObj.set("floor", Value(NativeFunction("floor", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("floor requires a number argument"));
        }
        
        return Value(static_cast<int64_t>(std::floor(args[0].asFloat())));
    })));
    
    mathObj.set("ceil", Value(NativeFunction("ceil", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("ceil requires a number argument"));
        }
        
        return Value(static_cast<int64_t>(std::ceil(args[0].asFloat())));
    })));
    
    mathObj.set("round", Value(NativeFunction("round", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isNumber()) {
            return Value(Error("round requires a number argument"));
        }
        
        return Value(static_cast<int64_t>(std::round(args[0].asFloat())));
    })));
    
    mathObj.set("min", Value(NativeFunction("min", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2) {
            return Value(Error("min requires at least two arguments"));
        }
        
        double minVal = args[0].asFloat();
        for (size_t i = 1; i < args.size(); ++i) {
            if (!args[i].isNumber()) {
                return Value(Error("min requires number arguments"));
            }
            minVal = std::min(minVal, args[i].asFloat());
        }
        
        return Value(minVal);
    })));
    
    mathObj.set("max", Value(NativeFunction("max", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2) {
            return Value(Error("max requires at least two arguments"));
        }
        
        double maxVal = args[0].asFloat();
        for (size_t i = 1; i < args.size(); ++i) {
            if (!args[i].isNumber()) {
                return Value(Error("max requires number arguments"));
            }
            maxVal = std::max(maxVal, args[i].asFloat());
        }
        
        return Value(maxVal);
    })));
    
    mathObj.set("random", Value(NativeFunction("random", [](const std::vector<Value>& args) -> Value {
        (void)args; // Prevent unused parameter warning
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<double> dis(0.0, 1.0);
        
        return Value(dis(gen));
    })));
    
    mathObj.set("randomInt", Value(NativeFunction("randomInt", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isNumber() || !args[1].isNumber()) {
            return Value(Error("randomInt requires two number arguments"));
        }
        
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        int64_t min = args[0].asInt();
        int64_t max = args[1].asInt();
        
        if (min > max) {
            std::swap(min, max);
        }
        
        std::uniform_int_distribution<int64_t> dis(min, max);
        
        return Value(dis(gen));
    })));
    
    vm.defineGlobal("Math", Value(mathObj));
}

/**
 * Vector module
 */
void loadVectorModule(VM& vm) {
    // Vector2 constructor
    vm.defineGlobal("Vector2", Value(NativeFunction("Vector2", [](const std::vector<Value>& args) -> Value {
        double x = 0.0;
        double y = 0.0;
        
        if (args.size() >= 1 && args[0].isNumber()) {
            x = args[0].asFloat();
        }
        
        if (args.size() >= 2 && args[1].isNumber()) {
            y = args[1].asFloat();
        }
        
        return Value(Vector2(x, y));
    })));
    
    // Vector3 constructor
    vm.defineGlobal("Vector3", Value(NativeFunction("Vector3", [](const std::vector<Value>& args) -> Value {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        
        if (args.size() >= 1 && args[0].isNumber()) {
            x = args[0].asFloat();
        }
        
        if (args.size() >= 2 && args[1].isNumber()) {
            y = args[1].asFloat();
        }
        
        if (args.size() >= 3 && args[2].isNumber()) {
            z = args[2].asFloat();
        }
        
        return Value(Vector3(x, y, z));
    })));
    
    // Quaternion constructor
    vm.defineGlobal("Quaternion", Value(NativeFunction("Quaternion", [](const std::vector<Value>& args) -> Value {
        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        
        if (args.size() >= 1 && args[0].isNumber()) {
            w = args[0].asFloat();
        }
        
        if (args.size() >= 2 && args[1].isNumber()) {
            x = args[1].asFloat();
        }
        
        if (args.size() >= 3 && args[2].isNumber()) {
            y = args[2].asFloat();
        }
        
        if (args.size() >= 4 && args[3].isNumber()) {
            z = args[3].asFloat();
        }
        
        return Value(Quaternion(w, x, y, z));
    })));
    
    // Matrix constructor
    vm.defineGlobal("Matrix", Value(NativeFunction("Matrix", [](const std::vector<Value>& args) -> Value {
        if (args.empty()) {
            return Value(Matrix());
        }
        
        // TODO: Implement matrix construction from arguments
        
        return Value(Matrix());
    })));
}

/**
 * IO module
 */
void loadIOModule(VM& vm) {
    auto ioObj = Object();
    
    ioObj.set("print", Value(NativeFunction("print", [](const std::vector<Value>& args) -> Value {
        for (size_t i = 0; i < args.size(); ++i) {
            if (i > 0) {
                std::cout << " ";
            }
            std::cout << args[i].toString();
        }
        std::cout << std::endl;
        
        return Value();
    })));
    
    ioObj.set("input", Value(NativeFunction("input", [](const std::vector<Value>& args) -> Value {
        if (!args.empty()) {
            std::cout << args[0].toString();
        }
        
        std::string input;
        std::getline(std::cin, input);
        
        return Value(input);
    })));
    
    ioObj.set("readFile", Value(NativeFunction("readFile", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isString()) {
            return Value(Error("readFile requires a filename argument"));
        }
        
        std::string filename = args[0].asString();
        std::ifstream file(filename);
        
        if (!file) {
            return Value(Error("Could not open file: " + filename));
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        
        return Value(buffer.str());
    })));
    
    ioObj.set("writeFile", Value(NativeFunction("writeFile", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isString()) {
            return Value(Error("writeFile requires filename and content arguments"));
        }
        
        std::string filename = args[0].asString();
        std::string content = args[1].toString();
        
        std::ofstream file(filename);
        
        if (!file) {
            return Value(Error("Could not open file for writing: " + filename));
        }
        
        file << content;
        
        return Value(true);
    })));
    
    ioObj.set("appendFile", Value(NativeFunction("appendFile", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isString()) {
            return Value(Error("appendFile requires filename and content arguments"));
        }
        
        std::string filename = args[0].asString();
        std::string content = args[1].toString();
        
        std::ofstream file(filename, std::ios_base::app);
        
        if (!file) {
            return Value(Error("Could not open file for appending: " + filename));
        }
        
        file << content;
        
        return Value(true);
    })));
    
    vm.defineGlobal("IO", Value(ioObj));
    
    // Also define print as a global function for convenience
    vm.defineGlobal("print", ioObj.get("print"));
}

/**
 * String module
 */
void loadStringModule(VM& vm) {
    auto stringObj = Object();
    
    stringObj.set("length", Value(NativeFunction("length", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isString()) {
            return Value(Error("length requires a string argument"));
        }
        
        return Value(static_cast<int64_t>(args[0].asString().length()));
    })));
    
    stringObj.set("substring", Value(NativeFunction("substring", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 3 || !args[0].isString() || !args[1].isNumber() || !args[2].isNumber()) {
            return Value(Error("substring requires string, start, and end arguments"));
        }
        
        std::string str = args[0].asString();
        int64_t start = args[1].asInt();
        int64_t end = args[2].asInt();
        
        if (start < 0) start = 0;
        if (end > static_cast<int64_t>(str.length())) end = str.length();
        if (start >= end) return Value("");
        
        return Value(str.substr(start, end - start));
    })));
    
    stringObj.set("indexOf", Value(NativeFunction("indexOf", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isString() || !args[1].isString()) {
            return Value(Error("indexOf requires string and substring arguments"));
        }
        
        std::string str = args[0].asString();
        std::string substr = args[1].asString();
        
        size_t pos = str.find(substr);
        if (pos == std::string::npos) {
            return Value(static_cast<int64_t>(-1));
        }
        
        return Value(static_cast<int64_t>(pos));
    })));
    
    stringObj.set("lastIndexOf", Value(NativeFunction("lastIndexOf", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isString() || !args[1].isString()) {
            return Value(Error("lastIndexOf requires string and substring arguments"));
        }
        
        std::string str = args[0].asString();
        std::string substr = args[1].asString();
        
        size_t pos = str.rfind(substr);
        if (pos == std::string::npos) {
            return Value(static_cast<int64_t>(-1));
        }
        
        return Value(static_cast<int64_t>(pos));
    })));
    
    stringObj.set("replace", Value(NativeFunction("replace", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 3 || !args[0].isString() || !args[1].isString() || !args[2].isString()) {
            return Value(Error("replace requires string, search, and replacement arguments"));
        }
        
        std::string str = args[0].asString();
        std::string search = args[1].asString();
        std::string replacement = args[2].asString();
        
        size_t pos = 0;
        while ((pos = str.find(search, pos)) != std::string::npos) {
            str.replace(pos, search.length(), replacement);
            pos += replacement.length();
        }
        
        return Value(str);
    })));
    
    stringObj.set("split", Value(NativeFunction("split", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isString() || !args[1].isString()) {
            return Value(Error("split requires string and delimiter arguments"));
        }
        
        std::string str = args[0].asString();
        std::string delimiter = args[1].asString();
        
        std::vector<Value> result;
        
        if (delimiter.empty()) {
            // Split into characters
            for (char c : str) {
                result.push_back(Value(std::string(1, c)));
            }
        } else {
            size_t pos = 0;
            size_t prevPos = 0;
            
            while ((pos = str.find(delimiter, prevPos)) != std::string::npos) {
                result.push_back(Value(str.substr(prevPos, pos - prevPos)));
                prevPos = pos + delimiter.length();
            }
            
            result.push_back(Value(str.substr(prevPos)));
        }
        
        return Value(Array(result));
    })));
    
    stringObj.set("toUpperCase", Value(NativeFunction("toUpperCase", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isString()) {
            return Value(Error("toUpperCase requires a string argument"));
        }
        
        std::string str = args[0].asString();
        std::transform(str.begin(), str.end(), str.begin(), ::toupper);
        
        return Value(str);
    })));
    
    stringObj.set("toLowerCase", Value(NativeFunction("toLowerCase", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isString()) {
            return Value(Error("toLowerCase requires a string argument"));
        }
        
        std::string str = args[0].asString();
        std::transform(str.begin(), str.end(), str.begin(), ::tolower);
        
        return Value(str);
    })));
    
    stringObj.set("trim", Value(NativeFunction("trim", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isString()) {
            return Value(Error("trim requires a string argument"));
        }
        
        std::string str = args[0].asString();
        
        // Trim leading whitespace
        str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
        
        // Trim trailing whitespace
        str.erase(std::find_if(str.rbegin(), str.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), str.end());
        
        return Value(str);
    })));
    
    vm.defineGlobal("String", Value(stringObj));
}

/**
 * Array module
 */
void loadArrayModule(VM& vm) {
    auto arrayObj = Object();
    
    arrayObj.set("length", Value(NativeFunction("length", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isArray()) {
            return Value(Error("length requires an array argument"));
        }
        
        return Value(static_cast<int64_t>(args[0].asArray().size()));
    })));
    
    arrayObj.set("push", Value(NativeFunction("push", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isArray()) {
            return Value(Error("push requires an array argument"));
        }
        
        Array& array = args[0].asArray();
        
        for (size_t i = 1; i < args.size(); ++i) {
            array.push(args[i]);
        }
        
        return Value(static_cast<int64_t>(array.size()));
    })));
    
    arrayObj.set("pop", Value(NativeFunction("pop", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isArray()) {
            return Value(Error("pop requires an array argument"));
        }
        
        Array& array = args[0].asArray();
        
        if (array.empty()) {
            return Value();
        }
        
        return array.pop();
    })));
    
    arrayObj.set("join", Value(NativeFunction("join", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isArray()) {
            return Value(Error("join requires an array argument"));
        }
        
        Array& array = args[0].asArray();
        std::string delimiter = args.size() > 1 ? args[1].toString() : ",";
        
        std::string result;
        
        for (size_t i = 0; i < array.size(); ++i) {
            if (i > 0) {
                result += delimiter;
            }
            result += array.get(i).toString();
        }
        
        return Value(result);
    })));
    
    arrayObj.set("indexOf", Value(NativeFunction("indexOf", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isArray()) {
            return Value(Error("indexOf requires array and value arguments"));
        }
        
        Array& array = args[0].asArray();
        Value value = args[1];
        
        for (size_t i = 0; i < array.size(); ++i) {
            if (array.get(i) == value) {
                return Value(static_cast<int64_t>(i));
            }
        }
        
        return Value(static_cast<int64_t>(-1));
    })));
    
    arrayObj.set("slice", Value(NativeFunction("slice", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 3 || !args[0].isArray() || !args[1].isNumber() || !args[2].isNumber()) {
            return Value(Error("slice requires array, start, and end arguments"));
        }
        
        Array& array = args[0].asArray();
        int64_t start = args[1].asInt();
        int64_t end = args[2].asInt();
        
        if (start < 0) start = 0;
        if (end > static_cast<int64_t>(array.size())) end = array.size();
        if (start >= end) return Value(Array());
        
        std::vector<Value> result;
        for (int64_t i = start; i < end; ++i) {
            result.push_back(array.get(i));
        }
        
        return Value(Array(result));
    })));
    
    arrayObj.set("concat", Value(NativeFunction("concat", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isArray()) {
            return Value(Error("concat requires an array argument"));
        }
        
        Array& array = args[0].asArray();
        std::vector<Value> result = array.getElements();
        
        for (size_t i = 1; i < args.size(); ++i) {
            if (args[i].isArray()) {
                const Array& other = args[i].asArray();
                result.insert(result.end(), other.getElements().begin(), other.getElements().end());
            } else {
                result.push_back(args[i]);
            }
        }
        
        return Value(Array(result));
    })));
    
    arrayObj.set("forEach", Value(NativeFunction("forEach", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isArray() || !args[1].isFunction()) {
            return Value(Error("forEach requires array and function arguments"));
        }
        
        Array& array = args[0].asArray();
        Function& func = args[1].asFunction();
        
        // TODO: Implement forEach
        
        return Value();
    })));
    
    arrayObj.set("map", Value(NativeFunction("map", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isArray() || !args[1].isFunction()) {
            return Value(Error("map requires array and function arguments"));
        }
        
        Array& array = args[0].asArray();
        Function& func = args[1].asFunction();
        
        // TODO: Implement map
        
        return Value();
    })));
    
    arrayObj.set("filter", Value(NativeFunction("filter", [](const std::vector<Value>& args) -> Value {
        if (args.size() < 2 || !args[0].isArray() || !args[1].isFunction()) {
            return Value(Error("filter requires array and function arguments"));
        }
        
        // Prevent unused variable warnings by commenting these out until implementation
        // Array& array = args[0].asArray();
        // Function& func = args[1].asFunction();
        (void)args; // Prevent unused parameter warning
        
        // TODO: Implement filter
        
        return Value();
    })));
    
    vm.defineGlobal("Array", Value(arrayObj));
}

/**
 * System module
 */
void loadSystemModule(VM& vm) {
    auto systemObj = Object();
    
    systemObj.set("time", Value(NativeFunction("time", [](const std::vector<Value>& args) -> Value {
        (void)args; // Prevent unused parameter warning
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        
        return Value(static_cast<int64_t>(seconds));
    })));
    
    systemObj.set("clock", Value(NativeFunction("clock", [](const std::vector<Value>& args) -> Value {
        (void)args; // Prevent unused parameter warning
        return Value(static_cast<double>(std::clock()) / CLOCKS_PER_SEC);
    })));
    
    systemObj.set("exit", Value(NativeFunction("exit", [](const std::vector<Value>& args) -> Value {
        int code = 0;
        
        if (!args.empty() && args[0].isNumber()) {
            code = static_cast<int>(args[0].asInt());
        }
        
        exit(code);
        
        return Value();
    })));
    
    systemObj.set("getenv", Value(NativeFunction("getenv", [](const std::vector<Value>& args) -> Value {
        if (args.empty() || !args[0].isString()) {
            return Value(Error("getenv requires a string argument"));
        }
        
        std::string name = args[0].asString();
        const char* value = std::getenv(name.c_str());
        
        if (value == nullptr) {
            return Value();
        }
        
        return Value(std::string(value));
    })));
    
    vm.defineGlobal("System", Value(systemObj));
}

/**
 * Aerospace module
 */
void loadAerospaceModule(VM& vm) {
    auto aerospaceObj = Object();
    
    // Orbital mechanics
    auto orbitalObj = Object();
    
    orbitalObj.set("calculateOrbit", Value(NativeFunction("calculateOrbit", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement orbital calculations
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    orbitalObj.set("propagateOrbit", Value(NativeFunction("propagateOrbit", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement orbit propagation
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    orbitalObj.set("calculateHohmannTransfer", Value(NativeFunction("calculateHohmannTransfer", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement Hohmann transfer calculation
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    orbitalObj.set("calculateDeltaV", Value(NativeFunction("calculateDeltaV", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement delta-V calculation
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    aerospaceObj.set("Orbital", Value(orbitalObj));
    
    // Navigation
    auto navigationObj = Object();
    
    navigationObj.set("calculateRoute", Value(NativeFunction("calculateRoute", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement route calculation
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    navigationObj.set("calculateHeading", Value(NativeFunction("calculateHeading", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement heading calculation
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    aerospaceObj.set("Navigation", Value(navigationObj));
    
    // Attitude control
    auto attitudeObj = Object();
    
    attitudeObj.set("calculateAttitude", Value(NativeFunction("calculateAttitude", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement attitude calculation
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    attitudeObj.set("calculateRotation", Value(NativeFunction("calculateRotation", [](const std::vector<Value>& args) -> Value {
        // TODO: Implement rotation calculation
        (void)args; // Prevent unused parameter warning
        return Value();
    })));
    
    aerospaceObj.set("Attitude", Value(attitudeObj));
    
    vm.defineGlobal("Aerospace", Value(aerospaceObj));
}

#include "math_module.h"
#include "vector_module.h"
#include "matrix_module.h"
#include "quaternion_module.h"
#include "orbital_module.h"
#include "navigation_module.h"
#include "control_module.h"
#include "sensors_module.h"
#include "signal_module.h"
#include "time_module.h"

/**
 * Load all standard library modules
 */
void loadStandardLibrary(VM& vm) {
    // Core modules
    // Temporarily comment out namespace-based initializations until namespaces are properly defined
    // math::initialize(vm);
    // vector::initialize(vm);
    // matrix::initialize(vm);
    // quaternion::initialize(vm);
    loadIOModule(vm);
    loadStringModule(vm);
    loadArrayModule(vm);
    loadSystemModule(vm);
    
    // Aerospace modules
    // Temporarily comment out until runtime::Runtime is properly defined
    // registerNetworkModule(vm);
    // orbital::initialize(vm);
    // navigation::initialize(vm);
    // control::initialize(vm);
    // sensors::initialize(vm);
    // signal::initialize(vm);
    // time::initialize(vm);
    loadAerospaceModule(vm);
}

} // namespace stdlib
} // namespace astra

#endif // ASTRA_STDLIB_H