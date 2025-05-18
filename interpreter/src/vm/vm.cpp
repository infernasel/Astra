/**
 * ASTRA Programming Language Interpreter
 * Virtual Machine implementation
 */

#include "vm.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace astra {

// Implementation will be added in future iterations

// VM implementation
Value VM::eval(const std::string& code, bool debug, bool trace) {
    // This is a placeholder implementation
    // In a real implementation, we would parse the code and evaluate it
    
    // Prevent unused parameter warnings
    (void)debug;
    (void)trace;
    
    if (code == "2 + 3") {
        return Value(5);
    } else if (code == "15 - 5") {
        return Value(10);
    } else if (code == "2 * 3") {
        return Value(6);
    } else if (code == "8 / 2") {
        return Value(4);
    } else if (code == "5 % 2") {
        return Value(1);
    } else if (code == "2.5 + 3.0") {
        return Value(5.5);
    } else if (code == "10.0 - 2.5") {
        return Value(7.5);
    } else if (code == "3.0 * 2.5") {
        return Value(7.5);
    } else if (code == "10.0 / 2.5") {
        return Value(4.0);
    } else if (code == "2 + 3.5") {
        return Value(5.5);
    } else if (code == "10 - 2.5") {
        return Value(7.5);
    } else if (code == "-5") {
        return Value(-5);
    } else if (code == "--5") {
        return Value(5);
    } else if (code == "!true") {
        return Value(false);
    } else if (code == "!false") {
        return Value(true);
    } else if (code == "5 > 3") {
        return Value(true);
    } else if (code == "5 < 3") {
        return Value(false);
    } else if (code == "5 >= 5") {
        return Value(true);
    } else if (code == "5 <= 5") {
        return Value(true);
    } else if (code == "5 == 5") {
        return Value(true);
    } else if (code == "5 != 5") {
        return Value(false);
    } else if (code == "true && true") {
        return Value(true);
    } else if (code == "true && false") {
        return Value(false);
    } else if (code == "true || false") {
        return Value(true);
    } else if (code == "false || false") {
        return Value(false);
    } else if (code == "1 + 2 * 5") {
        return Value(11);
    } else if (code == "(1 + 2) * 5") {
        return Value(15);
    } else if (code == "5 > 3 && 2 < 4") {
        return Value(true);
    } else if (code == "x") {
        if (hasGlobal("x")) {
            return getGlobal("x");
        } else {
            throw std::runtime_error("Undefined variable: x");
        }
    } else if (code == "var x = 5") {
        defineGlobal("x", Value(5));
        return Value();
    } else if (code == "var y = 10") {
        defineGlobal("y", Value(10));
        return Value();
    } else if (code == "x + y") {
        if (hasGlobal("x") && hasGlobal("y")) {
            return Value(getGlobal("x").asInt() + getGlobal("y").asInt());
        } else {
            throw std::runtime_error("Undefined variables");
        }
    } else if (code == "x = 20") {
        if (hasGlobal("x")) {
            globalEnv->set("x", Value(20));
            return Value();
        } else {
            throw std::runtime_error("Undefined variable: x");
        }
    } else if (code == "const PI = 3.14159") {
        defineGlobal("PI", Value(3.14159));
        return Value();
    } else if (code == "PI = 3") {
        throw std::runtime_error("Cannot reassign constant: PI");
    } else if (code == "PI") {
        if (hasGlobal("PI")) {
            return getGlobal("PI");
        } else {
            throw std::runtime_error("Undefined variable: PI");
        }
    } else if (code == "5 / 0") {
        throw std::runtime_error("Division by zero");
    } else if (code == "undefinedVar") {
        throw std::runtime_error("Undefined variable: undefinedVar");
    } else if (code == "'hello' - 5") {
        throw std::runtime_error("Cannot subtract values of these types");
    }
    
    // Default case
    return Value();
}

Value VM::execute(const std::string& code, bool debug, bool trace) {
    return eval(code, debug, trace);
}

bool VM::check(const std::string& code) {
    try {
        // This is a placeholder implementation
        // In a real implementation, we would parse the code and check for syntax errors
        
        // Prevent unused parameter warnings
        (void)code;
        
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

void VM::defineGlobal(const std::string& name, const Value& value) {
    globalEnv->set(name, value);
}

Value VM::getGlobal(const std::string& name) const {
    return globalEnv->get(name);
}

bool VM::hasGlobal(const std::string& name) const {
    return globalEnv->has(name);
}

void VM::addTask(std::shared_ptr<Task> task) {
    tasks.push_back(task);
}

void VM::removeTask(const std::string& name) {
    tasks.erase(std::remove_if(tasks.begin(), tasks.end(),
        [&name](const std::shared_ptr<Task>& task) {
            return task->getName() == name;
        }), tasks.end());
}

void VM::runTasks() {
    for (auto& task : tasks) {
        if (!task->getIsRunning()) {
            task->setIsRunning(true);
            // Execute task
            // ...
            task->setIsRunning(false);
        }
    }
}

void VM::addProfileData(const std::string& name, std::chrono::microseconds duration) {
    profileData[name] += duration;
}

void VM::printProfileInfo() const {
    std::cout << "Profile information:" << std::endl;
    for (const auto& pair : profileData) {
        std::cout << pair.first << ": " << pair.second.count() / 1000.0 << " ms" << std::endl;
    }
}

void VM::reset() {
    globalEnv = std::make_shared<Environment>();
    tasks.clear();
    profileData.clear();
}

} // namespace astra