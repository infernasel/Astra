/**
 * ASTRA Programming Language Interpreter
 * Virtual Machine
 */

#ifndef ASTRA_VM_H
#define ASTRA_VM_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <stdexcept>
#include <chrono>
#include "value.h"

namespace astra {

/**
 * Virtual Machine class forward declaration
 */
class VM;




























/**
 * Virtual Machine class
 */
class VM {
private:
    std::shared_ptr<Environment> globalEnv;
    std::vector<std::shared_ptr<Task>> tasks;
    std::unordered_map<std::string, std::chrono::microseconds> profileData;
    
    // Internal methods
    Value evaluateExpression(const std::string& expr, std::shared_ptr<Environment> env);
    void executeStatement(const std::string& stmt, std::shared_ptr<Environment> env);
    
public:
    VM() : globalEnv(std::make_shared<Environment>()) {}
    
    // Public API
    Value eval(const std::string& code, bool debug = false, bool trace = false);
    Value execute(const std::string& code, bool debug = false, bool trace = false);
    bool check(const std::string& code);
    
    // Environment management
    void defineGlobal(const std::string& name, const Value& value);
    Value getGlobal(const std::string& name) const;
    bool hasGlobal(const std::string& name) const;
    
    // Task management
    void addTask(std::shared_ptr<Task> task);
    void removeTask(const std::string& name);
    void runTasks();
    
    // Profiling
    void addProfileData(const std::string& name, std::chrono::microseconds duration);
    void printProfileInfo() const;
    
    // Reset VM state
    void reset();
};

} // namespace astra

#endif // ASTRA_VM_H