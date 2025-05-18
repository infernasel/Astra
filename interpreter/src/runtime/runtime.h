/**
 * ASTRA Programming Language Interpreter
 * Runtime system
 */

#ifndef ASTRA_RUNTIME_H
#define ASTRA_RUNTIME_H

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

#include "../vm/vm.h"

namespace astra {
namespace runtime {

/**
 * Task scheduler
 */
class TaskScheduler {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable condition;
    bool stop;
    
public:
    TaskScheduler(size_t numThreads = std::thread::hardware_concurrency())
        : stop(false) {
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    
                    {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        
                        if (stop && tasks.empty()) {
                            return;
                        }
                        
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    
                    task();
                }
            });
        }
    }
    
    ~TaskScheduler() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        
        condition.notify_all();
        
        for (auto& worker : workers) {
            worker.join();
        }
    }
    
    template<class F>
    void enqueue(F&& f) {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            tasks.emplace(std::forward<F>(f));
        }
        
        condition.notify_one();
    }
};

/**
 * Memory manager
 */
class MemoryManager {
private:
    size_t totalAllocated;
    size_t maxAllocated;
    std::mutex mutex;
    
public:
    MemoryManager() : totalAllocated(0), maxAllocated(0) {}
    
    void* allocate(size_t size) {
        std::lock_guard<std::mutex> lock(mutex);
        void* ptr = ::operator new(size);
        
        totalAllocated += size;
        if (totalAllocated > maxAllocated) {
            maxAllocated = totalAllocated;
        }
        
        return ptr;
    }
    
    void deallocate(void* ptr, size_t size) {
        std::lock_guard<std::mutex> lock(mutex);
        ::operator delete(ptr);
        
        totalAllocated -= size;
    }
    
    size_t getTotalAllocated() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex));
        return totalAllocated;
    }
    
    size_t getMaxAllocated() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex));
        return maxAllocated;
    }
};

/**
 * Timer class
 */
class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> endTime;
    bool running;
    
public:
    Timer() : running(false) {}
    
    void start() {
        startTime = std::chrono::high_resolution_clock::now();
        running = true;
    }
    
    void stop() {
        endTime = std::chrono::high_resolution_clock::now();
        running = false;
    }
    
    double elapsedMilliseconds() const {
        auto end = running ? std::chrono::high_resolution_clock::now() : endTime;
        return std::chrono::duration<double, std::milli>(end - startTime).count();
    }
    
    double elapsedSeconds() const {
        return elapsedMilliseconds() / 1000.0;
    }
};

/**
 * Event system
 */
class EventSystem {
private:
    using EventHandler = std::function<void(const Value&)>;
    std::unordered_map<std::string, std::vector<EventHandler>> handlers;
    std::mutex mutex;
    
public:
    void addEventListener(const std::string& event, EventHandler handler) {
        std::lock_guard<std::mutex> lock(mutex);
        handlers[event].push_back(handler);
    }
    
    void removeEventListener(const std::string& event) {
        std::lock_guard<std::mutex> lock(mutex);
        handlers.erase(event);
    }
    
    void dispatchEvent(const std::string& event, const Value& data) {
        std::lock_guard<std::mutex> lock(mutex);
        auto it = handlers.find(event);
        if (it != handlers.end()) {
            for (const auto& handler : it->second) {
                handler(data);
            }
        }
    }
};

/**
 * Runtime system
 */
class Runtime {
private:
    VM vm;
    TaskScheduler scheduler;
    MemoryManager memoryManager;
    EventSystem eventSystem;
    Timer timer;
    
public:
    Runtime() {
        // Initialize runtime
        timer.start();
    }
    
    VM& getVM() { return vm; }
    TaskScheduler& getScheduler() { return scheduler; }
    MemoryManager& getMemoryManager() { return memoryManager; }
    EventSystem& getEventSystem() { return eventSystem; }
    Timer& getTimer() { return timer; }
    
    void initialize() {
        // Register runtime-specific functions in VM
        vm.defineGlobal("setTimeout", Value(NativeFunction("setTimeout", [this](const std::vector<Value>& args) -> Value {
            if (args.size() < 2 || !args[0].isFunction() || !args[1].isNumber()) {
                return Value(Error("setTimeout requires a function and a delay"));
            }
            
            auto func = args[0].asFunction();
            auto delay = args[1].asInt();
            
            scheduler.enqueue([this, func = func, delay]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                vm.eval(func.getBody(), false, false);
            });
            
            return Value();
        })));
        
        vm.defineGlobal("setInterval", Value(NativeFunction("setInterval", [this](const std::vector<Value>& args) -> Value {
            if (args.size() < 2 || !args[0].isFunction() || !args[1].isNumber()) {
                return Value(Error("setInterval requires a function and a delay"));
            }
            
            auto func = args[0].asFunction();
            auto delay = args[1].asInt();
            bool running = true;
            
            scheduler.enqueue([this, func = func, delay, running]() mutable {
                while (running) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                    vm.eval(func.getBody(), false, false);
                }
            });
            
            return Value();
        })));
        
        vm.defineGlobal("clearTimeout", Value(NativeFunction("clearTimeout", [](const std::vector<Value>& args) -> Value {
            // Not implemented yet
            (void)args; // Prevent unused parameter warning
            return Value();
        })));
        
        vm.defineGlobal("clearInterval", Value(NativeFunction("clearInterval", [](const std::vector<Value>& args) -> Value {
            // Not implemented yet
            (void)args; // Prevent unused parameter warning
            return Value();
        })));
        
        vm.defineGlobal("addEventListener", Value(NativeFunction("addEventListener", [this](const std::vector<Value>& args) -> Value {
            if (args.size() < 2 || !args[0].isString() || !args[1].isFunction()) {
                return Value(Error("addEventListener requires an event name and a function"));
            }
            
            auto event = args[0].asString();
            auto func = args[1].asFunction();
            
            eventSystem.addEventListener(event, [this, func = func](const Value& data) {
                std::vector<Value> args = {data};
                (void)args; // Prevent unused variable warning
                vm.eval(func.getBody());
            });
            
            return Value();
        })));
        
        vm.defineGlobal("removeEventListener", Value(NativeFunction("removeEventListener", [this](const std::vector<Value>& args) -> Value {
            if (args.size() < 1 || !args[0].isString()) {
                return Value(Error("removeEventListener requires an event name"));
            }
            
            auto event = args[0].asString();
            eventSystem.removeEventListener(event);
            
            return Value();
        })));
        
        vm.defineGlobal("dispatchEvent", Value(NativeFunction("dispatchEvent", [this](const std::vector<Value>& args) -> Value {
            if (args.size() < 2 || !args[0].isString()) {
                return Value(Error("dispatchEvent requires an event name and data"));
            }
            
            auto event = args[0].asString();
            auto data = args[1];
            
            eventSystem.dispatchEvent(event, data);
            
            return Value();
        })));
        
        vm.defineGlobal("now", Value(NativeFunction("now", [this](const std::vector<Value>& args) -> Value {
            (void)args; // Prevent unused parameter warning
            return Value(static_cast<int64_t>(timer.elapsedMilliseconds()));
        })));
        
        vm.defineGlobal("sleep", Value(NativeFunction("sleep", [](const std::vector<Value>& args) -> Value {
            if (args.size() < 1 || !args[0].isNumber()) {
                return Value(Error("sleep requires a delay in milliseconds"));
            }
            
            auto delay = args[0].asInt();
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            
            return Value();
        })));
    }
};

} // namespace runtime
} // namespace astra

#endif // ASTRA_RUNTIME_H