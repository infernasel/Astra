#include <gtest/gtest.h>
#include "../src/runtime/runtime.h"

using namespace astra;
using namespace astra::runtime;

class RuntimeTest : public ::testing::Test {
protected:
    Runtime runtime;
};

TEST_F(RuntimeTest, TaskScheduler) {
    // Test task scheduler
    bool taskExecuted = false;
    
    runtime.getScheduler().enqueue([&taskExecuted]() {
        taskExecuted = true;
    });
    
    // Wait for task to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    EXPECT_TRUE(taskExecuted);
}

TEST_F(RuntimeTest, MemoryManager) {
    // Test memory manager
    void* ptr = runtime.getMemoryManager().allocate(1024);
    EXPECT_NE(ptr, nullptr);
    
    EXPECT_GE(runtime.getMemoryManager().getTotalAllocated(), 1024);
    
    runtime.getMemoryManager().deallocate(ptr, 1024);
    
    EXPECT_EQ(runtime.getMemoryManager().getTotalAllocated(), 0);
}

TEST_F(RuntimeTest, Timer) {
    // Test timer
    runtime.getTimer().start();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    runtime.getTimer().stop();
    
    EXPECT_GE(runtime.getTimer().elapsedMilliseconds(), 100);
}

TEST_F(RuntimeTest, EventSystem) {
    // Test event system
    bool eventHandled = false;
    
    runtime.getEventSystem().addEventListener("test", [&eventHandled](const Value& data) {
        eventHandled = true;
        EXPECT_EQ(data.asInt(), 42);
    });
    
    runtime.getEventSystem().dispatchEvent("test", Value(42));
    
    EXPECT_TRUE(eventHandled);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}