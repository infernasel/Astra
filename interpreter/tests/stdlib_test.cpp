#include <gtest/gtest.h>
#include "../src/vm/vm.h"
#include "../src/stdlib/stdlib.h"

using namespace astra;

class StdlibTest : public ::testing::Test {
protected:
    VM vm;
    
    void SetUp() override {
        stdlib::loadStandardLibrary(vm);
    }
};

TEST_F(StdlibTest, MathModule) {
    // Test Math constants
    EXPECT_DOUBLE_EQ(M_PI, vm.eval("Math.PI").asFloat());
    EXPECT_DOUBLE_EQ(M_E, vm.eval("Math.E").asFloat());
    
    // Test Math functions
    EXPECT_EQ(5, vm.eval("Math.abs(-5)").asInt());
    EXPECT_DOUBLE_EQ(0.0, vm.eval("Math.sin(0)").asFloat());
    EXPECT_DOUBLE_EQ(1.0, vm.eval("Math.cos(0)").asFloat());
    EXPECT_DOUBLE_EQ(4.0, vm.eval("Math.sqrt(16)").asFloat());
    EXPECT_DOUBLE_EQ(8.0, vm.eval("Math.pow(2, 3)").asFloat());
    EXPECT_EQ(3, vm.eval("Math.floor(3.7)").asInt());
    EXPECT_EQ(4, vm.eval("Math.ceil(3.2)").asInt());
    EXPECT_EQ(4, vm.eval("Math.round(3.7)").asInt());
    EXPECT_EQ(3, vm.eval("Math.min(3, 5, 7)").asInt());
    EXPECT_EQ(7, vm.eval("Math.max(3, 5, 7)").asInt());
    
    // Test random functions
    EXPECT_GE(vm.eval("Math.random()").asFloat(), 0.0);
    EXPECT_LT(vm.eval("Math.random()").asFloat(), 1.0);
    
    int randomInt = vm.eval("Math.randomInt(1, 10)").asInt();
    EXPECT_GE(randomInt, 1);
    EXPECT_LE(randomInt, 10);
}

TEST_F(StdlibTest, VectorModule) {
    // Test Vector2
    vm.eval("var v2 = Vector2(3, 4)");
    EXPECT_DOUBLE_EQ(3.0, vm.eval("v2.x").asFloat());
    EXPECT_DOUBLE_EQ(4.0, vm.eval("v2.y").asFloat());
    EXPECT_DOUBLE_EQ(5.0, vm.eval("v2.magnitude()").asFloat());
    
    // Test Vector3
    vm.eval("var v3 = Vector3(1, 2, 3)");
    EXPECT_DOUBLE_EQ(1.0, vm.eval("v3.x").asFloat());
    EXPECT_DOUBLE_EQ(2.0, vm.eval("v3.y").asFloat());
    EXPECT_DOUBLE_EQ(3.0, vm.eval("v3.z").asFloat());
    EXPECT_DOUBLE_EQ(std::sqrt(14.0), vm.eval("v3.magnitude()").asFloat());
    
    // Test vector operations
    vm.eval("var v3a = Vector3(1, 0, 0)");
    vm.eval("var v3b = Vector3(0, 1, 0)");
    vm.eval("var v3c = v3a.cross(v3b)");
    EXPECT_DOUBLE_EQ(0.0, vm.eval("v3c.x").asFloat());
    EXPECT_DOUBLE_EQ(0.0, vm.eval("v3c.y").asFloat());
    EXPECT_DOUBLE_EQ(1.0, vm.eval("v3c.z").asFloat());
}

TEST_F(StdlibTest, IOModule) {
    // Test IO functions
    // Note: These tests are limited since they involve console I/O
    
    // Redirect stdout to capture output
    std::stringstream buffer;
    std::streambuf* oldCout = std::cout.rdbuf(buffer.rdbuf());
    
    vm.eval("IO.print('Hello, world!')");
    
    // Restore stdout
    std::cout.rdbuf(oldCout);
    
    EXPECT_EQ("Hello, world!\n", buffer.str());
}

TEST_F(StdlibTest, StringModule) {
    // Test String functions
    EXPECT_EQ(5, vm.eval("String.length('hello')").asInt());
    EXPECT_EQ("el", vm.eval("String.substring('hello', 1, 3)").asString());
    EXPECT_EQ(2, vm.eval("String.indexOf('hello', 'l')").asInt());
    EXPECT_EQ(3, vm.eval("String.lastIndexOf('hello', 'l')").asInt());
    EXPECT_EQ("hXllo", vm.eval("String.replace('hello', 'e', 'X')").asString());
    EXPECT_EQ("HELLO", vm.eval("String.toUpperCase('hello')").asString());
    EXPECT_EQ("hello", vm.eval("String.toLowerCase('HELLO')").asString());
    EXPECT_EQ("hello", vm.eval("String.trim('  hello  ')").asString());
}

TEST_F(StdlibTest, ArrayModule) {
    // Test Array functions
    vm.eval("var arr = [1, 2, 3, 4, 5]");
    EXPECT_EQ(5, vm.eval("Array.length(arr)").asInt());
    
    vm.eval("Array.push(arr, 6)");
    EXPECT_EQ(6, vm.eval("Array.length(arr)").asInt());
    EXPECT_EQ(6, vm.eval("arr[5]").asInt());
    
    EXPECT_EQ(6, vm.eval("Array.pop(arr)").asInt());
    EXPECT_EQ(5, vm.eval("Array.length(arr)").asInt());
    
    EXPECT_EQ("1,2,3,4,5", vm.eval("Array.join(arr)").asString());
    EXPECT_EQ("1-2-3-4-5", vm.eval("Array.join(arr, '-')").asString());
    
    EXPECT_EQ(2, vm.eval("Array.indexOf(arr, 3)").asInt());
    EXPECT_EQ(-1, vm.eval("Array.indexOf(arr, 10)").asInt());
    
    vm.eval("var sliced = Array.slice(arr, 1, 4)");
    EXPECT_EQ(3, vm.eval("Array.length(sliced)").asInt());
    EXPECT_EQ(2, vm.eval("sliced[0]").asInt());
    EXPECT_EQ(3, vm.eval("sliced[1]").asInt());
    EXPECT_EQ(4, vm.eval("sliced[2]").asInt());
    
    vm.eval("var concat = Array.concat(arr, [6, 7, 8])");
    EXPECT_EQ(8, vm.eval("Array.length(concat)").asInt());
    EXPECT_EQ(6, vm.eval("concat[5]").asInt());
    EXPECT_EQ(7, vm.eval("concat[6]").asInt());
    EXPECT_EQ(8, vm.eval("concat[7]").asInt());
}

TEST_F(StdlibTest, SystemModule) {
    // Test System functions
    EXPECT_GT(vm.eval("System.time()").asInt(), 0);
    EXPECT_GE(vm.eval("System.clock()").asFloat(), 0.0);
    
    // Test getenv (result depends on environment)
    // We can't make specific assertions without knowing the environment
}

TEST_F(StdlibTest, AerospaceModule) {
    // Test Aerospace module
    // These are placeholder tests since the actual implementations are not complete
    
    EXPECT_TRUE(vm.hasGlobal("Aerospace"));
    EXPECT_TRUE(vm.eval("Aerospace").isObject());
    
    EXPECT_TRUE(vm.eval("Aerospace.Orbital").isObject());
    EXPECT_TRUE(vm.eval("Aerospace.Navigation").isObject());
    EXPECT_TRUE(vm.eval("Aerospace.Attitude").isObject());
    
    // Test that functions exist
    EXPECT_TRUE(vm.eval("Aerospace.Orbital.calculateOrbit").isFunction() || 
                vm.eval("Aerospace.Orbital.calculateOrbit").isNativeFunction());
    
    EXPECT_TRUE(vm.eval("Aerospace.Navigation.calculateRoute").isFunction() || 
                vm.eval("Aerospace.Navigation.calculateRoute").isNativeFunction());
    
    EXPECT_TRUE(vm.eval("Aerospace.Attitude.calculateAttitude").isFunction() || 
                vm.eval("Aerospace.Attitude.calculateAttitude").isNativeFunction());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}