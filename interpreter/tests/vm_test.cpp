#include <gtest/gtest.h>
#include "../src/vm/vm.h"

using namespace astra;

class VMTest : public ::testing::Test {
protected:
    VM vm;
};

TEST_F(VMTest, EvaluateSimpleExpressions) {
    // Integer arithmetic
    EXPECT_EQ(5, vm.eval("2 + 3").asInt());
    EXPECT_EQ(10, vm.eval("15 - 5").asInt());
    EXPECT_EQ(6, vm.eval("2 * 3").asInt());
    EXPECT_EQ(4, vm.eval("8 / 2").asInt());
    EXPECT_EQ(1, vm.eval("5 % 2").asInt());
    
    // Float arithmetic
    EXPECT_DOUBLE_EQ(5.5, vm.eval("2.5 + 3.0").asFloat());
    EXPECT_DOUBLE_EQ(7.5, vm.eval("10.0 - 2.5").asFloat());
    EXPECT_DOUBLE_EQ(7.5, vm.eval("3.0 * 2.5").asFloat());
    EXPECT_DOUBLE_EQ(4.0, vm.eval("10.0 / 2.5").asFloat());
    
    // Mixed arithmetic
    EXPECT_DOUBLE_EQ(5.5, vm.eval("2 + 3.5").asFloat());
    EXPECT_DOUBLE_EQ(7.5, vm.eval("10 - 2.5").asFloat());
    
    // Unary operators
    EXPECT_EQ(-5, vm.eval("-5").asInt());
    EXPECT_EQ(5, vm.eval("--5").asInt());
    EXPECT_EQ(false, vm.eval("!true").asBool());
    EXPECT_EQ(true, vm.eval("!false").asBool());
    
    // Comparison operators
    EXPECT_EQ(true, vm.eval("5 > 3").asBool());
    EXPECT_EQ(false, vm.eval("5 < 3").asBool());
    EXPECT_EQ(true, vm.eval("5 >= 5").asBool());
    EXPECT_EQ(true, vm.eval("5 <= 5").asBool());
    EXPECT_EQ(true, vm.eval("5 == 5").asBool());
    EXPECT_EQ(false, vm.eval("5 != 5").asBool());
    
    // Logical operators
    EXPECT_EQ(true, vm.eval("true && true").asBool());
    EXPECT_EQ(false, vm.eval("true && false").asBool());
    EXPECT_EQ(true, vm.eval("true || false").asBool());
    EXPECT_EQ(false, vm.eval("false || false").asBool());
    
    // Precedence
    EXPECT_EQ(11, vm.eval("1 + 2 * 5").asInt());
    EXPECT_EQ(15, vm.eval("(1 + 2) * 5").asInt());
    EXPECT_EQ(true, vm.eval("5 > 3 && 2 < 4").asBool());
}

TEST_F(VMTest, VariableDeclarationAndUsage) {
    vm.eval("var x = 5");
    EXPECT_EQ(5, vm.eval("x").asInt());
    
    vm.eval("var y = 10");
    EXPECT_EQ(15, vm.eval("x + y").asInt());
    
    vm.eval("x = 20");
    EXPECT_EQ(20, vm.eval("x").asInt());
    
    vm.eval("const PI = 3.14159");
    EXPECT_DOUBLE_EQ(3.14159, vm.eval("PI").asFloat());
    
    // Constant cannot be reassigned
    EXPECT_THROW(vm.eval("PI = 3"), std::runtime_error);
}

TEST_F(VMTest, FunctionDeclarationAndCall) {
    vm.eval("func add(a, b) { return a + b; }");
    EXPECT_EQ(8, vm.eval("add(3, 5)").asInt());
    
    vm.eval("func factorial(n) { if (n <= 1) return 1; else return n * factorial(n - 1); }");
    EXPECT_EQ(120, vm.eval("factorial(5)").asInt());
    
    vm.eval("func fibonacci(n) { if (n <= 1) return n; else return fibonacci(n - 1) + fibonacci(n - 2); }");
    EXPECT_EQ(55, vm.eval("fibonacci(10)").asInt());
}

TEST_F(VMTest, ControlFlow) {
    // If statement
    vm.eval("var max = 0; if (5 > 3) max = 5; else max = 3;");
    EXPECT_EQ(5, vm.eval("max").asInt());
    
    vm.eval("var min = 0; if (5 < 3) min = 5; else min = 3;");
    EXPECT_EQ(3, vm.eval("min").asInt());
    
    // While loop
    vm.eval("var i = 0; var sum = 0; while (i < 5) { sum += i; i += 1; }");
    EXPECT_EQ(10, vm.eval("sum").asInt());
    
    // For loop
    vm.eval("var sum = 0; for (var i = 0; i < 5; i += 1) { sum += i; }");
    EXPECT_EQ(10, vm.eval("sum").asInt());
    
    // Break statement
    vm.eval("var i = 0; var sum = 0; while (i < 10) { if (i >= 5) break; sum += i; i += 1; }");
    EXPECT_EQ(10, vm.eval("sum").asInt());
    
    // Continue statement
    vm.eval("var i = 0; var sum = 0; while (i < 10) { i += 1; if (i % 2 == 0) continue; sum += i; }");
    EXPECT_EQ(25, vm.eval("sum").asInt());
}

TEST_F(VMTest, DataStructures) {
    // Arrays
    vm.eval("var arr = [1, 2, 3, 4, 5]");
    EXPECT_EQ(3, vm.eval("arr[2]").asInt());
    EXPECT_EQ(5, vm.eval("arr.length").asInt());
    
    vm.eval("arr[2] = 10");
    EXPECT_EQ(10, vm.eval("arr[2]").asInt());
    
    vm.eval("arr.push(6)");
    EXPECT_EQ(6, vm.eval("arr.length").asInt());
    EXPECT_EQ(6, vm.eval("arr[5]").asInt());
    
    // Objects
    vm.eval("var obj = {x: 10, y: 20}");
    EXPECT_EQ(10, vm.eval("obj.x").asInt());
    EXPECT_EQ(20, vm.eval("obj.y").asInt());
    
    vm.eval("obj.z = 30");
    EXPECT_EQ(30, vm.eval("obj.z").asInt());
    
    vm.eval("obj['w'] = 40");
    EXPECT_EQ(40, vm.eval("obj.w").asInt());
}

TEST_F(VMTest, ErrorHandling) {
    // Division by zero
    EXPECT_THROW(vm.eval("5 / 0"), std::runtime_error);
    
    // Undefined variable
    EXPECT_THROW(vm.eval("undefinedVar"), std::runtime_error);
    
    // Type error
    EXPECT_THROW(vm.eval("'hello' - 5"), std::runtime_error);
    
    // Try-catch
    vm.eval("var result = 0; try { result = 5 / 0; } catch (e) { result = -1; }");
    EXPECT_EQ(-1, vm.eval("result").asInt());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}