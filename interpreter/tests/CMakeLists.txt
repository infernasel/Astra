cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Test source files
set(TEST_SOURCES
    vm_test.cpp
    runtime_test.cpp
    stdlib_test.cpp
)

# Create test executable
add_executable(astra_interpreter_tests ${TEST_SOURCES})
target_link_libraries(astra_interpreter_tests ${GTEST_LIBRARIES} pthread)

# Add tests
add_test(NAME astra_interpreter_tests COMMAND astra_interpreter_tests)