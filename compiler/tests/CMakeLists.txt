cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Test source files
set(TEST_SOURCES
    lexer_test.cpp
    parser_test.cpp
    semantic_test.cpp
    ir_test.cpp
    optimizer_test.cpp
    codegen_test.cpp
    error_handler_test.cpp
    ast_test.cpp
    destructors_test.cpp
)

# Additional source files needed for tests
set(ADDITIONAL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/ast/ast.cpp
)

# Create test executable
add_executable(astra_tests ${TEST_SOURCES} ${ADDITIONAL_SOURCES})
target_link_libraries(astra_tests ${GTEST_LIBRARIES} pthread)

# Add tests
add_test(NAME astra_tests COMMAND astra_tests)