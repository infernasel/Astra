cmake_minimum_required(VERSION 3.10)
project(astra-compiler VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -fsanitize=address,undefined")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /WX")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
endif()

# Include directories
include_directories(src)

# Source files
set(SOURCES
    src/main_minimal.cpp
    src/ast/ast_minimal.cpp
    src/lexer/lexer_minimal.cpp
    src/parser/parser_minimal.cpp
    src/semantic/analyzer_minimal.cpp
    src/ir/generator_minimal.cpp
    src/optimizer/optimizer_minimal.cpp
    src/codegen/llvm_generator_minimal.cpp
    src/utils/error_handler_minimal.cpp
    src/utils/options_minimal.cpp
    src/vm/vm.cpp
)

# Header files
set(HEADERS
    src/lexer/lexer.h
    src/lexer/token.h
    src/parser/parser.h
    src/ast/ast.h
    src/semantic/analyzer.h
    src/ir/generator.h
    src/optimizer/optimizer.h
    src/codegen/generator.h
    src/utils/error_handler.h
    src/utils/options.h
)

# Create executable
add_executable(astrac ${SOURCES} ${HEADERS})

# Install
install(TARGETS astrac DESTINATION bin)

# Testing (disabled for now)
# if(NOT CMAKE_SYSTEM_NAME STREQUAL "Windows")
#     enable_testing()
#     add_subdirectory(tests)
# endif()

# Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()