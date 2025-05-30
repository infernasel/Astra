# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /workspace/github/compiler

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /workspace/github/build/compiler

# Include any dependencies generated for this target.
include tests/CMakeFiles/astra_tests.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tests/CMakeFiles/astra_tests.dir/compiler_depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/astra_tests.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/astra_tests.dir/flags.make

tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.o: /workspace/github/compiler/tests/lexer_test.cpp
tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.o -MF CMakeFiles/astra_tests.dir/lexer_test.cpp.o.d -o CMakeFiles/astra_tests.dir/lexer_test.cpp.o -c /workspace/github/compiler/tests/lexer_test.cpp

tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/lexer_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/lexer_test.cpp > CMakeFiles/astra_tests.dir/lexer_test.cpp.i

tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/lexer_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/lexer_test.cpp -o CMakeFiles/astra_tests.dir/lexer_test.cpp.s

tests/CMakeFiles/astra_tests.dir/parser_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/parser_test.cpp.o: /workspace/github/compiler/tests/parser_test.cpp
tests/CMakeFiles/astra_tests.dir/parser_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tests/CMakeFiles/astra_tests.dir/parser_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/parser_test.cpp.o -MF CMakeFiles/astra_tests.dir/parser_test.cpp.o.d -o CMakeFiles/astra_tests.dir/parser_test.cpp.o -c /workspace/github/compiler/tests/parser_test.cpp

tests/CMakeFiles/astra_tests.dir/parser_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/parser_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/parser_test.cpp > CMakeFiles/astra_tests.dir/parser_test.cpp.i

tests/CMakeFiles/astra_tests.dir/parser_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/parser_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/parser_test.cpp -o CMakeFiles/astra_tests.dir/parser_test.cpp.s

tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.o: /workspace/github/compiler/tests/semantic_test.cpp
tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.o -MF CMakeFiles/astra_tests.dir/semantic_test.cpp.o.d -o CMakeFiles/astra_tests.dir/semantic_test.cpp.o -c /workspace/github/compiler/tests/semantic_test.cpp

tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/semantic_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/semantic_test.cpp > CMakeFiles/astra_tests.dir/semantic_test.cpp.i

tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/semantic_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/semantic_test.cpp -o CMakeFiles/astra_tests.dir/semantic_test.cpp.s

tests/CMakeFiles/astra_tests.dir/ir_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/ir_test.cpp.o: /workspace/github/compiler/tests/ir_test.cpp
tests/CMakeFiles/astra_tests.dir/ir_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object tests/CMakeFiles/astra_tests.dir/ir_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/ir_test.cpp.o -MF CMakeFiles/astra_tests.dir/ir_test.cpp.o.d -o CMakeFiles/astra_tests.dir/ir_test.cpp.o -c /workspace/github/compiler/tests/ir_test.cpp

tests/CMakeFiles/astra_tests.dir/ir_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/ir_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/ir_test.cpp > CMakeFiles/astra_tests.dir/ir_test.cpp.i

tests/CMakeFiles/astra_tests.dir/ir_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/ir_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/ir_test.cpp -o CMakeFiles/astra_tests.dir/ir_test.cpp.s

tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.o: /workspace/github/compiler/tests/optimizer_test.cpp
tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.o -MF CMakeFiles/astra_tests.dir/optimizer_test.cpp.o.d -o CMakeFiles/astra_tests.dir/optimizer_test.cpp.o -c /workspace/github/compiler/tests/optimizer_test.cpp

tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/optimizer_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/optimizer_test.cpp > CMakeFiles/astra_tests.dir/optimizer_test.cpp.i

tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/optimizer_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/optimizer_test.cpp -o CMakeFiles/astra_tests.dir/optimizer_test.cpp.s

tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.o: /workspace/github/compiler/tests/codegen_test.cpp
tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.o -MF CMakeFiles/astra_tests.dir/codegen_test.cpp.o.d -o CMakeFiles/astra_tests.dir/codegen_test.cpp.o -c /workspace/github/compiler/tests/codegen_test.cpp

tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/codegen_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/codegen_test.cpp > CMakeFiles/astra_tests.dir/codegen_test.cpp.i

tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/codegen_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/codegen_test.cpp -o CMakeFiles/astra_tests.dir/codegen_test.cpp.s

tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.o: /workspace/github/compiler/tests/error_handler_test.cpp
tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.o -MF CMakeFiles/astra_tests.dir/error_handler_test.cpp.o.d -o CMakeFiles/astra_tests.dir/error_handler_test.cpp.o -c /workspace/github/compiler/tests/error_handler_test.cpp

tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/error_handler_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/error_handler_test.cpp > CMakeFiles/astra_tests.dir/error_handler_test.cpp.i

tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/error_handler_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/error_handler_test.cpp -o CMakeFiles/astra_tests.dir/error_handler_test.cpp.s

tests/CMakeFiles/astra_tests.dir/ast_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/ast_test.cpp.o: /workspace/github/compiler/tests/ast_test.cpp
tests/CMakeFiles/astra_tests.dir/ast_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object tests/CMakeFiles/astra_tests.dir/ast_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/ast_test.cpp.o -MF CMakeFiles/astra_tests.dir/ast_test.cpp.o.d -o CMakeFiles/astra_tests.dir/ast_test.cpp.o -c /workspace/github/compiler/tests/ast_test.cpp

tests/CMakeFiles/astra_tests.dir/ast_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/ast_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/ast_test.cpp > CMakeFiles/astra_tests.dir/ast_test.cpp.i

tests/CMakeFiles/astra_tests.dir/ast_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/ast_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/ast_test.cpp -o CMakeFiles/astra_tests.dir/ast_test.cpp.s

tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.o: /workspace/github/compiler/tests/destructors_test.cpp
tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.o -MF CMakeFiles/astra_tests.dir/destructors_test.cpp.o.d -o CMakeFiles/astra_tests.dir/destructors_test.cpp.o -c /workspace/github/compiler/tests/destructors_test.cpp

tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/destructors_test.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/tests/destructors_test.cpp > CMakeFiles/astra_tests.dir/destructors_test.cpp.i

tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/destructors_test.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/tests/destructors_test.cpp -o CMakeFiles/astra_tests.dir/destructors_test.cpp.s

tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o: tests/CMakeFiles/astra_tests.dir/flags.make
tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o: /workspace/github/compiler/src/ast/ast.cpp
tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o: tests/CMakeFiles/astra_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o -MF CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o.d -o CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o -c /workspace/github/compiler/src/ast/ast.cpp

tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.i"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/github/compiler/src/ast/ast.cpp > CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.i

tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.s"
	cd /workspace/github/build/compiler/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/github/compiler/src/ast/ast.cpp -o CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.s

# Object files for target astra_tests
astra_tests_OBJECTS = \
"CMakeFiles/astra_tests.dir/lexer_test.cpp.o" \
"CMakeFiles/astra_tests.dir/parser_test.cpp.o" \
"CMakeFiles/astra_tests.dir/semantic_test.cpp.o" \
"CMakeFiles/astra_tests.dir/ir_test.cpp.o" \
"CMakeFiles/astra_tests.dir/optimizer_test.cpp.o" \
"CMakeFiles/astra_tests.dir/codegen_test.cpp.o" \
"CMakeFiles/astra_tests.dir/error_handler_test.cpp.o" \
"CMakeFiles/astra_tests.dir/ast_test.cpp.o" \
"CMakeFiles/astra_tests.dir/destructors_test.cpp.o" \
"CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o"

# External object files for target astra_tests
astra_tests_EXTERNAL_OBJECTS =

tests/astra_tests: tests/CMakeFiles/astra_tests.dir/lexer_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/parser_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/semantic_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/ir_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/optimizer_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/codegen_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/error_handler_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/ast_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/destructors_test.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/__/src/ast/ast.cpp.o
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/build.make
tests/astra_tests: /usr/lib/x86_64-linux-gnu/libgtest.a
tests/astra_tests: tests/CMakeFiles/astra_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/workspace/github/build/compiler/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable astra_tests"
	cd /workspace/github/build/compiler/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/astra_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/astra_tests.dir/build: tests/astra_tests
.PHONY : tests/CMakeFiles/astra_tests.dir/build

tests/CMakeFiles/astra_tests.dir/clean:
	cd /workspace/github/build/compiler/tests && $(CMAKE_COMMAND) -P CMakeFiles/astra_tests.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/astra_tests.dir/clean

tests/CMakeFiles/astra_tests.dir/depend:
	cd /workspace/github/build/compiler && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /workspace/github/compiler /workspace/github/compiler/tests /workspace/github/build/compiler /workspace/github/build/compiler/tests /workspace/github/build/compiler/tests/CMakeFiles/astra_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/astra_tests.dir/depend

