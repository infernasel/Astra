#!/bin/bash

# Script to build ASTRA compiler for Windows x64 (minimal version)
# This script uses MinGW for cross-compilation

# Create a temporary directory for the minimal build
mkdir -p /tmp/astra_minimal_build
cd /tmp/astra_minimal_build

# Copy necessary files
mkdir -p src/utils
cp /workspace/github/compiler/src/main_minimal.cpp src/
cp /workspace/github/compiler/src/utils/error_handler_minimal.h src/utils/
cp /workspace/github/compiler/src/utils/error_handler_minimal.cpp src/utils/
cp /workspace/github/compiler/src/utils/options_minimal.h src/utils/
cp /workspace/github/compiler/src/utils/options_minimal.cpp src/utils/
cp /workspace/github/compiler/CMakeLists_minimal.txt CMakeLists.txt
cp /workspace/github/windows-toolchain.cmake .

# Configure with CMake for Windows
cmake -DCMAKE_TOOLCHAIN_FILE=windows-toolchain.cmake .

# Build
make -j$(nproc)

# Create release directory if it doesn't exist
mkdir -p /workspace/github/release/windows

# Copy executable to release directory
cp astrac.exe /workspace/github/release/windows/

echo "Build completed. Windows executable is in release/windows/astrac.exe"