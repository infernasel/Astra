#!/bin/bash

# Script to build ASTRA compiler for Windows x64
# This script uses MinGW for cross-compilation

# Navigate to compiler directory
cd compiler

# Create build directory if it doesn't exist
mkdir -p build_windows

# Navigate to build directory
cd build_windows

# Configure with CMake for Windows
cmake -DCMAKE_TOOLCHAIN_FILE=../../windows-toolchain.cmake ..

# Build
make -j$(nproc)

# Create release directory if it doesn't exist
mkdir -p ../../release/windows

# Copy executable to release directory
cp astrac.exe ../../release/windows/

echo "Build completed. Windows executable is in release/windows/astrac.exe"