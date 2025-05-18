#!/bin/bash

# ASTRA Language Build Script (Interpreter Only)
# This script builds only the interpreter component of ASTRA

set -e  # Exit on error

# Create build directory
mkdir -p build/interpreter

# Build interpreter
echo "Building ASTRA interpreter..."
cd build/interpreter
cmake ../../interpreter -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=OFF
make -j$(nproc) astra
cd ../..

# Copy executable to bin directory
mkdir -p bin
cp build/interpreter/astra bin/

echo "Build completed successfully!"
echo "Interpreter executable is available at bin/astra"