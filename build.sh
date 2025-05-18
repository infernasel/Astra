#!/bin/bash

# ASTRA Language Build Script
# This script builds both the compiler and interpreter components of ASTRA

set -e  # Exit on error

# Create build directories
mkdir -p build/compiler
mkdir -p build/interpreter

# Build compiler
echo "Building ASTRA compiler..."
cd build/compiler
cmake ../../compiler -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ../..

# Build interpreter
echo "Building ASTRA interpreter..."
cd build/interpreter
cmake ../../interpreter -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ../..

# Copy executables to bin directory
mkdir -p bin
cp build/compiler/astrac bin/
cp build/interpreter/astra bin/

echo "Build completed successfully!"
echo "Executables are available in the bin directory:"
echo "  - bin/astrac (compiler)"
echo "  - bin/astra (interpreter)"