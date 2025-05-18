#!/bin/bash

# ASTRA Language Test Script
# This script runs tests for both the compiler and interpreter components of ASTRA

set -e  # Exit on error

# Test compiler
echo "Testing ASTRA compiler..."
cd build/compiler
ctest -V
cd ../..

# Test interpreter
echo "Testing ASTRA interpreter..."
cd build/interpreter
ctest -V
cd ../..

echo "All tests completed successfully!"