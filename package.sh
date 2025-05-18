#!/bin/bash

# ASTRA Language Packaging Script
# This script creates installation packages for the ASTRA language

set -e  # Exit on error

VERSION="0.1.0"
PACKAGE_NAME="astra-lang-$VERSION"

# Check if build exists
if [ ! -f "bin/astra" ]; then
    echo "Error: ASTRA binaries not found. Please run build.sh first."
    exit 1
fi

# Create package directory
echo "Creating package $PACKAGE_NAME..."
mkdir -p "dist/$PACKAGE_NAME"

# Copy files to package directory
cp -r bin "dist/$PACKAGE_NAME/"
cp -r examples "dist/$PACKAGE_NAME/"
cp -r docs "dist/$PACKAGE_NAME/"
cp README.md "dist/$PACKAGE_NAME/"
cp install.sh "dist/$PACKAGE_NAME/"

# Create tarball
echo "Creating tarball..."
cd dist
tar -czf "$PACKAGE_NAME.tar.gz" "$PACKAGE_NAME"
cd ..

# Create ZIP archive
echo "Creating ZIP archive..."
cd dist
zip -r "$PACKAGE_NAME.zip" "$PACKAGE_NAME"
cd ..

echo "Packages created successfully:"
echo "  - dist/$PACKAGE_NAME.tar.gz"
echo "  - dist/$PACKAGE_NAME.zip"