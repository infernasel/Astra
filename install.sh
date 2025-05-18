#!/bin/bash

# ASTRA Language Installation Script
# This script installs the ASTRA language tools to the system

set -e  # Exit on error

# Default installation directory
INSTALL_DIR="/usr/local"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --prefix=*)
        INSTALL_DIR="${key#*=}"
        shift
        ;;
        --help)
        echo "ASTRA Language Installer"
        echo "Usage: $0 [options]"
        echo ""
        echo "Options:"
        echo "  --prefix=DIR    Install files in DIR [default: /usr/local]"
        echo "  --help          Display this help message"
        exit 0
        ;;
        *)
        echo "Unknown option: $key"
        echo "Use --help for usage information"
        exit 1
        ;;
    esac
done

# Check if build exists
if [ ! -f "bin/astrac" ] || [ ! -f "bin/astra" ]; then
    echo "Error: ASTRA binaries not found. Please run build.sh first."
    exit 1
fi

# Create installation directories
echo "Installing ASTRA to $INSTALL_DIR..."
mkdir -p "$INSTALL_DIR/bin"
mkdir -p "$INSTALL_DIR/share/astra/examples"
mkdir -p "$INSTALL_DIR/share/astra/docs"
mkdir -p "$INSTALL_DIR/share/astra/stdlib"

# Install binaries
cp bin/astrac "$INSTALL_DIR/bin/"
cp bin/astra "$INSTALL_DIR/bin/"

# Install examples
cp -r examples/* "$INSTALL_DIR/share/astra/examples/"

# Install documentation
cp -r docs/* "$INSTALL_DIR/share/astra/docs/"

# Create astra-run script for easy execution
cat > "$INSTALL_DIR/bin/astra-run" << 'EOF'
#!/bin/bash
# ASTRA Run Script - Compiles and runs ASTRA programs

if [ $# -lt 1 ]; then
    echo "Usage: astra-run <source_file.astra> [arguments]"
    exit 1
fi

SOURCE_FILE="$1"
shift

if [ ! -f "$SOURCE_FILE" ]; then
    echo "Error: Source file '$SOURCE_FILE' not found"
    exit 1
fi

# Get the base filename without extension
BASENAME=$(basename "$SOURCE_FILE" .astra)
OUTPUT_FILE="${BASENAME}.avm"

# Compile the source file
astrac "$SOURCE_FILE" -o "$OUTPUT_FILE"
if [ $? -ne 0 ]; then
    echo "Compilation failed"
    exit 1
fi

# Run the compiled file
astra "$OUTPUT_FILE" "$@"
EOF

chmod +x "$INSTALL_DIR/bin/astra-run"

echo "ASTRA has been successfully installed to $INSTALL_DIR"
echo ""
echo "You can now run ASTRA programs using:"
echo "  astra-run program.astra"
echo ""
echo "For more information, see the documentation in:"
echo "  $INSTALL_DIR/share/astra/docs"