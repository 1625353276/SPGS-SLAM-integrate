#!/bin/bash
# SPGS-SLAM Build Script
# This script sets up and compiles the SPGS-SLAM project

set -e  # Exit on error

echo "=========================================="
echo "  SPGS-SLAM Build Script"
echo "=========================================="
echo ""

# Project root directory
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

# Number of CPU cores for parallel compilation
NUM_CORES=$(nproc)

echo "Project root: $PROJECT_ROOT"
echo "Build directory: $BUILD_DIR"
echo "Using $NUM_CORES cores for compilation"
echo ""

# Step 1: Create build directory if it doesn't exist
echo "[1/4] Creating build directory..."
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
    echo "✓ Build directory created"
else
    echo "✓ Build directory already exists"
fi
echo ""

# Step 2: Run CMake configuration
echo "[2/4] Running CMake configuration..."
cd "$BUILD_DIR"
if cmake ..; then
    echo "✓ CMake configuration successful"
else
    echo "✗ CMake configuration failed"
    echo "Please check error messages above"
    exit 1
fi
echo ""

# Step 3: Compile the project
echo "[3/4] Compiling project with make -j$NUM_CORES..."
if make -j"$NUM_CORES"; then
    echo "✓ Compilation successful"
else
    echo "✗ Compilation failed"
    echo "Please check error messages above"
    exit 1
fi
echo ""

# Step 4: Verify build output
echo "[4/4] Verifying build output..."
echo ""
echo "Generated libraries:"
if [ -d "$PROJECT_ROOT/lib" ]; then
    ls -lh "$PROJECT_ROOT/lib"/*.so 2>/dev/null || echo "  No .so files found"
else
    echo "  lib/ directory not found"
fi
echo ""
echo "Generated executables:"
if [ -d "$PROJECT_ROOT/bin" ]; then
    ls -lh "$PROJECT_ROOT/bin"/* 2>/dev/null || echo "  No executables found"
else
    echo "  bin/ directory not found"
fi
echo ""

# Success message
echo "=========================================="
echo "  Build completed successfully!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Test executables in bin/ directory"
echo "  2. Run example programs with appropriate datasets"
echo "  3. Check BUILD_STATUS.md for detailed build status"
echo ""
echo "Example usage:"
echo "  cd $PROJECT_ROOT"
echo "  ./bin/tum_mono ORB-SLAM3/Vocabulary/ORBvoc.txt cfg/ORB_SLAM3/Monocular/TUM1.yaml /path/to/dataset"
echo ""