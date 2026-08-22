#!/usr/bin/env bash
# build.sh — Quick build + test runner for the Train Collision Avoidance System
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$ROOT/build"

echo "══════════════════════════════════════════"
echo "  Train Collision Avoidance — Build Script"
echo "══════════════════════════════════════════"

# Configure
cmake -B "$BUILD" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_COMPILER=g++ \
      "$ROOT"

# Build (all targets)
cmake --build "$BUILD" --parallel "$(nproc)"

echo ""
echo "Build successful!"
echo ""
echo "Run options:"
echo "  $BUILD/bin/train_sim --help"
echo "  $BUILD/bin/train_sim --scenario rear_end --duration 30"
echo "  $BUILD/bin/train_sim --scenario head_on  --duration 60"
echo "  $BUILD/bin/train_sim --realtime"
echo ""

# Run tests
if [ "${1}" == "--test" ]; then
    echo "Running unit tests..."
    cd "$BUILD"
    ctest --output-on-failure
fi
