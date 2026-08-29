#!/bin/bash

# Run ToF-drivers tests in Docker container
# Usage: ./run_docker.sh [options]
#
# Options:
#   --device <path>     Specify V4L2 device (default: /dev/video0)
#   --i2c <path>        Specify I2C device (default: /dev/i2c-1)
#   --image <name>      Docker image name (default: tof-driver-tests)
#   --filter <pattern>  GoogleTest filter pattern
#   --repeat <n>        Repeat tests N times
#   --interactive       Run interactive shell instead of tests
#   --no-mount          Don't mount source (use pre-built tests in image)
#   --build             Build tests inside container (when source mounted)
#   --help              Show this help

set -e

# Default values
IMAGE_NAME="tof-driver-tests:latest"
VIDEO_DEVICE="/dev/video0"
I2C_DEVICE="/dev/i2c-1"
GTEST_FILTER=""
GTEST_REPEAT=""
INTERACTIVE=false
MOUNT_SOURCE=true
BUILD_IN_CONTAINER=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --device)
            VIDEO_DEVICE="$2"
            shift 2
            ;;
        --i2c)
            I2C_DEVICE="$2"
            shift 2
            ;;
        --image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        --filter)
            GTEST_FILTER="--gtest_filter=$2"
            shift 2
            ;;
        --repeat)
            GTEST_REPEAT="--gtest_repeat=$2"
            shift 2
            ;;
        --interactive)
            INTERACTIVE=true
            shift
            ;;
        --no-mount)
            MOUNT_SOURCE=false
            shift
            ;;
        --build)
            BUILD_IN_CONTAINER=true
            shift
            ;;
        --help)
            grep "^#" "$0" | grep -v "^#!/" | sed 's/^# //'
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "============================================"
# Create result directories on host
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${SCRIPT_DIR}/test-results"
mkdir -p "${SCRIPT_DIR}/test-logs"

echo "============================================"
echo "Running ToF-drivers Tests in Docker"
echo "============================================"
echo "Image: ${IMAGE_NAME}"
echo "Video device: ${VIDEO_DEVICE}"
echo "I2C device: ${I2C_DEVICE}"
[[ -n "${GTEST_FILTER}" ]] && echo "Test filter: ${GTEST_FILTER}"
[[ -n "${GTEST_REPEAT}" ]] && echo "Repeat: ${GTEST_REPEAT}"
echo "Results will be saved to: ${SCRIPT_DIR}/test-results/"
echo "Logs will be saved to: ${SCRIPT_DIR}/test-logs/"
echo ""

# Check if devices exist
if [ ! -e "${VIDEO_DEVICE}" ]; then
    echo "WARNING: Video device ${VIDEO_DEVICE} not found"
    echo "V4L2 tests may skip or fail"
fi

if [ ! -e "${I2C_DEVICE}" ]; then
    echo "WARNING: I2C device ${I2C_DEVICE} not found"
    echo "I2C tests may skip or fail"
fi

# Docker run options
DOCKER_OPTS=(
    --rm
    --privileged
    --device="${VIDEO_DEVICE}"
    --device="${I2C_DEVICE}"
    -v "${SCRIPT_DIR}/test-results:/workspace/test-results"
    -v "${SCRIPT_DIR}/test-logs:/workspace/test-logs"
    --network host
    -e GTEST_OUTPUT="json:/workspace/test-results/gtest_results.json"
    -e TEST_RESULTS_DIR="/workspace/test-results"
    -e TEST_LOGS_DIR="/workspace/test-logs"
)

# Mount source code for local development (unless --no-mount specified)
if [ "${MOUNT_SOURCE}" = true ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    TESTS_DIR="$(cd "$SCRIPT_DIR/../" && pwd)"
    DOCKER_OPTS+=(-v "${TESTS_DIR}:/workspace/ToF-drivers/tests")
    echo "Mounting source: ${TESTS_DIR}"
fi

if [ "${INTERACTIVE}" = true ]; then
    echo "Starting interactive shell..."
    docker run "${DOCKER_OPTS[@]}" -it "${IMAGE_NAME}" /bin/bash
else
    echo "Running tests..."
    echo ""
    
    # Build test command
    TEST_CMD="cd /workspace/ToF-drivers/tests && "
    
    # Auto-detect if build is needed (when source mounted and build dir missing)
    # Or explicit --build flag
    if [ "${BUILD_IN_CONTAINER}" = true ]; then
        echo "Building tests in container..."
        TEST_CMD+="mkdir -p build && cd build && "
        TEST_CMD+="cmake .. && cmake --build . -j\$(nproc) && "
    elif [ "${MOUNT_SOURCE}" = true ]; then
        echo "Auto-detecting if build needed..."
        TEST_CMD+="if [ ! -d build ] || [ ! -f build/driver-v4l2_test ] || [ ! -f build/driver-i2c_test ]; then "
        TEST_CMD+="echo 'Build directory missing or incomplete, building tests...' && "
        TEST_CMD+="rm -rf build && mkdir -p build && cd build && "
        TEST_CMD+="cmake .. && cmake --build . -j\$(nproc); "
        TEST_CMD+="else echo 'Using existing build'; cd build; fi && "
    else
        TEST_CMD+="cd build && "
    fi
    
    # Run tests and save results
    TEST_CMD+="./driver-v4l2_test --device=${VIDEO_DEVICE} ${GTEST_FILTER} ${GTEST_REPEAT} --gtest_output=json:/workspace/test-results/v4l2_results.json 2>&1 | tee /workspace/test-logs/v4l2_test.log && "
    TEST_CMD+="./driver-i2c_test --device=${I2C_DEVICE} ${GTEST_FILTER} ${GTEST_REPEAT} --gtest_output=json:/workspace/test-results/i2c_results.json 2>&1 | tee /workspace/test-logs/i2c_test.log"
    
    docker run "${DOCKER_OPTS[@]}" "${IMAGE_NAME}" /bin/bash -c "${TEST_CMD}"
    
    # Print results location
    echo ""
    echo "Test results saved to:"
    echo "  - ${SCRIPT_DIR}/test-results/v4l2_results.json"
    echo "  - ${SCRIPT_DIR}/test-results/i2c_results.json"
    echo "Test logs saved to:"
    echo "  - ${SCRIPT_DIR}/test-logs/v4l2_test.log"
    echo "  - ${SCRIPT_DIR}/test-logs/i2c_test.log"
fi

echo ""
echo "============================================"
echo "Test run complete!"
echo "Results: ${SCRIPT_DIR}/test-results/"
echo "Logs: ${SCRIPT_DIR}/test-logs/"
echo ""
echo "View results:"
echo "  cat ${SCRIPT_DIR}/test-results/v4l2_results.json"
echo "  cat ${SCRIPT_DIR}/test-results/i2c_results.json"
echo "============================================"
