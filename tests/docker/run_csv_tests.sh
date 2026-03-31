#!/bin/bash

# Run CSV-based driver tests in Docker container
# Usage: ./run_csv_tests.sh [run_tests.sh options]
#
# Examples:
#   ./run_csv_tests.sh              # Run all tests
#   ./run_csv_tests.sh -p P1        # Run P1 priority tests only
#   ./run_csv_tests.sh -c V4L2      # Run V4L2 category tests only
#   ./run_csv_tests.sh -c I2C -n 3  # Run I2C tests 3 times
#   ./run_csv_tests.sh -p P1 -v     # Run P1 tests with verbose output

set -e

# Default values
IMAGE_NAME="tof-driver-tests:latest"
VIDEO_DEVICE="/dev/video0"
I2C_DEVICE="/dev/i2c-1"

# Create result directories on host
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${SCRIPT_DIR}/test-results"
mkdir -p "${SCRIPT_DIR}/test-logs"

echo "============================================"
echo "Running CSV-based Driver Tests in Docker"
echo "============================================"
echo "Image: ${IMAGE_NAME}"
echo "Video device: ${VIDEO_DEVICE}"
echo "I2C device: ${I2C_DEVICE}"
echo "Results: ${SCRIPT_DIR}/test-results/"
echo "Logs: ${SCRIPT_DIR}/test-logs/"
echo ""

# Check if devices exist
if [ ! -e "${VIDEO_DEVICE}" ]; then
    echo "WARNING: Video device ${VIDEO_DEVICE} not found"
fi

if [ ! -e "${I2C_DEVICE}" ]; then
    echo "WARNING: I2C device ${I2C_DEVICE} not found"
fi

# Mount source and volumes
TESTS_DIR="$(cd "$SCRIPT_DIR/../" && pwd)"

echo "Mounting source: ${TESTS_DIR}"
echo ""
echo "Running CSV test runner with arguments: $@"
echo ""

# Docker run with CSV test runner
docker run \
    --rm \
    --privileged \
    --device="${VIDEO_DEVICE}" \
    --device="${I2C_DEVICE}" \
    -v "${TESTS_DIR}:/workspace/ToF-drivers/tests" \
    -v "${SCRIPT_DIR}/test-results:/workspace/test-results" \
    -v "${SCRIPT_DIR}/test-logs:/workspace/test-logs" \
    --network host \
    -e GTEST_OUTPUT="json:/workspace/test-results/gtest_results.json" \
    -e TEST_RESULTS_DIR="/workspace/test-results" \
    -e TEST_LOGS_DIR="/workspace/test-logs" \
    "${IMAGE_NAME}" \
    /bin/bash -c "cd /workspace/ToF-drivers/tests && \
        if [ ! -d build ] || [ ! -f build/driver-v4l2_test ] || [ ! -f build/driver-i2c_test ]; then \
            echo 'Building tests...' && \
            rm -rf build && \
            mkdir -p build && cd build && \
            cmake .. && cmake --build . -j\$(nproc) && cd ..; \
        fi && \
        chmod +x run_tests.sh && \
        bash ./run_tests.sh $@"

echo ""
echo "============================================"
echo "CSV Test Run Complete!"
echo "============================================"
echo "Results: ${SCRIPT_DIR}/test-results/"
echo "Logs: ${SCRIPT_DIR}/test-logs/"
echo ""
echo "View summary:"
echo "  ls -lh ${SCRIPT_DIR}/test-logs/test_summary_*.txt"
echo "  cat ${SCRIPT_DIR}/test-logs/test_summary_*.txt"
echo ""
echo "View individual test logs:"
echo "  ls -lh ${SCRIPT_DIR}/test-logs/"
echo "============================================"
