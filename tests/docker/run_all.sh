#!/bin/bash

################################################################################
# ToF-drivers Complete Test Automation Script
################################################################################
# This script provides end-to-end automation for:
#   1. Building Docker image
#   2. Running system tests (V4L2 + I2C)
#   3. Generating test reports and summaries
#
# Usage:
#   ./run_all.sh [options]
#
# Options:
#   --skip-build        Skip Docker image build
#   --ci-mode           Build Docker in CI/CD mode (pre-build tests)
#   --git-repo <url>    Clone from git repository during Docker build
#   --git-branch <name> Git branch to clone (default: main)
#   --build-jobs <n>    Number of parallel build jobs (default: auto)
#   --priority <P1|P2|P3>  Run only specific priority tests
#   --category <V4L2|I2C>  Run only specific category tests
#   --repeat <N>        Repeat tests N times
#   --device <path>     Specify V4L2 device (default: /dev/video0)
#   --i2c <path>        Specify I2C device (default: /dev/i2c-1)
#   --clean             Clean all previous results and build artifacts
#   --help              Show this help message
#
# Examples:
#   ./run_all.sh                          # Full run: build + all tests
#   ./run_all.sh --skip-build             # Run tests only (use existing image)
#   ./run_all.sh --priority P1            # Build + run P1 tests only
#   ./run_all.sh --clean --ci-mode        # Clean build in CI mode
#   ./run_all.sh --category V4L2 --repeat 3  # V4L2 tests 3 times
#   ./run_all.sh --git-repo https://github.com/user/ToF.git --git-branch feature/test
#   ./run_all.sh --clean --git-repo <url> --git-branch develop --ci-mode
#
################################################################################

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
SKIP_BUILD=false
CI_MODE=false
GIT_REPO=""
GIT_BRANCH="main"
BUILD_JOBS=""
PRIORITY=""
CATEGORY=""
REPEAT=""
VIDEO_DEVICE="/dev/video0"
I2C_DEVICE="/dev/i2c-1"
CLEAN=false
IMAGE_NAME="tof-driver-tests"

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TESTS_DIR="$(cd "$SCRIPT_DIR/../" && pwd)"

# Timestamp for this run
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${BLUE}"
    echo "============================================"
    echo "$1"
    echo "============================================"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

show_help() {
    grep "^#" "$0" | grep -v "^#!/" | sed 's/^# //' | sed 's/^#//'
    exit 0
}

################################################################################
# Parse Arguments
################################################################################

while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --ci-mode)
            CI_MODE=true
            shift
            ;;
        --git-repo)
            GIT_REPO="$2"
            shift 2
            ;;
        --git-branch)
            GIT_BRANCH="$2"
            shift 2
            ;;
        --build-jobs)
            BUILD_JOBS="$2"
            shift 2
            ;;
        --priority)
            PRIORITY="$2"
            shift 2
            ;;
        --category)
            CATEGORY="$2"
            shift 2
            ;;
        --repeat)
            REPEAT="$2"
            shift 2
            ;;
        --device)
            VIDEO_DEVICE="$2"
            shift 2
            ;;
        --i2c)
            I2C_DEVICE="$2"
            shift 2
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --help)
            show_help
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

################################################################################
# Main Execution
################################################################################

print_header "ToF-drivers Complete Test Automation"
echo "Start Time: $(date)"
echo "Test Run ID: ${TIMESTAMP}"
echo ""

# Step 0: Clean if requested
if [ "${CLEAN}" = true ]; then
    print_header "Step 0: Cleaning Previous Artifacts"
    
    print_info "Removing test results and logs..."
    rm -rf "${SCRIPT_DIR}/test-results" "${SCRIPT_DIR}/test-logs"
    
    print_info "Removing build artifacts..."
    rm -rf "${TESTS_DIR}/build"
    
    if [ "${SKIP_BUILD}" = false ]; then
        print_info "Removing Docker image..."
        docker rmi "${IMAGE_NAME}:latest" 2>/dev/null || true
    fi
    
    print_success "Cleanup complete"
    echo ""
fi

# Create result directories
mkdir -p "${SCRIPT_DIR}/test-results"
mkdir -p "${SCRIPT_DIR}/test-logs"

# Step 1: Build Docker Image
if [ "${SKIP_BUILD}" = false ]; then
    print_header "Step 1: Building Docker Image"
    
    cd "${SCRIPT_DIR}"
    
    BUILD_CMD="bash ./build_docker.sh ${IMAGE_NAME}"
    
    if [ "${CI_MODE}" = true ]; then
        print_info "Building in CI/CD mode (pre-built tests)..."
        BUILD_CMD+=" --ci"
    else
        print_info "Building in local development mode..."
    fi
    
    if [ -n "${GIT_REPO}" ]; then
        print_info "Cloning from git repository..."
        print_info "  Repository: ${GIT_REPO}"
        print_info "  Branch: ${GIT_BRANCH}"
        BUILD_CMD+=" --git-repo ${GIT_REPO} --git-branch ${GIT_BRANCH}"
    fi
    
    if [ -n "${BUILD_JOBS}" ]; then
        BUILD_CMD+=" --build-jobs ${BUILD_JOBS}"
    fi
    
    eval ${BUILD_CMD}
    
    if [ $? -eq 0 ]; then
        print_success "Docker image built successfully"
    else
        print_error "Docker build failed"
        exit 1
    fi
    echo ""
else
    print_header "Step 1: Skipping Docker Build"
    print_info "Using existing image: ${IMAGE_NAME}:latest"
    
    # Verify image exists
    if ! docker image inspect "${IMAGE_NAME}:latest" >/dev/null 2>&1; then
        print_error "Image ${IMAGE_NAME}:latest not found"
        print_info "Run without --skip-build to build the image first"
        exit 1
    fi
    echo ""
fi

# Step 2: Check Hardware Availability
print_header "Step 2: Hardware Check"

if [ -e "${VIDEO_DEVICE}" ]; then
    print_success "V4L2 device found: ${VIDEO_DEVICE}"
else
    print_warning "V4L2 device not found: ${VIDEO_DEVICE}"
    print_info "V4L2 tests may skip or fail"
fi

if [ -e "${I2C_DEVICE}" ]; then
    print_success "I2C device found: ${I2C_DEVICE}"
else
    print_warning "I2C device not found: ${I2C_DEVICE}"
    print_info "I2C tests may skip or fail"
fi
echo ""

# Step 3: Run Tests
print_header "Step 3: Running System Tests"

# Build test arguments
TEST_ARGS=""
[ -n "${PRIORITY}" ] && TEST_ARGS+="-p ${PRIORITY} "
[ -n "${CATEGORY}" ] && TEST_ARGS+="-c ${CATEGORY} "
[ -n "${REPEAT}" ] && TEST_ARGS+="-n ${REPEAT} "

if [ -n "${TEST_ARGS}" ]; then
    print_info "Test filters: ${TEST_ARGS}"
fi

print_info "Running CSV-based test suite..."
echo ""

# Run tests and capture exit code
set +e
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
    "${IMAGE_NAME}:latest" \
    /bin/bash -c "cd /workspace/ToF-drivers/tests && \
        if [ ! -d build ] || [ ! -f build/driver-v4l2_test ] || [ ! -f build/driver-i2c_test ]; then \
            echo 'Building tests...' && \
            rm -rf build && mkdir -p build && cd build && \
            cmake .. && cmake --build . -j\$(nproc) && cd ..; \
        fi && \
        chmod +x run_tests.sh && \
        bash ./run_tests.sh ${TEST_ARGS}"

TEST_EXIT_CODE=$?
set -e

echo ""
if [ ${TEST_EXIT_CODE} -eq 0 ]; then
    print_success "All tests passed"
else
    print_warning "Some tests failed or encountered errors"
fi
echo ""

# Step 4: Generate Summary Report
print_header "Step 4: Generating Test Report"

REPORT_FILE="${SCRIPT_DIR}/test-results/test_report_${TIMESTAMP}.txt"

cat > "${REPORT_FILE}" << EOF
================================================================================
ToF-drivers System Test Report
================================================================================
Test Run ID: ${TIMESTAMP}
Date: $(date)
Host: $(hostname)
Docker Image: ${IMAGE_NAME}:latest

Hardware Configuration:
  V4L2 Device: ${VIDEO_DEVICE}
  I2C Device: ${I2C_DEVICE}

Test Configuration:
  Priority Filter: ${PRIORITY:-All}
  Category Filter: ${CATEGORY:-All}
  Repeat Count: ${REPEAT:-1}
  Build Mode: $([ "${CI_MODE}" = true ] && echo "CI/CD" || echo "Local Dev")

================================================================================
Test Results Summary
================================================================================

EOF

# Find most recent test summary (check both test-results and test-logs)
LATEST_SUMMARY=$(ls -t "${SCRIPT_DIR}/test-results/test_summary_"*.txt "${SCRIPT_DIR}/test-logs/test_summary_"*.txt 2>/dev/null | head -n1)

if [ -n "${LATEST_SUMMARY}" ] && [ -f "${LATEST_SUMMARY}" ]; then
    cat "${LATEST_SUMMARY}" >> "${REPORT_FILE}"
else
    echo "No test summary found" >> "${REPORT_FILE}"
fi

cat >> "${REPORT_FILE}" << EOF

================================================================================
Test Artifacts
================================================================================

Results Directory: ${SCRIPT_DIR}/test-results/
Logs Directory: ${SCRIPT_DIR}/test-logs/

Available Files:
EOF

echo "" >> "${REPORT_FILE}"
ls -lh "${SCRIPT_DIR}/test-results/" 2>/dev/null >> "${REPORT_FILE}" || echo "  No result files" >> "${REPORT_FILE}"
echo "" >> "${REPORT_FILE}"
ls -lh "${SCRIPT_DIR}/test-logs/" 2>/dev/null | head -20 >> "${REPORT_FILE}" || echo "  No log files" >> "${REPORT_FILE}"

cat >> "${REPORT_FILE}" << EOF

================================================================================
End of Report
================================================================================
EOF

print_success "Test report generated: ${REPORT_FILE}"
echo ""

# Step 5: Display Results
print_header "Step 5: Test Results Summary"

if [ -n "${LATEST_SUMMARY}" ] && [ -f "${LATEST_SUMMARY}" ]; then
    cat "${LATEST_SUMMARY}"
else
    print_warning "No test summary found"
fi

echo ""
print_header "Test Automation Complete"
echo "End Time: $(date)"
echo ""
echo "Results Location:"
echo "  Report:  ${REPORT_FILE}"
echo "  Results: ${SCRIPT_DIR}/test-results/"
echo "  Logs:    ${SCRIPT_DIR}/test-logs/"
echo ""
echo "Quick View Commands:"
echo "  cat ${REPORT_FILE}"
echo "  cat ${SCRIPT_DIR}/test-results/*.json"
echo "  ls -lh ${SCRIPT_DIR}/test-logs/"
echo ""

# Exit with test result code
exit ${TEST_EXIT_CODE}
