#!/bin/bash

# Run ToF-drivers tests from CSV configuration
# Usage: ./run_tests.sh [options]
#   -n N        Repeat all tests N times
#   -f FILE     CSV file path (default: test_csvs/driver_test_list.csv)
#   -p PRIORITY Filter by priority (P1, P2, P3)
#   -c CATEGORY Filter by category (V4L2, I2C)
#   -v          Verbose output
#   -h          Show help

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSV_FILE="${SCRIPT_DIR}/test_csvs/driver_test_list.csv"
REPEAT_COUNT=1
PRIORITY_FILTER=""
CATEGORY_FILTER=""
VERBOSE=false
BUILD_DIR="${SCRIPT_DIR}/build"

# Use environment variable for results directory if set (Docker)
RESULTS_DIR="${TEST_RESULTS_DIR:-${SCRIPT_DIR}/test-results}"
LOGS_DIR="${TEST_LOGS_DIR:-${SCRIPT_DIR}/test-logs}"

# Create result directories
mkdir -p "${RESULTS_DIR}"
mkdir -p "${LOGS_DIR}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -n)
            REPEAT_COUNT="$2"
            shift 2
            ;;
        -f)
            CSV_FILE="$2"
            shift 2
            ;;
        -p)
            PRIORITY_FILTER="$2"
            shift 2
            ;;
        -c)
            CATEGORY_FILTER="$2"
            shift 2
            ;;
        -v)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  -n N        Repeat all tests N times"
            echo "  -f FILE     CSV file path (default: test_csvs/driver_test_list.csv)"
            echo "  -p PRIORITY Filter by priority (P1, P2, P3)"
            echo "  -c CATEGORY Filter by category (V4L2, I2C)"
            echo "  -v          Verbose output"
            echo "  -h          Show help"
            echo ""
            echo "Examples:"
            echo "  $0                      # Run all tests once"
            echo "  $0 -n 3                 # Run all tests 3 times"
            echo "  $0 -p P1                # Run only P1 priority tests"
            echo "  $0 -c V4L2              # Run only V4L2 tests"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h for help"
            exit 1
            ;;
    esac
done

# Check if CSV file exists
if [ ! -f "${CSV_FILE}" ]; then
    echo -e "${RED}ERROR: CSV file not found: ${CSV_FILE}${NC}"
    exit 1
fi

# Check if build directory exists
if [ ! -d "${BUILD_DIR}" ]; then
    echo -e "${RED}ERROR: Build directory not found: ${BUILD_DIR}${NC}"
    echo "Please build tests first:"
    echo "  cd ${SCRIPT_DIR}"
    echo "  mkdir -p build && cd build"
    echo "  cmake .. && make -j"
    exit 1
fi

echo "============================================"
echo "ToF-drivers Test Runner"
echo "============================================"
echo "CSV File: ${CSV_FILE}"
echo "Repeat Count: ${REPEAT_COUNT}"
[ -n "${PRIORITY_FILTER}" ] && echo "Priority Filter: ${PRIORITY_FILTER}"
[ -n "${CATEGORY_FILTER}" ] && echo "Category Filter: ${CATEGORY_FILTER}"
echo "Results Directory: ${RESULTS_DIR}"
echo "Logs Directory: ${LOGS_DIR}"
echo ""

# Create summary file
SUMMARY_FILE="${RESULTS_DIR}/test_summary_$(date +%Y%m%d_%H%M%S).txt"
echo "ToF-drivers Test Results" > "${SUMMARY_FILE}"
echo "Date: $(date)" >> "${SUMMARY_FILE}"
echo "CSV: ${CSV_FILE}" >> "${SUMMARY_FILE}"
echo "" >> "${SUMMARY_FILE}"

# Statistics
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0
SKIPPED_TESTS=0

# Read CSV and execute tests
while IFS=',' read -r test_id category subcategory test_file test_name description preconditions cmd_params test_steps hw_required priority expected actual comments; do
    # Skip header line
    if [[ "$test_id" == "Test ID" ]]; then
        continue
    fi
    
    # Skip empty lines
    if [[ -z "$test_id" ]]; then
        continue
    fi
    
    # Apply priority filter
    if [[ -n "${PRIORITY_FILTER}" && "$priority" != "${PRIORITY_FILTER}" ]]; then
        continue
    fi
    
    # Apply category filter
    if [[ -n "${CATEGORY_FILTER}" && "$subcategory" != "${CATEGORY_FILTER}" ]]; then
        continue
    fi
    
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    
    # Extract test executable and filter from test file
    if [[ "$test_file" == "driver-v4l2_test.cpp" ]]; then
        TEST_BINARY="${BUILD_DIR}/driver-v4l2_test"
    elif [[ "$test_file" == "driver-i2c_test.cpp" ]]; then
        TEST_BINARY="${BUILD_DIR}/driver-i2c_test"
    else
        echo -e "${YELLOW}WARNING: Unknown test file: ${test_file}${NC}"
        SKIPPED_TESTS=$((SKIPPED_TESTS + 1))
        continue
    fi
    
    # Check if binary exists
    if [ ! -x "${TEST_BINARY}" ]; then
        echo -e "${RED}FAIL: ${test_id} - ${test_name}${NC}"
        echo "  Binary not found: ${TEST_BINARY}"
        FAILED_TESTS=$((FAILED_TESTS + 1))
        continue
    fi
    
    # Extract GoogleTest filter from test name
    GTEST_FILTER="*${test_name}"
    
    # Run test multiple times
    for i in $(seq 1 ${REPEAT_COUNT}); do
        if [ ${VERBOSE} = true ]; then
            echo ""
            echo "--------------------------------------------"
            echo "Test ID: ${test_id}"
            echo "Category: ${category} / ${subcategory}"
            echo "Test: ${test_name}"
            echo "Description: ${description}"
            echo "Priority: ${priority}"
            if [ ${REPEAT_COUNT} -gt 1 ]; then
                echo "Iteration: ${i}/${REPEAT_COUNT}"
            fi
            echo "Command: ${TEST_BINARY} --gtest_filter=${GTEST_FILTER}"
            echo "--------------------------------------------"
        else
            if [ ${REPEAT_COUNT} -gt 1 ]; then
                echo -n "Running ${test_id} (${i}/${REPEAT_COUNT}): "
            else
                echo -n "Running ${test_id}: "
            fi
        fi
        
        # Execute test and save results
        LOG_FILE="${LOGS_DIR}/${test_id}_${test_name}.log"
        if ${TEST_BINARY} --gtest_filter="${GTEST_FILTER}" --gtest_color=yes > "${LOG_FILE}" 2>&1; then
            if [ ${VERBOSE} = true ]; then
                echo -e "${GREEN}PASS${NC}"
                cat "${LOG_FILE}"
            else
                echo -e "${GREEN}PASS${NC}"
            fi
            echo "${test_id},PASS,${test_name}" >> "${SUMMARY_FILE}"
            if [ ${i} -eq ${REPEAT_COUNT} ]; then
                PASSED_TESTS=$((PASSED_TESTS + 1))
            fi
        else
            if [ ${VERBOSE} = true ]; then
                echo -e "${RED}FAIL${NC}"
                cat "${LOG_FILE}"
            else
                echo -e "${RED}FAIL${NC}"
                echo "  See details: cat ${LOG_FILE}"
            fi
            echo "${test_id},FAIL,${test_name}" >> "${SUMMARY_FILE}"
            if [ ${i} -eq ${REPEAT_COUNT} ]; then
                FAILED_TESTS=$((FAILED_TESTS + 1))
            fi
            # Don't break on failure, continue to next iteration
        fi
    done
    
done < "${CSV_FILE}"

# Print summary
echo ""
echo "============================================"
echo "Test Summary"
echo "============================================"
echo "Total Tests: ${TOTAL_TESTS}"
echo -e "${GREEN}Passed: ${PASSED_TESTS}${NC}"
echo -e "${RED}Failed: ${FAILED_TESTS}${NC}"
echo -e "${YELLOW}Skipped: ${SKIPPED_TESTS}${NC}"
echo "============================================"
echo ""
echo "Results saved to:"
echo "  Summary: ${SUMMARY_FILE}"
echo "  Logs: ${LOGS_DIR}/"
echo "============================================"

# Save summary statistics
echo "" >> "${SUMMARY_FILE}"
echo "Summary:" >> "${SUMMARY_FILE}"
echo "  Total: ${TOTAL_TESTS}" >> "${SUMMARY_FILE}"
echo "  Passed: ${PASSED_TESTS}" >> "${SUMMARY_FILE}"
echo "  Failed: ${FAILED_TESTS}" >> "${SUMMARY_FILE}"
echo "  Skipped: ${SKIPPED_TESTS}" >> "${SUMMARY_FILE}"

# Exit with error if any tests failed
if [ ${FAILED_TESTS} -gt 0 ]; then
    exit 1
fi

exit 0
