#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <gtest/gtest.h>
#include <string>
#include <iostream>

namespace test_utils {

// Global variables for test configuration
extern std::string g_device_address;
extern std::string g_config_path;
extern std::string g_frame_mode;

/**
 * @brief TestRunner class - Manages test execution and command-line arguments
 * 
 * This class handles:
 * - Command-line argument parsing
 * - GoogleTest initialization
 * - Test execution
 */
class TestRunner {
public:
    explicit TestRunner(const std::string& test_name);
    
    /**
     * @brief Initialize the test runner
     * @param argc Argument count
     * @param argv Argument values
     * @return -1 if tests should run, 0 if help was shown, 1 on error
     */
    int initialize(int argc, char** argv);
    
    /**
     * @brief Run all tests
     * @return Test result code (0 = success)
     */
    int runTests();
    
    /**
     * @brief Print usage information
     */
    void printUsage();

private:
    std::string test_name_;
    bool initialized_;
};

} // namespace test_utils

#endif // TEST_UTILS_H
