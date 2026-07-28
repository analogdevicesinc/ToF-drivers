#include "test_utils.h"
#include <cstring>

namespace test_utils {

// Define global variables
std::string g_device_address = "";
std::string g_config_path = "";
std::string g_frame_mode = "0";

TestRunner::TestRunner(const std::string& test_name)
    : test_name_(test_name), initialized_(false) {
}

int TestRunner::initialize(int argc, char** argv) {
    // Parse custom arguments before GoogleTest
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage();
            return 0;
        } else if (strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            g_device_address = argv[++i];
        } else if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            g_config_path = argv[++i];
        } else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            g_frame_mode = argv[++i];
        }
    }
    
    // Initialize GoogleTest
    ::testing::InitGoogleTest(&argc, argv);
    initialized_ = true;
    
    return -1; // Continue with tests
}

int TestRunner::runTests() {
    if (!initialized_) {
        std::cerr << "Error: TestRunner not initialized. Call initialize() first." << std::endl;
        return 1;
    }
    
    return RUN_ALL_TESTS();
}

void TestRunner::printUsage() {
    std::cout << "Usage: " << test_name_ << " [OPTIONS]\n\n";
    std::cout << "Driver Test Options:\n";
    std::cout << "  --device ADDR    Device address\n";
    std::cout << "  --config PATH    Configuration file path\n";
    std::cout << "  --mode MODE      Frame mode\n";
    std::cout << "  -h, --help       Show this help\n\n";
    std::cout << "GoogleTest Options:\n";
    std::cout << "  --gtest_filter=PATTERN    Run only tests matching pattern\n";
    std::cout << "  --gtest_repeat=N          Repeat tests N times\n";
    std::cout << "  --gtest_list_tests        List all tests\n";
    std::cout << "  --gtest_color=yes|no      Enable/disable colored output\n";
}

} // namespace test_utils
