/**
 * @file driver-v4l2_cli_test.cpp
 * @brief V4L2 command-line tool validation tests for ADSD3500 ToF sensor
 * 
 * Tests v4l2-ctl and media-ctl utilities including:
 * - Format listing and querying
 * - Subdevice control enumeration
 * - Media bus code listing
 * - Media topology printing
 */

#include <gtest/gtest.h>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <iostream>

#include "../utilities/test_utils.h"

using namespace test_utils;

/**
 * Helper function to execute command and capture output
 */
std::string executeCommand(const char* cmd) {
    std::string result;
    FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        return "ERROR: Failed to execute command";
    }
    
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }
    
    int status = pclose(pipe);
    if (status != 0 && result.empty()) {
        result = "ERROR: Command failed with exit code " + std::to_string(status);
    }
    
    return result;
}

// Test fixture for V4L2 CLI tests
class V4L2CliTest : public ::testing::Test {
protected:
    std::string device_path_;
    
    void SetUp() override {
        // Default to /dev/video0 if not specified
        device_path_ = g_device_address.empty() ? "/dev/video0" : g_device_address;
        
        // Check if device exists
        if (access(device_path_.c_str(), F_OK) != 0) {
            GTEST_SKIP() << "Device " << device_path_ << " not found. Skipping V4L2 CLI tests.";
        }
        
        // Check if v4l2-ctl is available
        int ret = system("which v4l2-ctl > /dev/null 2>&1");
        if (ret != 0) {
            GTEST_SKIP() << "v4l2-ctl utility not found. Install v4l-utils package.";
        }
    }
};

/**
 * Test: V4L2 CLI - List formats with extended info
 * Command: v4l2-ctl --device /dev/video0 --list-formats-ext
 */
TEST_F(V4L2CliTest, ListFormatsExtended) {
    std::string cmd = "v4l2-ctl --device " + device_path_ + " --list-formats-ext 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    std::cout << "=== List Formats Extended ===\n" << output << "\n";
}

/**
 * Test: V4L2 CLI - List formats
 * Command: v4l2-ctl --device=/dev/video0 --list-formats
 */
TEST_F(V4L2CliTest, ListFormats) {
    std::string cmd = "v4l2-ctl --device=" + device_path_ + " --list-formats 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    std::cout << "=== List Formats ===\n" << output << "\n";
}

/**
 * Test: V4L2 CLI - Get video format
 * Command: v4l2-ctl --device /dev/video0 --get-fmt-video
 */
TEST_F(V4L2CliTest, GetVideoFormat) {
    std::string cmd = "v4l2-ctl --device " + device_path_ + " --get-fmt-video 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    // Check for expected format information
    bool has_format_info = (output.find("Width/Height") != std::string::npos ||
                           output.find("Pixel Format") != std::string::npos ||
                           output.find("Field") != std::string::npos);
    
    EXPECT_TRUE(has_format_info) << "Missing format information in output";
    
    std::cout << "=== Get Video Format ===\n" << output << "\n";
}

/**
 * Test: V4L2 CLI - List all devices
 * Command: v4l2-ctl --list-devices
 */
TEST_F(V4L2CliTest, ListAllDevices) {
    std::string cmd = "v4l2-ctl --list-devices 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    // Should list at least one device
    bool has_devices = (output.find("/dev/video") != std::string::npos ||
                       output.find("/dev/v4l") != std::string::npos);
    
    EXPECT_TRUE(has_devices) << "No V4L2 devices found in output";
    
    std::cout << "=== List All Devices ===\n" << output << "\n";
}

/**
 * Test: V4L2 CLI - Show all device information
 * Command: v4l2-ctl --device /dev/video0 --all
 */
TEST_F(V4L2CliTest, ShowAllInfo) {
    std::string cmd = "v4l2-ctl --device " + device_path_ + " --all 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    // Should contain comprehensive device info
    bool has_info = (output.find("Driver Info") != std::string::npos ||
                    output.find("Capabilities") != std::string::npos ||
                    output.find("Card") != std::string::npos);
    
    EXPECT_TRUE(has_info) << "Missing device information in output";
    
    std::cout << "=== Show All Device Info ===\n" << output << "\n";
}

// Test fixture for V4L2 subdevice tests
class V4L2SubdevTest : public ::testing::Test {
protected:
    std::string subdev_path_;
    
    void SetUp() override {
        // Use /dev/v4l-subdev1 by default
        subdev_path_ = g_device_address.empty() ? "/dev/v4l-subdev1" : g_device_address;
        
        // Check if subdev exists
        if (access(subdev_path_.c_str(), F_OK) != 0) {
            GTEST_SKIP() << "Subdev " << subdev_path_ << " not found. Skipping subdev tests.";
        }
        
        // Check if v4l2-ctl is available
        int ret = system("which v4l2-ctl > /dev/null 2>&1");
        if (ret != 0) {
            GTEST_SKIP() << "v4l2-ctl utility not found. Install v4l-utils package.";
        }
    }
};

/**
 * Test: V4L2 Subdev - List controls
 * Command: v4l2-ctl -d /dev/v4l-subdev1 -l
 */
TEST_F(V4L2SubdevTest, ListControls) {
    std::string cmd = "v4l2-ctl -d " + subdev_path_ + " -l 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    // Command should execute without critical errors
    EXPECT_TRUE(output.find("ERROR: Failed to execute") == std::string::npos) 
        << "Command execution failed";
    EXPECT_TRUE(output.find("No such file or directory") == std::string::npos) 
        << "Subdevice not accessible";
    
    // Output should either list controls or indicate none are available
    bool has_controls = (output.find("0x") != std::string::npos ||  // Control IDs
                        output.find("min=") != std::string::npos ||   // Control range
                        output.find("max=") != std::string::npos);
    bool no_controls = (output.empty() || 
                       output.find("inappropriate ioctl") != std::string::npos);
    
    EXPECT_TRUE(has_controls || no_controls) 
        << "Unexpected output format. Output: " << output;
    
    std::cout << "=== Subdev List Controls ===\n" << output << "\n";
}

/**
 * Test: V4L2 Subdev - List media bus codes
 * Command: v4l2-ctl -d /dev/v4l-subdev1 --list-subdev-mbus-codes
 */
TEST_F(V4L2SubdevTest, ListMbusCodes) {
    std::string cmd = "v4l2-ctl -d " + subdev_path_ + " --list-subdev-mbus-codes 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    std::cout << "=== Subdev Media Bus Codes ===\n" << output << "\n";
}

/**
 * Test: V4L2 Subdev - List controls with details
 * Command: v4l2-ctl -d /dev/v4l-subdev1 -L
 */
TEST_F(V4L2SubdevTest, ListControlsDetailed) {
    std::string cmd = "v4l2-ctl -d " + subdev_path_ + " -L 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    // Command should execute without critical errors
    EXPECT_TRUE(output.find("ERROR: Failed to execute") == std::string::npos) 
        << "Command execution failed";
    EXPECT_TRUE(output.find("No such file or directory") == std::string::npos) 
        << "Subdevice not accessible";
    
    // Output should either list controls with details or indicate none available
    bool has_controls = (output.find("0x") != std::string::npos ||  // Control IDs
                        output.find("range:") != std::string::npos || // Detailed range info
                        output.find("flags:") != std::string::npos);   // Control flags
    bool no_controls = (output.empty() || 
                       output.find("inappropriate ioctl") != std::string::npos);
    
    EXPECT_TRUE(has_controls || no_controls) 
        << "Unexpected output format. Output: " << output;
    
    std::cout << "=== Subdev List Controls Detailed ===\n" << output << "\n";
}

// Test fixture for media controller tests
class MediaControllerTest : public ::testing::Test {
protected:
    std::string media_path_;
    
    void SetUp() override {
        // Use /dev/media0 by default
        media_path_ = "/dev/media0";
        
        // Check if media device exists
        if (access(media_path_.c_str(), F_OK) != 0) {
            GTEST_SKIP() << "Media device " << media_path_ << " not found. Skipping media-ctl tests.";
        }
        
        // Check if media-ctl is available
        int ret = system("which media-ctl > /dev/null 2>&1");
        if (ret != 0) {
            GTEST_SKIP() << "media-ctl utility not found. Install v4l-utils package.";
        }
    }
};

/**
 * Test: Media Controller - Print topology in DOT format
 * Command: media-ctl --device /dev/media0 --print-dot
 */
TEST_F(MediaControllerTest, PrintTopologyDot) {
    std::string cmd = "media-ctl --device " + media_path_ + " --print-dot 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    // Should contain DOT format markers
    bool has_dot_format = (output.find("digraph") != std::string::npos ||
                          output.find("->") != std::string::npos ||
                          output.find("node") != std::string::npos);
    
    EXPECT_TRUE(has_dot_format) << "Missing DOT format markers in output";
    
    std::cout << "=== Media Topology (DOT Format) ===\n" << output << "\n";
}

/**
 * Test: Media Controller - Print topology
 * Command: media-ctl --device /dev/media0 -p
 */
TEST_F(MediaControllerTest, PrintTopology) {
    std::string cmd = "media-ctl --device " + media_path_ + " -p 2>&1";
    std::string output = executeCommand(cmd.c_str());
    
    EXPECT_FALSE(output.empty()) << "No output from command";
    
    // Should contain topology information
    bool has_topology = (output.find("entity") != std::string::npos ||
                        output.find("pad") != std::string::npos ||
                        output.find("link") != std::string::npos);
    
    EXPECT_TRUE(has_topology) << "Missing topology information in output";
    
    std::cout << "=== Media Topology ===\n" << output << "\n";
}

int main(int argc, char** argv) {
    TestRunner runner("driver-v4l2_cli_test");
    
    int result = runner.initialize(argc, argv);
    if (result == 0) {
        return 0; // Help was shown
    }
    
    return runner.runTests();
}
