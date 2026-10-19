/**
 * @file driver-v4l2_test.cpp
 * @brief V4L2 driver validation tests for ADSD3500 ToF sensor
 * 
 * Tests V4L2 video device interface including:
 * - Device enumeration and capability query
 * - Format enumeration and configuration
 * - Buffer management (REQBUFS, MMAP)
 * - Frame size enumeration
 * - Control enumeration and verification
 * - Streaming operations
 */

#include <gtest/gtest.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <memory>

#include "../utilities/test_utils.h"

using namespace test_utils;

class V4L2DriverTest : public ::testing::Test {
protected:
    int fd_;
    std::string device_path_;
    
    void SetUp() override {
        // Default to /dev/video0 if not specified
        device_path_ = g_device_address.empty() ? "/dev/video0" : g_device_address;
        
        // Open video device
        fd_ = open(device_path_.c_str(), O_RDWR);
        if (fd_ < 0) {
            GTEST_SKIP() << "Cannot open device " << device_path_ 
                        << ". Skipping V4L2 tests (error: " << strerror(errno) << ")";
        }
    }
    
    void TearDown() override {
        if (fd_ >= 0) {
            close(fd_);
        }
    }
};

/**
 * Test: Device enumeration and basic open/close operations
 * Verifies that the V4L2 video device can be opened and closed properly
 */
TEST_F(V4L2DriverTest, DeviceEnumeration) {
    ASSERT_GE(fd_, 0) << "Failed to open device " << device_path_;
    
    // Device is already open from SetUp()
    SUCCEED() << "Device " << device_path_ << " opened successfully";
}

/**
 * Test: Capability query
 * Verifies that the device supports video capture capabilities
 */
TEST_F(V4L2DriverTest, CapabilityQuery) {
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    
    int ret = ioctl(fd_, VIDIOC_QUERYCAP, &cap);
    ASSERT_EQ(ret, 0) << "VIDIOC_QUERYCAP failed: " << strerror(errno);
    
    // Verify driver and device info
    EXPECT_GT(strlen((char*)cap.driver), 0) << "Driver name is empty";
    EXPECT_GT(strlen((char*)cap.card), 0) << "Card name is empty";
    
    // Check for video capture capability
    EXPECT_TRUE(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) 
        << "Device does not support video capture";
    
    std::cout << "Driver: " << cap.driver << "\n";
    std::cout << "Card: " << cap.card << "\n";
    std::cout << "Bus Info: " << cap.bus_info << "\n";
    std::cout << "Version: " << ((cap.version >> 16) & 0xFF) << "."
              << ((cap.version >> 8) & 0xFF) << "."
              << (cap.version & 0xFF) << "\n";
}

/**
 * Test: Format enumeration
 * Verifies that the device can enumerate supported pixel formats
 */
TEST_F(V4L2DriverTest, FormatEnumeration) {
    struct v4l2_fmtdesc fmtdesc;
    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    int format_count = 0;
    while (ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        format_count++;
        std::cout << "Format #" << format_count << ": " 
                  << fmtdesc.description << " (fourcc: "
                  << (char)(fmtdesc.pixelformat & 0xFF)
                  << (char)((fmtdesc.pixelformat >> 8) & 0xFF)
                  << (char)((fmtdesc.pixelformat >> 16) & 0xFF)
                  << (char)((fmtdesc.pixelformat >> 24) & 0xFF)
                  << ")\n";
        fmtdesc.index++;
    }
    
    EXPECT_GT(format_count, 0) << "No video formats enumerated";
}

/**
 * Test: Get and set video format
 * Verifies format configuration operations
 */
TEST_F(V4L2DriverTest, FormatGetSet) {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    // Get current format
    int ret = ioctl(fd_, VIDIOC_G_FMT, &fmt);
    ASSERT_EQ(ret, 0) << "VIDIOC_G_FMT failed: " << strerror(errno);
    
    std::cout << "Current format:\n";
    std::cout << "  Width: " << fmt.fmt.pix.width << "\n";
    std::cout << "  Height: " << fmt.fmt.pix.height << "\n";
    std::cout << "  Bytes per line: " << fmt.fmt.pix.bytesperline << "\n";
    std::cout << "  Size image: " << fmt.fmt.pix.sizeimage << "\n";
    
    EXPECT_GT(fmt.fmt.pix.width, 0) << "Invalid width";
    EXPECT_GT(fmt.fmt.pix.height, 0) << "Invalid height";
    EXPECT_GT(fmt.fmt.pix.sizeimage, 0) << "Invalid image size";
    
    // Try to set the same format (should succeed)
    ret = ioctl(fd_, VIDIOC_S_FMT, &fmt);
    EXPECT_EQ(ret, 0) << "VIDIOC_S_FMT failed: " << strerror(errno);
}

/**
 * Test: Buffer allocation
 * Verifies REQBUFS and MMAP operations for buffer management
 */
TEST_F(V4L2DriverTest, BufferAllocation) {
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    int ret = ioctl(fd_, VIDIOC_REQBUFS, &req);
    ASSERT_EQ(ret, 0) << "VIDIOC_REQBUFS failed: " << strerror(errno);
    EXPECT_GE(req.count, 1) << "No buffers allocated";
    
    std::cout << "Allocated " << req.count << " buffers\n";
    
    // Query and map first buffer
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    
    ret = ioctl(fd_, VIDIOC_QUERYBUF, &buf);
    ASSERT_EQ(ret, 0) << "VIDIOC_QUERYBUF failed: " << strerror(errno);
    
    void* buffer = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    ASSERT_NE(buffer, MAP_FAILED) << "mmap failed: " << strerror(errno);
    
    std::cout << "Buffer 0:\n";
    std::cout << "  Length: " << buf.length << " bytes\n";
    std::cout << "  Offset: " << buf.m.offset << "\n";
    
    // Cleanup
    munmap(buffer, buf.length);
}

/**
 * Test: Frame size enumeration
 * Verifies discrete or stepwise frame size support
 */
TEST_F(V4L2DriverTest, FrameSizeEnumeration) {
    // Get first supported format
    struct v4l2_fmtdesc fmtdesc;
    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmtdesc.index = 0;
    
    int ret = ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc);
    if (ret != 0) {
        GTEST_SKIP() << "No formats available for frame size enumeration";
    }
    
    // Enumerate frame sizes
    struct v4l2_frmsizeenum frmsize;
    memset(&frmsize, 0, sizeof(frmsize));
    frmsize.pixel_format = fmtdesc.pixelformat;
    frmsize.index = 0;
    
    int size_count = 0;
    while (ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
        size_count++;
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            std::cout << "Frame size #" << size_count << ": "
                      << frmsize.discrete.width << "x" << frmsize.discrete.height << "\n";
        } else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
            std::cout << "Frame size stepwise:\n";
            std::cout << "  Min: " << frmsize.stepwise.min_width << "x" << frmsize.stepwise.min_height << "\n";
            std::cout << "  Max: " << frmsize.stepwise.max_width << "x" << frmsize.stepwise.max_height << "\n";
            std::cout << "  Step: " << frmsize.stepwise.step_width << "x" << frmsize.stepwise.step_height << "\n";
        }
        frmsize.index++;
    }
    
    EXPECT_GT(size_count, 0) << "No frame sizes enumerated";
}

/**
 * Test: Control enumeration
 * Verifies that device controls can be enumerated and queried
 */
TEST_F(V4L2DriverTest, ControlEnumeration) {
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    
    int control_count = 0;
    while (ioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
        if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
            control_count++;
            std::cout << "Control: " << queryctrl.name 
                      << " (ID: 0x" << std::hex << queryctrl.id << std::dec << ")\n";
        }
        queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    
    std::cout << "Total controls enumerated: " << control_count << "\n";
    
    // Note: Some devices may not have any controls, so we don't assert > 0
    SUCCEED();
}

/**
 * Test: Streaming start/stop
 * Verifies that streaming can be started and stopped without errors
 */
TEST_F(V4L2DriverTest, StreamingControl) {
    // Allocate buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    int ret = ioctl(fd_, VIDIOC_REQBUFS, &req);
    ASSERT_EQ(ret, 0) << "VIDIOC_REQBUFS failed: " << strerror(errno);
    
    // Queue buffers
    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        ret = ioctl(fd_, VIDIOC_QBUF, &buf);
        ASSERT_EQ(ret, 0) << "VIDIOC_QBUF failed for buffer " << i << ": " << strerror(errno);
    }
    
    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd_, VIDIOC_STREAMON, &type);
    ASSERT_EQ(ret, 0) << "VIDIOC_STREAMON failed: " << strerror(errno);
    
    std::cout << "Streaming started successfully\n";
    
    // Stop streaming
    ret = ioctl(fd_, VIDIOC_STREAMOFF, &type);
    ASSERT_EQ(ret, 0) << "VIDIOC_STREAMOFF failed: " << strerror(errno);
    
    std::cout << "Streaming stopped successfully\n";
}

// Main function
int main(int argc, char** argv) {
    TestRunner runner("driver-v4l2_test");
    
    int result = runner.initialize(argc, argv);
    if (result == 0) {
        return 0; // Help was shown
    }
    
    return runner.runTests();
}
