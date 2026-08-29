cmake_minimum_required(VERSION 3.10)

project(tof-driver-tests)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GoogleTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Build test utilities library
add_library(driver_test_utils STATIC
    utilities/test_utils.cpp
)
target_include_directories(driver_test_utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(driver_test_utils
    ${GTEST_LIBRARIES}
    pthread
)

# Build V4L2 driver test
add_executable(driver-v4l2_test
    system/driver-v4l2_test.cpp
)
target_include_directories(driver-v4l2_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(driver-v4l2_test
    driver_test_utils
    ${GTEST_LIBRARIES}
    pthread
)

# Build I2C driver test
add_executable(driver-i2c_test
    system/driver-i2c_test.cpp
)
target_include_directories(driver-i2c_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(driver-i2c_test
    driver_test_utils
    ${GTEST_LIBRARIES}
    pthread
)

# Enable testing
enable_testing()

# Add tests to CTest
add_test(NAME V4L2DriverTest COMMAND driver-v4l2_test)
add_test(NAME I2CDriverTest COMMAND driver-i2c_test)

# Install targets
install(TARGETS driver-v4l2_test driver-i2c_test
    RUNTIME DESTINATION bin
)

# Install test utilities
install(FILES utilities/test_utils.h
    DESTINATION include/driver_tests
)

# Install CSV configuration
install(FILES test_csvs/driver_test_list.csv
    DESTINATION share/driver_tests
)
