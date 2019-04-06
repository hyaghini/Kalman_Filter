cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(KalmanFilter)

set(EIGEN_INC_DIR /usr/include/eigen3/)
include_directories(${EIGEN_INC_DIR})

################################
# GTest
################################
ADD_SUBDIRECTORY (../../googletest-release-1.8.1 ../../googletest-release-1.8.1/build EXCLUDE_FROM_ALL)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Unit Tests
################################
# Add test cpp file
add_executable( TestKalmanFilter KalmanGtest.cc ../kalman.cc)
# Link test executable against gtest & gtest_main
target_link_libraries(TestKalmanFilter gtest gtest_main)
add_test( TestKalmanFilter runUnitTests )
