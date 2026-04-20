cmake_minimum_required(VERSION 3.15)
project(KalmanFilterTest)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch Catch2 if not already available
include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)

# Source files for the Kalman filter library
set(KALMAN_SOURCES
    kalman.c
    sensor.c
)

set(KALMAN_HEADERS
    kalman.h
    sensor.h
)

# Create library
add_library(kalman_lib STATIC ${KALMAN_SOURCES} ${KALMAN_HEADERS})
target_include_directories(kalman_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Test executable for Kalman Filter
add_executable(test_kalman_filter 
    test_kalman_filter.cpp
)
target_link_libraries(test_kalman_filter PRIVATE kalman_lib Catch2::Catch2WithMain)
target_include_directories(test_kalman_filter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Enable testing
enable_testing()

# Register tests with CTest
add_test(NAME KalmanFilterTests COMMAND test_kalman_filter)