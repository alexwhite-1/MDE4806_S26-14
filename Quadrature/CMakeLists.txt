cmake_minimum_required(VERSION 3.15)
project(QuadratureOutput)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the main library
add_library(quadrature_output SHARED
    quadrature_output.cpp
    quadrature_output.hpp
)

target_include_directories(quadrature_output PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Catch2 test setup
include(FetchContent)
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)

# Add test executable
add_executable(test_quadrature_output
    test_quadrature_output.cpp
)

target_link_libraries(test_quadrature_output PRIVATE Catch2::Catch2WithMain quadrature_output)
target_include_directories(test_quadrature_output PRIVATE ${Catch2_SOURCE_DIR}/src)

# Register tests with CTest
include(CTest)
include(Catch)
catch_discover_tests(test_quadrature_output)
