cmake_minimum_required(VERSION 3.15)
project(Quadrature)

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

# Source files for the library
set(QUADRATURE_SOURCES
    quadrature_output.c
    quadrature_decoder.c
)

set(QUADRATURE_HEADERS
    quadrature_output.h
    quadrature_decoder.h
    quadrature_common.h
)

# Create library
add_library(quadrature_lib STATIC ${QUADRATURE_SOURCES} ${QUADRATURE_HEADERS})
target_include_directories(quadrature_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Test executable for QuadratureOutput
add_executable(test_quadrature_output 
    test_quadrature_output.cpp
)
target_link_libraries(test_quadrature_output PRIVATE quadrature_lib Catch2::Catch2WithMain)
target_include_directories(test_quadrature_output PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Test executable for QuadratureDecoder
add_executable(test_quadrature_decoder 
    test_quadrature_decoder.cpp
)
target_link_libraries(test_quadrature_decoder PRIVATE quadrature_lib Catch2::Catch2WithMain)
target_include_directories(test_quadrature_decoder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Enable testing
enable_testing()

# Register tests with CTest
add_test(NAME QuadratureOutputTests COMMAND test_quadrature_output)
add_test(NAME QuadratureDecoderTests COMMAND test_quadrature_decoder)

