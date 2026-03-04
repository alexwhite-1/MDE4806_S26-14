# CMake generated Testfile for 
# Source directory: C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature
# Build directory: C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(QuadratureOutputTests "C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature/build/test_quadrature_output.exe")
set_tests_properties(QuadratureOutputTests PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature/CMakeLists.txt;49;add_test;C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature/CMakeLists.txt;0;")
add_test(QuadratureDecoderTests "C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature/build/test_quadrature_decoder.exe")
set_tests_properties(QuadratureDecoderTests PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature/CMakeLists.txt;50;add_test;C:/Users/alexx/SENIOR_DESIGN/MDE4806_S26-14/Quadrature/CMakeLists.txt;0;")
subdirs("_deps/catch2-build")
