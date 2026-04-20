# CMake generated Testfile for 
# Source directory: C:/VirginiaTech/ECE4806/MDE4806_S26-14/TEST_Kalman
# Build directory: C:/VirginiaTech/ECE4806/MDE4806_S26-14/TEST_Kalman/out/build/x64-Debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(KalmanFilterTests "C:/VirginiaTech/ECE4806/MDE4806_S26-14/TEST_Kalman/out/build/x64-Debug/test_kalman_filter.exe")
set_tests_properties(KalmanFilterTests PROPERTIES  _BACKTRACE_TRIPLES "C:/VirginiaTech/ECE4806/MDE4806_S26-14/TEST_Kalman/CMakeLists.txt;44;add_test;C:/VirginiaTech/ECE4806/MDE4806_S26-14/TEST_Kalman/CMakeLists.txt;0;")
subdirs("_deps/catch2-build")
