add_library(kite_core src/angles.cpp src/assignment.cpp src/baseline_tracker.cpp src/kalman.cpp
            src/kalman_tracker.cpp src/log.cpp src/pid.cpp src/tracker.cpp)
target_include_directories(kite_core PUBLIC include)
target_compile_features(kite_core PUBLIC cxx_std_17)
target_compile_options(kite_core PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)
kite_enable_warnings(kite_core)

if(KITE_BUILD_TESTS)
  add_executable(kite_core_tests tests/angles_test.cpp tests/assignment_bruteforce_test.cpp
                 tests/assignment_test.cpp tests/baseline_tracker_test.cpp tests/kalman_test.cpp
                 tests/kalman_tracker_test.cpp tests/log_test.cpp tests/matrix_test.cpp
                 tests/pid_test.cpp)
  # kite_sim supplies generator data to the Kalman NEES test; kite_core never links it.
  target_link_libraries(kite_core_tests PRIVATE kite_core kite_sim GTest::gmock_main)
  target_compile_options(kite_core_tests
                         PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)
  kite_enable_warnings(kite_core_tests)
  gtest_discover_tests(kite_core_tests)
endif()
