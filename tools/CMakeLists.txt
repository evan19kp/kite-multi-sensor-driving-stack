add_executable(kite_gen kite_gen.cpp)
target_link_libraries(kite_gen PRIVATE kite_sim kite_core)
kite_enable_warnings(kite_gen)

if(KITE_BUILD_TESTS)
  function(kite_gen_exit_test name expect args)
    add_test(NAME ${name}
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_gen> "-DARGS=${args}" -DEXPECT=${expect}
              -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_exit.cmake)
  endfunction()

  kite_gen_exit_test(kite_gen_no_args 2 "")
  kite_gen_exit_test(kite_gen_bad_value 2
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1.5;--out;unused.log")
  kite_gen_exit_test(kite_gen_non_number 2
    "--seed;abc;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;unused.log")
  kite_gen_exit_test(kite_gen_unknown_flag 2 "--bogus;1")
  kite_gen_exit_test(kite_gen_duplicate_flag 2
    "--seed;1;--seed;2;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;unused.log")
  kite_gen_exit_test(kite_gen_missing_value 2
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out")
  kite_gen_exit_test(kite_gen_write_failure 1
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;${CMAKE_CURRENT_BINARY_DIR}/no_such_dir/x.log")

  kite_gen_exit_test(kite_gen_golden 0
    "--seed;42;--vehicles;3;--duration;2;--dt;0.1;--noise;0.5;--pd;0.9;--out;${CMAKE_CURRENT_BINARY_DIR}/seed42.log")
  set_tests_properties(kite_gen_golden PROPERTIES FIXTURES_SETUP kite_gen_golden_file)

  add_test(NAME kite_gen_golden_matches
    COMMAND ${CMAKE_COMMAND} -E compare_files --ignore-eol
            ${CMAKE_CURRENT_BINARY_DIR}/seed42.log ${PROJECT_SOURCE_DIR}/sim/tests/data/seed42.log)
  set_tests_properties(kite_gen_golden_matches PROPERTIES FIXTURES_REQUIRED kite_gen_golden_file)
endif()
