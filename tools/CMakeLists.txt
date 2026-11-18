add_executable(kite_gen kite_gen.cpp)
target_link_libraries(kite_gen PRIVATE kite_sim kite_core)
kite_enable_warnings(kite_gen)

add_executable(kite_score kite_score.cpp)
target_link_libraries(kite_score PRIVATE kite_eval kite_core)
kite_enable_warnings(kite_score)

add_executable(kite_baseline kite_baseline.cpp)
target_link_libraries(kite_baseline PRIVATE kite_sim kite_eval kite_core)
kite_enable_warnings(kite_baseline)

if(KITE_BUILD_TESTS)
  function(kite_exit_test name tool expect args)
    add_test(NAME ${name}
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:${tool}> "-DARGS=${args}" -DEXPECT=${expect}
              ${ARGN} -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_exit.cmake)
  endfunction()

  kite_exit_test(kite_gen_no_args kite_gen 2 "")
  kite_exit_test(kite_gen_bad_value kite_gen 2
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1.5;--out;unused.log")
  kite_exit_test(kite_gen_non_number kite_gen 2
    "--seed;abc;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;unused.log")
  kite_exit_test(kite_gen_unknown_flag kite_gen 2 "--bogus;1")
  kite_exit_test(kite_gen_duplicate_flag kite_gen 2
    "--seed;1;--seed;2;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;unused.log")
  kite_exit_test(kite_gen_missing_value kite_gen 2
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out")
  kite_exit_test(kite_gen_write_failure kite_gen 1
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;${CMAKE_CURRENT_BINARY_DIR}/no_such_dir/x.log")

  kite_exit_test(kite_gen_golden kite_gen 0
    "--seed;42;--vehicles;3;--duration;2;--dt;0.1;--noise;0.5;--pd;0.9;--out;${CMAKE_CURRENT_BINARY_DIR}/seed42.log")
  set_tests_properties(kite_gen_golden PROPERTIES FIXTURES_SETUP kite_gen_golden_file)

  add_test(NAME kite_gen_golden_matches
    COMMAND ${CMAKE_COMMAND} -E compare_files --ignore-eol
            ${CMAKE_CURRENT_BINARY_DIR}/seed42.log ${PROJECT_SOURCE_DIR}/sim/tests/data/seed42.log)
  set_tests_properties(kite_gen_golden_matches PROPERTIES FIXTURES_REQUIRED kite_gen_golden_file)

  kite_exit_test(kite_score_no_args kite_score 2 "")
  kite_exit_test(kite_score_unknown_flag kite_score 2 "--log;x.log;--bogus;1")
  kite_exit_test(kite_score_duplicate_flag kite_score 2 "--log;x.log;--log;y.log")
  kite_exit_test(kite_score_missing_value kite_score 2 "--log")
  kite_exit_test(kite_score_bad_gate kite_score 2 "--log;x.log;--gate;-1")
  kite_exit_test(kite_score_missing_log kite_score 1
    "--log;${CMAKE_CURRENT_BINARY_DIR}/no_such_dir/x.log" "-DERROR_TEXT=cannot read")
  kite_exit_test(kite_score_not_a_log kite_score 1
    "--log;${PROJECT_SOURCE_DIR}/eval/README.md" "-DERROR_TEXT=cannot read")

  add_test(NAME kite_score_small_log
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_score>
            -DLOG=${PROJECT_SOURCE_DIR}/eval/tests/data/small.log
            -DCOPY=${CMAKE_CURRENT_BINARY_DIR}/small.log
            "-DEXPECT_STDOUT=matched=3 missed=1 false=1 rmse_m=1.291 id_switches=1 steps=2"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/score_log.cmake)

  add_test(NAME kite_score_no_match_log
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_score>
            -DLOG=${PROJECT_SOURCE_DIR}/eval/tests/data/no_match.log
            -DCOPY=${CMAKE_CURRENT_BINARY_DIR}/no_match.log
            "-DEXPECT_STDOUT=matched=0 missed=2 false=2 rmse_m=n/a id_switches=0 steps=2"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/score_log.cmake)

  kite_exit_test(kite_baseline_help kite_baseline 0 "--help")
  kite_exit_test(kite_baseline_zero_seeds kite_baseline 2 "--seeds;0")
  kite_exit_test(kite_baseline_non_number kite_baseline 2 "--seeds;abc")
  kite_exit_test(kite_baseline_unknown_flag kite_baseline 2 "--bogus;1")
  kite_exit_test(kite_baseline_duplicate_flag kite_baseline 2 "--seeds;1;--seeds;2")
  kite_exit_test(kite_baseline_missing_value kite_baseline 2 "--seeds")

  add_test(NAME kite_baseline_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline>
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/baseline.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_baseline_output PROPERTIES FIXTURES_SETUP kite_baseline_table)

  add_test(NAME kite_baseline_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/baseline.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/BASELINE.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_baseline_doc_matches PROPERTIES FIXTURES_REQUIRED kite_baseline_table)
endif()
