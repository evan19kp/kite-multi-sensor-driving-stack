add_executable(kite_gen kite_gen.cpp)
target_link_libraries(kite_gen PRIVATE kite_sim kite_core)
kite_enable_warnings(kite_gen)

add_executable(kite_score kite_score.cpp)
target_link_libraries(kite_score PRIVATE kite_eval kite_core)
kite_enable_warnings(kite_score)

add_executable(kite_baseline kite_baseline.cpp)
target_link_libraries(kite_baseline PRIVATE kite_sim kite_eval kite_core)
kite_enable_warnings(kite_baseline)

add_executable(kite_association_study kite_association_study.cpp)
target_link_libraries(kite_association_study PRIVATE kite_sim kite_eval kite_core)
target_compile_options(kite_association_study
                       PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)
kite_enable_warnings(kite_association_study)

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
  kite_exit_test(kite_baseline_tracker_bad_value kite_baseline 2 "--tracker;bogus")
  kite_exit_test(kite_baseline_tracker_missing_value kite_baseline 2 "--seeds;1;--tracker")
  kite_exit_test(kite_baseline_tracker_duplicate kite_baseline 2
    "--tracker;kalman;--seeds;1;--tracker;baseline")
  kite_exit_test(kite_baseline_range_reversed kite_baseline 2 "--seeds;5-3")
  kite_exit_test(kite_baseline_range_zero_start kite_baseline 2 "--seeds;0-3")
  kite_exit_test(kite_baseline_range_missing_end kite_baseline 2 "--seeds;1-")
  kite_exit_test(kite_baseline_range_missing_start kite_baseline 2 "--seeds;-3")
  kite_exit_test(kite_baseline_range_three_parts kite_baseline 2 "--seeds;1-2-3")
  kite_exit_test(kite_baseline_range_non_number kite_baseline 2 "--seeds;1-x")

  add_test(NAME kite_baseline_tracker_after_seeds
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--seeds;101-101;--tracker;kalman"
            "-DEXPECT_MATCH=[|] 101 [|] 1494 [|] 11 [|] 0 [|] 0[.]412 [|] 0 [|] 301 [|]"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/kalman_101.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)

  add_test(NAME kite_baseline_range_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--tracker;baseline;--seeds;19-20"
            "-DEXPECT_MATCH=[|] 19 [|] 1402 [|].*[|] 20 [|] 1424 [|].*[|] mean [|] 1413[.]000 [|] 92[.]000 [|]"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/baseline_19_20.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)

  add_test(NAME kite_baseline_kalman_tuning
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--tracker;kalman;--seeds;101-120"
            "-DEXPECT_MATCH=[|] mean [|] 1494[.]100 [|] 10[.]900 [|] 0[.]050 [|] 0[.]414 [|] 0[.]000 [|] 301[.]000 [|]"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/kalman_tuning.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)

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

  add_test(NAME kite_baseline_kalman_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--tracker;kalman"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/kalman.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_baseline_kalman_output PROPERTIES FIXTURES_SETUP kite_kalman_table)

  add_test(NAME kite_baseline_kalman_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/kalman.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/BASELINE.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_baseline_kalman_doc_matches PROPERTIES FIXTURES_REQUIRED kite_kalman_table)

  kite_exit_test(kite_baseline_association_without_tracker kite_baseline 2 "--association;joint")
  kite_exit_test(kite_baseline_association_with_baseline_tracker kite_baseline 2
    "--tracker;baseline;--association;joint")
  kite_exit_test(kite_baseline_association_bad_value kite_baseline 2
    "--tracker;kalman;--association;bogus")
  kite_exit_test(kite_baseline_association_missing_value kite_baseline 2
    "--tracker;kalman;--association")
  kite_exit_test(kite_baseline_association_duplicate kite_baseline 2
    "--tracker;kalman;--association;joint;--association;joint")
  kite_exit_test(kite_baseline_association_before_tracker kite_baseline 0
    "--association;joint_likelihood;--seeds;101-101;--tracker;kalman")

  foreach(mode two_stage joint joint_likelihood)
    set(expect_match "")
    if(mode STREQUAL "two_stage")
      set(expect_match
        "-DEXPECT_MATCH=[|] mean [|] 1494[.]100 [|] 10[.]900 [|] 0[.]050 [|] 0[.]414 [|] 0[.]000 [|] 301[.]000 [|]")
    endif()
    add_test(NAME kite_baseline_association_${mode}_output
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline>
              "-DARGS=--tracker;kalman;--association;${mode};--seeds;101-120"
              ${expect_match}
              -DOUT=${CMAKE_CURRENT_BINARY_DIR}/kalman_${mode}_tuning.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
    set_tests_properties(kite_baseline_association_${mode}_output
                         PROPERTIES FIXTURES_SETUP kite_baseline_association_${mode}_table)
    add_test(NAME kite_baseline_association_${mode}_doc_matches
      COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/kalman_${mode}_tuning.md
              -DHEADING=${mode}:
              -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
    set_tests_properties(kite_baseline_association_${mode}_doc_matches
                         PROPERTIES FIXTURES_REQUIRED kite_baseline_association_${mode}_table)

    add_test(NAME kite_association_study_${mode}_summary_output
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_association_study>
              "-DARGS=--association;${mode};--summary"
              "-DEXPECT_MATCH=^Association ${mode}, seeds 101-120:"
              -DOUT=${CMAKE_CURRENT_BINARY_DIR}/association_study_${mode}_summary.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
    set_tests_properties(kite_association_study_${mode}_summary_output
                         PROPERTIES FIXTURES_SETUP kite_association_study_${mode}_summary)
    add_test(NAME kite_association_study_${mode}_summary_doc_matches
      COMMAND ${CMAKE_COMMAND}
              -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/association_study_${mode}_summary.md
              -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
    set_tests_properties(kite_association_study_${mode}_summary_doc_matches
                         PROPERTIES FIXTURES_REQUIRED kite_association_study_${mode}_summary)
  endforeach()

  kite_exit_test(kite_association_study_help kite_association_study 0 "--help")
  kite_exit_test(kite_association_study_unknown_flag kite_association_study 2 "--bogus")
  kite_exit_test(kite_association_study_association_bad_value kite_association_study 2
    "--association;bogus")
  kite_exit_test(kite_association_study_association_missing_value kite_association_study 2
    "--summary;--association")
  kite_exit_test(kite_association_study_association_duplicate kite_association_study 2
    "--association;joint;--association;two_stage")
  kite_exit_test(kite_association_study_summary_duplicate kite_association_study 2
    "--summary;--summary")
  kite_exit_test(kite_association_study_help_with_another_flag kite_association_study 2
    "--help;--summary")

  add_test(NAME kite_association_study_joint_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_association_study> "-DARGS=--association;joint"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/association_study_joint.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_association_study_joint_output
                       PROPERTIES FIXTURES_SETUP kite_association_study_joint_tables)
  add_test(NAME kite_association_study_joint_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/association_study_joint.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_association_study_joint_doc_matches
                       PROPERTIES FIXTURES_REQUIRED kite_association_study_joint_tables)

  add_test(NAME kite_association_study_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_association_study>
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/association_study.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_association_study_output
                       PROPERTIES FIXTURES_SETUP kite_association_study_tables)

  add_test(NAME kite_association_study_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/association_study.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_association_study_doc_matches
                       PROPERTIES FIXTURES_REQUIRED kite_association_study_tables)
endif()
