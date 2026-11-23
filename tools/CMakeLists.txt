add_executable(kite_gen kite_gen.cpp)
target_link_libraries(kite_gen PRIVATE kite_sim kite_core)
kite_enable_warnings(kite_gen)

add_executable(kite_score kite_score.cpp)
target_link_libraries(kite_score PRIVATE kite_eval kite_core)
kite_enable_warnings(kite_score)

add_executable(kite_baseline kite_baseline.cpp)
target_link_libraries(kite_baseline PRIVATE kite_sim kite_eval kite_core)
kite_enable_warnings(kite_baseline)

add_executable(kite_track kite_track.cpp)
target_link_libraries(kite_track PRIVATE kite_sim kite_core)
kite_enable_warnings(kite_track)

add_executable(kite_association_study kite_association_study.cpp)
target_link_libraries(kite_association_study PRIVATE kite_sim kite_eval kite_core)
target_compile_options(kite_association_study
                       PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)
kite_enable_warnings(kite_association_study)

add_executable(kite_drop_study kite_drop_study.cpp)
target_link_libraries(kite_drop_study PRIVATE kite_sim kite_eval kite_core)
target_compile_options(kite_drop_study
                       PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)
kite_enable_warnings(kite_drop_study)

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

  kite_exit_test(kite_gen_turning_bad_value kite_gen 2
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;unused.log;--turning;yes")
  kite_exit_test(kite_gen_speed_changes_bad_value kite_gen 2
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;unused.log;--speed-changes;1")
  kite_exit_test(kite_gen_clutter_negative kite_gen 2
    "--seed;1;--vehicles;1;--duration;1;--dt;0.1;--noise;0;--pd;1;--out;unused.log;--clutter;-1")
  kite_exit_test(kite_gen_turning_coarse_dt kite_gen 2
    "--seed;1;--vehicles;1;--duration;4;--dt;2;--noise;0;--pd;1;--out;unused.log;--turning;on")

  kite_exit_test(kite_gen_hard_golden kite_gen 0
    "--seed;7;--vehicles;3;--duration;10;--dt;0.1;--noise;1;--pd;0.95;--turning;on;--speed-changes;on;--clutter;2;--out;${CMAKE_CURRENT_BINARY_DIR}/hard_seed7.log")
  set_tests_properties(kite_gen_hard_golden PROPERTIES FIXTURES_SETUP kite_gen_hard_golden_file)

  add_test(NAME kite_gen_hard_golden_matches
    COMMAND ${CMAKE_COMMAND} -E compare_files --ignore-eol
            ${CMAKE_CURRENT_BINARY_DIR}/hard_seed7.log ${PROJECT_SOURCE_DIR}/sim/tests/data/hard_seed7.log)
  set_tests_properties(kite_gen_hard_golden_matches
                       PROPERTIES FIXTURES_REQUIRED kite_gen_hard_golden_file)

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

  add_executable(kite_tools_tests tests/track_log_test.cpp tests/study_test.cpp)
  target_link_libraries(kite_tools_tests PRIVATE kite_sim kite_eval kite_core GTest::gmock_main)
  target_include_directories(kite_tools_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(kite_tools_tests
                         PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)
  kite_enable_warnings(kite_tools_tests)
  gtest_discover_tests(kite_tools_tests)

  kite_exit_test(kite_track_help kite_track 0 "--help")
  kite_exit_test(kite_track_no_args kite_track 2 "")
  kite_exit_test(kite_track_unknown_flag kite_track 2 "--bogus;1")
  kite_exit_test(kite_track_duplicate_flag kite_track 2 "--in;a.log;--in;b.log;--out;c.log")
  kite_exit_test(kite_track_missing_value kite_track 2 "--in;a.log;--out")
  kite_exit_test(kite_track_missing_in kite_track 2 "--out;c.log")
  kite_exit_test(kite_track_missing_out kite_track 2 "--in;a.log")
  kite_exit_test(kite_track_tracker_bad_value kite_track 2
    "--in;a.log;--out;c.log;--tracker;bogus")
  kite_exit_test(kite_track_help_with_another_flag kite_track 2 "--help;--in")
  kite_exit_test(kite_track_missing_input kite_track 1
    "--in;${CMAKE_CURRENT_BINARY_DIR}/no_such_dir/x.log;--out;${CMAKE_CURRENT_BINARY_DIR}/unused.log"
    "-DERROR_TEXT=cannot read")
  kite_exit_test(kite_track_input_not_a_log kite_track 1
    "--in;${PROJECT_SOURCE_DIR}/eval/README.md;--out;${CMAKE_CURRENT_BINARY_DIR}/unused.log"
    "-DERROR_TEXT=cannot read")
  kite_exit_test(kite_track_write_failure kite_track 1
    "--in;${PROJECT_SOURCE_DIR}/sim/tests/data/seed42.log;--out;${CMAKE_CURRENT_BINARY_DIR}/no_such_dir/x.log")

  # The golden log (3 vehicles, 2 s) is not kite_baseline's scenario, so these scores are pinned
  # from kite_track and kite_score. The RunTracker tests check the kite_baseline numbers.
  set(kite_track_scores
    "baseline|matched=57 missed=6 false=0 rmse_m=0.738 id_switches=4 steps=21"
    "kalman|matched=56 missed=7 false=0 rmse_m=0.375 id_switches=0 steps=21")
  foreach(entry IN LISTS kite_track_scores)
    string(REPLACE "|" ";" parts "${entry}")
    list(GET parts 0 tracker)
    list(GET parts 1 expect)
    add_test(NAME kite_track_${tracker}_golden
      COMMAND ${CMAKE_COMMAND} -DTRACK=$<TARGET_FILE:kite_track> -DSCORE=$<TARGET_FILE:kite_score>
              -DIN=${PROJECT_SOURCE_DIR}/sim/tests/data/seed42.log -DTRACKER=${tracker}
              -DGOLDEN=${PROJECT_SOURCE_DIR}/sim/tests/data/seed42_${tracker}_tracks.log
              -DWORK=${CMAKE_CURRENT_BINARY_DIR} -DNAME=seed42_${tracker}
              "-DEXPECT_STDOUT=${expect}"
              -P ${CMAKE_CURRENT_SOURCE_DIR}/track_log.cmake)
  endforeach()

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

  kite_exit_test(kite_baseline_scenario_bad_value kite_baseline 2 "--scenario;bogus")
  kite_exit_test(kite_baseline_scenario_missing_value kite_baseline 2 "--seeds;1;--scenario")
  kite_exit_test(kite_baseline_scenario_duplicate kite_baseline 2
    "--scenario;hard;--seeds;1;--scenario;straight")
  kite_exit_test(kite_baseline_grid_on_reporting_seeds kite_baseline 2
    "--scenario;hard;--grid;1-20")
  kite_exit_test(kite_baseline_grid_single_number kite_baseline 2 "--grid;120")
  kite_exit_test(kite_baseline_grid_straddling_reporting_seeds kite_baseline 2
    "--grid;20-101")
  kite_exit_test(kite_baseline_grid_with_seeds kite_baseline 2 "--grid;101-102;--seeds;101-102")
  kite_exit_test(kite_baseline_grid_with_tracker kite_baseline 2
    "--tracker;kalman;--grid;101-102")
  kite_exit_test(kite_baseline_grid_with_association_only kite_baseline 2
    "--grid;101-102;--association;joint")

  # --scenario straight is the default, so its output is the table the doc-matches tests pin.
  add_test(NAME kite_baseline_scenario_straight_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--scenario;straight"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/baseline_straight.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_baseline_scenario_straight_output
                       PROPERTIES FIXTURES_SETUP kite_baseline_straight_table)
  add_test(NAME kite_baseline_scenario_straight_matches_default
    COMMAND ${CMAKE_COMMAND} -E compare_files
            ${CMAKE_CURRENT_BINARY_DIR}/baseline.md ${CMAKE_CURRENT_BINARY_DIR}/baseline_straight.md)
  set_tests_properties(kite_baseline_scenario_straight_matches_default
                       PROPERTIES FIXTURES_REQUIRED "kite_baseline_table;kite_baseline_straight_table")

  add_test(NAME kite_baseline_scenario_straight_kalman_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline>
            "-DARGS=--scenario;straight;--tracker;kalman"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/kalman_straight.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_baseline_scenario_straight_kalman_output
                       PROPERTIES FIXTURES_SETUP kite_kalman_straight_table)
  add_test(NAME kite_baseline_scenario_straight_kalman_matches_default
    COMMAND ${CMAKE_COMMAND} -E compare_files
            ${CMAKE_CURRENT_BINARY_DIR}/kalman.md ${CMAKE_CURRENT_BINARY_DIR}/kalman_straight.md)
  set_tests_properties(kite_baseline_scenario_straight_kalman_matches_default
                       PROPERTIES FIXTURES_REQUIRED "kite_kalman_table;kite_kalman_straight_table")

  add_test(NAME kite_baseline_hard_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--scenario;hard"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/baseline_hard.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_baseline_hard_output PROPERTIES FIXTURES_SETUP kite_baseline_hard_table)
  add_test(NAME kite_baseline_hard_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/baseline_hard.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/BASELINE.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_baseline_hard_doc_matches
                       PROPERTIES FIXTURES_REQUIRED kite_baseline_hard_table)

  add_test(NAME kite_baseline_hard_kalman_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline>
            "-DARGS=--scenario;hard;--tracker;kalman"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/kalman_hard.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_baseline_hard_kalman_output
                       PROPERTIES FIXTURES_SETUP kite_kalman_hard_table)
  add_test(NAME kite_baseline_hard_kalman_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/kalman_hard.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/BASELINE.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_baseline_hard_kalman_doc_matches
                       PROPERTIES FIXTURES_REQUIRED kite_kalman_hard_table)

  add_test(NAME kite_baseline_hard_grid_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--scenario;hard;--grid;101-120"
            "-DEXPECT_MATCH=[|] 10[.]000 [|] 5[.]000 [|] 1494[.]100 [|] 10[.]900 [|] 1[.]100 [|] 0[.]806 [|] 0[.]150 [|] 301[.]000 [|] 12[.]150 [|].*selected: q 10[.]000, velocity_sigma 5[.]000"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/grid_hard.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_baseline_hard_grid_output
                       PROPERTIES FIXTURES_SETUP kite_baseline_hard_grid_table)
  add_test(NAME kite_baseline_hard_grid_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/grid_hard.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/BASELINE.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_baseline_hard_grid_doc_matches
                       PROPERTIES FIXTURES_REQUIRED kite_baseline_hard_grid_table)

  # On seed 101 alone the three q 10 settings tie on cost (10 errors) and differ in rmse_m: 0.792,
  # 0.789 and 0.790 for velocity_sigma 5, 10 and 20. The rule picks 10. Keeping the first of the
  # tie would pick 5, and preferring the higher rmse_m would pick 5 too.
  add_test(NAME kite_baseline_hard_grid_tie_break
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline> "-DARGS=--scenario;hard;--grid;101-101"
            "-DEXPECT_MATCH=[|] 10[.]000 [|] 5[.]000 [|] 1495[.]000 [|] 10[.]000 [|] 0[.]000 [|] 0[.]792 [|] 0[.]000 [|] 301[.]000 [|] 10[.]000 [|].*[|] 10[.]000 [|] 10[.]000 [|] 1495[.]000 [|] 10[.]000 [|] 0[.]000 [|] 0[.]789 [|] 0[.]000 [|] 301[.]000 [|] 10[.]000 [|].*[|] 10[.]000 [|] 20[.]000 [|] 1495[.]000 [|] 10[.]000 [|] 0[.]000 [|] 0[.]790 [|] 0[.]000 [|] 301[.]000 [|] 10[.]000 [|].*selected: q 10[.]000, velocity_sigma 10[.]000"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/grid_hard_tie.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)

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

  kite_exit_test(kite_association_study_scenario_bad_value kite_association_study 2
    "--scenario;bogus")
  kite_exit_test(kite_association_study_scenario_missing_value kite_association_study 2
    "--summary;--scenario")
  kite_exit_test(kite_association_study_scenario_duplicate kite_association_study 2
    "--scenario;hard;--scenario;straight")

  # --scenario straight is the default, so its output is what the doc-matches tests pin.
  add_test(NAME kite_association_study_scenario_straight_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_association_study>
            "-DARGS=--scenario;straight;--association;joint"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/association_study_straight.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_association_study_scenario_straight_output
                       PROPERTIES FIXTURES_SETUP kite_association_study_straight_tables)
  add_test(NAME kite_association_study_scenario_straight_matches_default
    COMMAND ${CMAKE_COMMAND} -E compare_files
            ${CMAKE_CURRENT_BINARY_DIR}/association_study.md
            ${CMAKE_CURRENT_BINARY_DIR}/association_study_straight.md)
  set_tests_properties(kite_association_study_scenario_straight_matches_default
                       PROPERTIES FIXTURES_REQUIRED
                       "kite_association_study_tables;kite_association_study_straight_tables")

  add_test(NAME kite_association_study_scenario_straight_summary_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_association_study>
            "-DARGS=--scenario;straight;--association;joint;--summary"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/association_study_straight_summary.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_association_study_scenario_straight_summary_output
                       PROPERTIES FIXTURES_SETUP kite_association_study_straight_summary)
  add_test(NAME kite_association_study_scenario_straight_summary_matches_default
    COMMAND ${CMAKE_COMMAND} -E compare_files
            ${CMAKE_CURRENT_BINARY_DIR}/association_study_joint_summary.md
            ${CMAKE_CURRENT_BINARY_DIR}/association_study_straight_summary.md)
  set_tests_properties(kite_association_study_scenario_straight_summary_matches_default
                       PROPERTIES FIXTURES_REQUIRED
                       "kite_association_study_joint_summary;kite_association_study_straight_summary")

  add_test(NAME kite_association_study_hard_tables_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_association_study>
            "-DARGS=--scenario;hard;--association;joint"
            "-DEXPECT_MATCH=^Association joint, hard scenario, seeds 101-120: [0-9]+ measurements, [0-9]+ clutter, [0-9]+ ambiguous[.].*[|] seed [|] measurements [|] clutter [|] ambiguous [|] steals [|] H1 [|] nearer [|] other [|] duplicates [|]"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/association_study_hard.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)

  foreach(mode two_stage joint joint_likelihood)
    add_test(NAME kite_baseline_hard_association_${mode}_output
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_baseline>
              "-DARGS=--scenario;hard;--tracker;kalman;--association;${mode};--seeds;101-120"
              -DOUT=${CMAKE_CURRENT_BINARY_DIR}/kalman_hard_${mode}_tuning.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
    set_tests_properties(kite_baseline_hard_association_${mode}_output
                         PROPERTIES FIXTURES_SETUP kite_baseline_hard_association_${mode}_table)
    add_test(NAME kite_baseline_hard_association_${mode}_doc_matches
      COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/kalman_hard_${mode}_tuning.md
              "-DHEADING=hard ${mode}:"
              -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
    set_tests_properties(kite_baseline_hard_association_${mode}_doc_matches
                         PROPERTIES FIXTURES_REQUIRED kite_baseline_hard_association_${mode}_table)

    add_test(NAME kite_association_study_hard_${mode}_summary_output
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_association_study>
              "-DARGS=--scenario;hard;--association;${mode};--summary"
              "-DEXPECT_MATCH=^Association ${mode}, hard scenario, seeds 101-120:"
              -DOUT=${CMAKE_CURRENT_BINARY_DIR}/association_study_hard_${mode}_summary.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
    set_tests_properties(kite_association_study_hard_${mode}_summary_output
                         PROPERTIES FIXTURES_SETUP kite_association_study_hard_${mode}_summary)
    add_test(NAME kite_association_study_hard_${mode}_summary_doc_matches
      COMMAND ${CMAKE_COMMAND}
              -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/association_study_hard_${mode}_summary.md
              -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
    set_tests_properties(kite_association_study_hard_${mode}_summary_doc_matches
                         PROPERTIES FIXTURES_REQUIRED kite_association_study_hard_${mode}_summary)
  endforeach()

  kite_exit_test(kite_drop_study_help kite_drop_study 0 "--help")
  kite_exit_test(kite_drop_study_no_args kite_drop_study 2 "")
  kite_exit_test(kite_drop_study_unknown_flag kite_drop_study 2 "--bogus")
  kite_exit_test(kite_drop_study_two_flags kite_drop_study 2 "--monte-carlo;--tuning")

  foreach(study monte_carlo tuning)
    string(REPLACE "_" "-" flag "--${study}")
    add_test(NAME kite_drop_study_${study}_output
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_drop_study> "-DARGS=${flag}"
              -DOUT=${CMAKE_CURRENT_BINARY_DIR}/drop_study_${study}.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
    set_tests_properties(kite_drop_study_${study}_output
                         PROPERTIES FIXTURES_SETUP kite_drop_study_${study}_tables)
    add_test(NAME kite_drop_study_${study}_doc_matches
      COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/drop_study_${study}.md
              -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
              -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
    set_tests_properties(kite_drop_study_${study}_doc_matches
                         PROPERTIES FIXTURES_REQUIRED kite_drop_study_${study}_tables)
  endforeach()

  kite_exit_test(kite_drop_study_scenario_bad_value kite_drop_study 2 "--tuning;--scenario;bogus")
  kite_exit_test(kite_drop_study_scenario_missing_value kite_drop_study 2 "--tuning;--scenario")
  kite_exit_test(kite_drop_study_scenario_duplicate kite_drop_study 2
    "--tuning;--scenario;hard;--scenario;straight")
  kite_exit_test(kite_drop_study_scenario_without_a_study kite_drop_study 2 "--scenario;hard")
  kite_exit_test(kite_drop_study_monte_carlo_hard kite_drop_study 2
    "--monte-carlo;--scenario;hard")

  # --scenario straight is the default, so its output is what the doc-matches test pins.
  add_test(NAME kite_drop_study_scenario_straight_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_drop_study> "-DARGS=--tuning;--scenario;straight"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/drop_study_tuning_straight.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_drop_study_scenario_straight_output
                       PROPERTIES FIXTURES_SETUP kite_drop_study_tuning_straight_table)
  add_test(NAME kite_drop_study_scenario_straight_matches_default
    COMMAND ${CMAKE_COMMAND} -E compare_files
            ${CMAKE_CURRENT_BINARY_DIR}/drop_study_tuning.md
            ${CMAKE_CURRENT_BINARY_DIR}/drop_study_tuning_straight.md)
  set_tests_properties(kite_drop_study_scenario_straight_matches_default
                       PROPERTIES FIXTURES_REQUIRED
                       "kite_drop_study_tuning_tables;kite_drop_study_tuning_straight_table")

  add_test(NAME kite_drop_study_hard_tuning_output
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:kite_drop_study> "-DARGS=--tuning;--scenario;hard"
            "-DEXPECT_MATCH=^Tentative tracks, two_stage, hard scenario, seeds 101-120:"
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/drop_study_hard_tuning.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_to_file.cmake)
  set_tests_properties(kite_drop_study_hard_tuning_output
                       PROPERTIES FIXTURES_SETUP kite_drop_study_hard_tuning_tables)
  add_test(NAME kite_drop_study_hard_tuning_doc_matches
    COMMAND ${CMAKE_COMMAND} -DTABLE=${CMAKE_CURRENT_BINARY_DIR}/drop_study_hard_tuning.md
            -DDOC=${PROJECT_SOURCE_DIR}/docs/INVESTIGATION-ASSOCIATION.md
            -P ${CMAKE_CURRENT_SOURCE_DIR}/doc_contains.cmake)
  set_tests_properties(kite_drop_study_hard_tuning_doc_matches
                       PROPERTIES FIXTURES_REQUIRED kite_drop_study_hard_tuning_tables)
endif()
