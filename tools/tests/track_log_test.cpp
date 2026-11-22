#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "baseline_setup.hpp"
#include "kite/baseline_tracker.hpp"
#include "kite/generator.hpp"
#include "kite/scoring.hpp"
#include "kite/types.hpp"
#include "track_log.hpp"

namespace {

kite::LogRecords scenario_records(std::uint64_t seed) {
  return kite::to_log_records(kite::generate(kite::baseline_setup::scenario_config(seed)));
}

kite::ScoreResult score_tracks(const kite::LogRecords& records,
                               const std::vector<kite::TrackState>& tracks) {
  return kite::score(records.truth, tracks, kite::baseline_setup::kMatchGateM);
}

// The rows of docs/BASELINE.md are what kite_baseline prints; the CTest test
// kite_baseline_doc_matches keeps that table equal to the tool's output.
TEST(RunTracker, BaselineTrackerScoresAsKiteBaselineDoesOnSeed1) {
  const kite::LogRecords records = scenario_records(1);
  kite::NearestNeighbourBaseline tracker(kite::baseline_setup::kBaselineTrackerGateM);

  const kite::ScoreResult result =
      score_tracks(records, kite::tool_track::run_tracker(tracker, records.measurements));

  EXPECT_EQ(result.matched, 1398);
  EXPECT_EQ(result.missed, 107);
  EXPECT_EQ(result.false_tracks, 23);
  EXPECT_EQ(kite::format_fixed(result.rmse_m), "1.393");
  EXPECT_EQ(result.id_switches, 87);
  EXPECT_EQ(result.steps, 301);
}

// Seed 101 row printed by `kite_baseline --tracker kalman --seeds 101-101`, pinned in the CTest
// test kite_baseline_tracker_after_seeds.
TEST(RunTracker, KalmanTrackerScoresAsKiteBaselineDoesOnSeed101) {
  const kite::LogRecords records = scenario_records(101);
  kite::KalmanTracker tracker =
      kite::baseline_setup::make_kalman_tracker(kite::AssociationMode::two_stage);

  const kite::ScoreResult result =
      score_tracks(records, kite::tool_track::run_tracker(tracker, records.measurements));

  EXPECT_EQ(result.matched, 1494);
  EXPECT_EQ(result.missed, 11);
  EXPECT_EQ(result.false_tracks, 0);
  EXPECT_EQ(kite::format_fixed(result.rmse_m), "0.412");
  EXPECT_EQ(result.id_switches, 0);
  EXPECT_EQ(result.steps, 301);
}

TEST(RunTracker, SortsStepsIntoTimeOrderAndKeepsEachStepsOrder) {
  const kite::GeneratedScenario scenario = kite::generate(kite::baseline_setup::scenario_config(7));
  std::vector<kite::Measurement> in_time_order;
  std::vector<kite::Measurement> latest_step_first;
  for (auto step = scenario.steps.begin(); step != scenario.steps.end(); ++step) {
    in_time_order.insert(in_time_order.end(), step->measurements.begin(), step->measurements.end());
  }
  for (auto step = scenario.steps.rbegin(); step != scenario.steps.rend(); ++step) {
    latest_step_first.insert(latest_step_first.end(), step->measurements.begin(),
                             step->measurements.end());
  }
  kite::NearestNeighbourBaseline sorted_run(kite::baseline_setup::kBaselineTrackerGateM);
  kite::NearestNeighbourBaseline reversed_run(kite::baseline_setup::kBaselineTrackerGateM);

  const std::vector<kite::TrackState> expected =
      kite::tool_track::run_tracker(sorted_run, in_time_order);

  EXPECT_FALSE(expected.empty());
  EXPECT_EQ(kite::tool_track::run_tracker(reversed_run, latest_step_first), expected);
}

TEST(RunTracker, NoMeasurementsGiveNoTracks) {
  kite::NearestNeighbourBaseline tracker(kite::baseline_setup::kBaselineTrackerGateM);

  EXPECT_TRUE(kite::tool_track::run_tracker(tracker, {}).empty());
}

}
