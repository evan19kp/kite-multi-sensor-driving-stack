#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <vector>

#include "baseline_setup.hpp"
#include "drop_study.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/types.hpp"
#include "study_common.hpp"

namespace {

using kite::baseline_setup::Scenario;
using kite::drop_study::TentativeClassifier;
using kite::drop_study::TentativeOutcome;
using kite::study::Label;
using kite::study::LabelCounts;
using kite::study::LabelKind;

const kite::Vec2 kOrigin{0.0, 0.0};

kite::GroundTruthState vehicle(int id, double x) {
  return kite::GroundTruthState{0.0, 0, id, {x, 0.0}, {0.0, 0.0}};
}

Label classify(const std::vector<kite::GroundTruthState>& truth) {
  return kite::study::classify_measurement(kOrigin, truth);
}

TEST(ClassifyMeasurement, WithNoTruthVehicleAtAllIsClutter) {
  EXPECT_EQ(classify({}).kind, LabelKind::clutter);
}

TEST(ClassifyMeasurement, NoTruthVehicleWithinTheGateIsClutterNotAmbiguous) {
  // The nearest vehicle is 4 m away and the second is 4.2 m away. Neither is within 3 m, and the
  // 0.2 m gap would be ambiguous if the gate were passed, so only the gate makes it clutter.
  const Label label = classify({vehicle(1, 4.0), vehicle(2, 4.2)});
  EXPECT_EQ(label.kind, LabelKind::clutter);
  EXPECT_FALSE(label.as_car());
}

TEST(ClassifyMeasurement, ANearestVehicleExactlyAtTheGateIsLabelled) {
  const Label label = classify({vehicle(7, 3.0)});
  EXPECT_EQ(label.kind, LabelKind::car);
  EXPECT_EQ(label.as_car(), kite::study::CarLabel(7));
}

TEST(ClassifyMeasurement, ANearestVehicleJustBeyondTheGateIsClutter) {
  EXPECT_EQ(classify({vehicle(7, 3.001)}).kind, LabelKind::clutter);
}

TEST(ClassifyMeasurement, ASecondNearestExactlyOneMetreFartherStillLabels) {
  // 1 m and 2 m are exact in binary floating point, so the margin is exactly 1 m.
  const Label label = classify({vehicle(5, 1.0), vehicle(6, 2.0)});
  EXPECT_EQ(label.kind, LabelKind::car);
  EXPECT_EQ(label.as_car(), kite::study::CarLabel(5));
}

TEST(ClassifyMeasurement, ASecondNearestUnderOneMetreFartherIsAmbiguous) {
  const Label label = classify({vehicle(5, 1.0), vehicle(6, 1.99)});
  EXPECT_EQ(label.kind, LabelKind::ambiguous);
  EXPECT_FALSE(label.as_car());
}

TEST(ClassifyMeasurement, TwoVehiclesEquallyNearAreAmbiguous) {
  EXPECT_EQ(classify({vehicle(5, 2.0), vehicle(6, -2.0)}).kind, LabelKind::ambiguous);
}

TEST(ClassifyMeasurement, ASecondVehicleInsideTheGateMakesAnOtherwiseNearMeasurementAmbiguous) {
  EXPECT_EQ(classify({vehicle(5, 2.9), vehicle(6, 3.0)}).kind, LabelKind::ambiguous);
}

TEST(ClassifyMeasurement, TheTruthOrderDoesNotChangeTheLabel) {
  EXPECT_EQ(classify({vehicle(6, 2.0), vehicle(5, 1.0)}).as_car(), kite::study::CarLabel(5));
}

TEST(LabelCounts, CountClutterAndAmbiguousApartAndNeverACar) {
  LabelCounts counts;
  counts.add({LabelKind::clutter});
  counts.add({LabelKind::clutter});
  counts.add({LabelKind::ambiguous});
  counts.add({LabelKind::car, 3});
  EXPECT_EQ(counts.clutter, 2);
  EXPECT_EQ(counts.ambiguous, 1);
  EXPECT_EQ(counts.unlabelled(), 3);
}

TEST(LabelSummary, TheStraightScenarioPrintsOneAmbiguousCountAsItAlwaysDid) {
  const LabelCounts counts{2, 3};
  EXPECT_EQ(kite::study::label_summary(Scenario::straight, 10, counts),
            "10 measurements, 5 ambiguous");
  EXPECT_EQ(kite::study::label_headers(Scenario::straight), std::vector<std::string>{"ambiguous"});
  EXPECT_EQ(kite::study::label_headers(Scenario::straight, "born "),
            std::vector<std::string>{"born ambiguous"});
  EXPECT_EQ(kite::study::label_cells(Scenario::straight, counts), std::vector<std::string>{"5"});
  EXPECT_EQ(kite::study::scenario_suffix(Scenario::straight), "");
}

TEST(LabelSummary, TheHardScenarioStatesClutterAndAmbiguousApart) {
  const LabelCounts counts{2, 3};
  EXPECT_EQ(kite::study::label_summary(Scenario::hard, 10, counts),
            "10 measurements, 2 clutter, 3 ambiguous");
  EXPECT_EQ(kite::study::label_headers(Scenario::hard),
            (std::vector<std::string>{"clutter", "ambiguous"}));
  EXPECT_EQ(kite::study::label_headers(Scenario::hard, "born ", " birth"),
            (std::vector<std::string>{"born clutter birth", "born ambiguous birth"}));
  EXPECT_EQ(kite::study::label_cells(Scenario::hard, counts),
            (std::vector<std::string>{"2", "3"}));
  EXPECT_EQ(kite::study::scenario_suffix(Scenario::hard), ", hard scenario");
}

struct LabelledStep {
  std::vector<kite::Measurement> measurements;
  std::vector<Label> labels;
};

kite::Measurement at(double x) { return kite::Measurement{0.0, 0, 0, {x, 0.0}}; }

void add(std::vector<LabelledStep>& steps, std::size_t step, double x, Label label) {
  steps[step].measurements.push_back(at(x));
  steps[step].labels.push_back(label);
}

TentativeOutcome classify_steps(const std::vector<LabelledStep>& steps) {
  kite::KalmanTracker tracker =
      kite::baseline_setup::make_kalman_tracker(kite::AssociationMode::two_stage);
  tracker.record_steps(true);
  TentativeClassifier classifier(kite::baseline_setup::kKalmanGateChi2);
  for (std::size_t k = 0; k < steps.size(); ++k) {
    const std::vector<kite::TrackState> output =
        tracker.update(0.1 * static_cast<double>(k), steps[k].measurements);
    classifier.observe_labelled(tracker.step_record().value(), steps[k].measurements, output,
                                steps[k].labels);
  }
  return classifier.outcome();
}

Label car(int id) { return {LabelKind::car, id}; }

// Two births 50 m apart at the steps given. The first track is dropped by its third miss, so the
// second measurement always starts a track of its own.
TentativeOutcome two_births(std::size_t first, std::size_t second, Label first_label,
                            Label second_label) {
  std::vector<LabelledStep> steps(second + 1);
  add(steps, first, 0.0, first_label);
  add(steps, second, 50.0, second_label);
  return classify_steps(steps);
}

TEST(TentativeClassifierDuplicates, TwoBirthsOfOneCarExactlyFiveStepsApartCount) {
  const TentativeOutcome outcome = two_births(0, 5, car(7), car(7));
  EXPECT_EQ(outcome.tentative, 2);
  EXPECT_EQ(outcome.duplicate_tentative_tracks, 1);
}

TEST(TentativeClassifierDuplicates, TwoBirthsOfOneCarSixStepsApartDoNotCount) {
  const TentativeOutcome outcome = two_births(0, 6, car(7), car(7));
  EXPECT_EQ(outcome.tentative, 2);
  EXPECT_EQ(outcome.duplicate_tentative_tracks, 0);
}

TEST(TentativeClassifierDuplicates, BirthsOfDifferentCarsDoNotCount) {
  EXPECT_EQ(two_births(0, 5, car(7), car(8)).duplicate_tentative_tracks, 0);
  EXPECT_EQ(two_births(0, 1, car(7), car(8)).duplicate_tentative_tracks, 0);
}

TEST(TentativeClassifierDuplicates, TwoBirthsOfOneCarInTheSameStepCount) {
  std::vector<LabelledStep> steps(1);
  add(steps, 0, 0.0, car(7));
  add(steps, 0, 50.0, car(7));
  EXPECT_EQ(classify_steps(steps).duplicate_tentative_tracks, 1);
}

TEST(TentativeClassifierDuplicates, ClutterAndAmbiguousBirthsAreNeverDuplicates) {
  const Label clutter{LabelKind::clutter};
  const Label ambiguous{LabelKind::ambiguous};
  EXPECT_EQ(two_births(0, 3, clutter, clutter).duplicate_tentative_tracks, 0);
  EXPECT_EQ(two_births(0, 3, ambiguous, ambiguous).duplicate_tentative_tracks, 0);
  EXPECT_EQ(two_births(0, 3, car(7), ambiguous).duplicate_tentative_tracks, 0);
}

TEST(TentativeClassifierDuplicates, EachLaterBirthWithinFiveStepsOfAnEarlierOneCountsOnce) {
  // Births of car 7 at steps 0, 3 and 6. The one at 3 follows the one at 0; the one at 6 follows
  // the one at 3 (3 steps) though it is 6 steps after the one at 0. Two tracks count, not three.
  std::vector<LabelledStep> steps(7);
  add(steps, 0, 0.0, car(7));
  add(steps, 3, 50.0, car(7));
  add(steps, 6, 100.0, car(7));
  const TentativeOutcome outcome = classify_steps(steps);
  EXPECT_EQ(outcome.tentative, 3);
  EXPECT_EQ(outcome.duplicate_tentative_tracks, 2);
}

TEST(TentativeClassifierBirths, ADroppedClutterBirthIsCountedApartFromAnAmbiguousOne) {
  std::vector<LabelledStep> clutter_steps(5);
  add(clutter_steps, 0, 0.0, Label{LabelKind::clutter});
  const TentativeOutcome clutter = classify_steps(clutter_steps);
  EXPECT_EQ(clutter.dropped, 1);
  EXPECT_EQ(clutter.clutter_births, 1);
  EXPECT_EQ(clutter.ambiguous_births, 0);
  EXPECT_EQ(clutter.births().unlabelled(), 1);

  std::vector<LabelledStep> ambiguous_steps(5);
  add(ambiguous_steps, 0, 0.0, Label{LabelKind::ambiguous});
  const TentativeOutcome ambiguous = classify_steps(ambiguous_steps);
  EXPECT_EQ(ambiguous.dropped, 1);
  EXPECT_EQ(ambiguous.clutter_births, 0);
  EXPECT_EQ(ambiguous.ambiguous_births, 1);
  EXPECT_EQ(ambiguous.births().unlabelled(), 1);
}

}
