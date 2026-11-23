#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "baseline_setup.hpp"
#include "kite/generator.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/matrix.hpp"
#include "kite/scoring.hpp"
#include "kite/types.hpp"
#include "steal_class.hpp"
#include "study_common.hpp"

namespace {

using namespace kite::baseline_setup;
using namespace kite::study;

constexpr std::size_t kDuplicateWindowSteps = 5;

constexpr const char* kUsage =
    "usage: kite_association_study [--scenario straight|hard] [--association MODE] [--summary]\n"
    "Runs KalmanTracker with the kite_baseline scenario and parameters on seeds 101-120, labels\n"
    "each measurement with a truth vehicle (nearest within 3 m and at least 1 m nearer than the\n"
    "second nearest, else ambiguous; clutter when no truth vehicle is within 3 m), and prints\n"
    "Markdown tables of the measurements that tentative tracks took from confirmed tracks of the\n"
    "same vehicle.\n"
    "  --scenario     straight (default) or hard, as in kite_baseline; the straight tables count\n"
    "                 clutter and ambiguous measurements together as ambiguous, the hard ones apart\n"
    "  --association  two_stage, joint (default) or joint_likelihood\n"
    "  --summary      print only one table of steals, duplicates and reverse steals per seed\n"
    "  --help         print this text\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct Options {
  Scenario scenario = Scenario::straight;
  kite::AssociationMode association = kite::AssociationMode::joint;
  bool summary = false;
};

Options parse_options(int argc, char** argv) {
  Options options;
  std::set<std::string_view> seen;
  for (int i = 1; i < argc; ++i) {
    const std::string_view flag = argv[i];
    if (flag != "--association" && flag != "--summary" && flag != "--scenario") {
      throw UsageError("unknown argument: " + std::string(flag));
    }
    if (!seen.insert(flag).second) {
      throw UsageError("duplicate argument: " + std::string(flag));
    }
    if (flag == "--summary") {
      options.summary = true;
      continue;
    }
    if (++i == argc) {
      throw UsageError("missing value for " + std::string(flag));
    }
    if (flag == "--scenario") {
      const std::optional<Scenario> scenario = parse_scenario(argv[i]);
      if (!scenario) {
        throw UsageError("invalid value for --scenario: '" + std::string(argv[i]) + "'");
      }
      options.scenario = *scenario;
      continue;
    }
    const std::optional<kite::AssociationMode> mode = parse_association(argv[i]);
    if (!mode) {
      throw UsageError("invalid value for --association: '" + std::string(argv[i]) + "'");
    }
    options.association = *mode;
  }
  return options;
}

constexpr std::array<StealClass, 3> kClasses = {StealClass::h1, StealClass::nearer,
                                                StealClass::other};

const char* class_name(StealClass kind) {
  switch (kind) {
    case StealClass::h1:
      return "H1";
    case StealClass::nearer:
      return "nearer";
    case StealClass::other:
      return "other";
  }
  throw std::logic_error("unknown steal class");
}

struct Birth {
  Label label;
  bool inside_a_gate;
};

struct Steal {
  std::size_t step;
  int car;
  StealClass kind;
  double distance_ratio;
  double det_ratio;
  bool confirmed_outside_gate;
  bool tentative_scores_lower;
  bool tentative_det_larger;
  int tentative_age;
  Birth birth;
  bool duplicate;
};

struct SeedResult {
  int seed;
  int measurements;
  LabelCounts unlabelled;
  std::vector<Steal> steals;
  int reverse_steals;
};

bool inside_a_gate(const kite::StepRecord& record, kite::Vec2 z) {
  return std::any_of(record.tracks.begin(), record.tracks.end(),
                     [z](const kite::TrackRecord& track) {
                       return mahalanobis2(track, z) <= kKalmanGateChi2;
                     });
}

Steal make_steal(std::size_t step, int car, kite::Vec2 z, const kite::TrackRecord& tentative,
                 const kite::TrackRecord& confirmed, const Birth& birth) {
  const double tentative_distance = distance(z, tentative.predicted);
  const double confirmed_distance = distance(z, confirmed.predicted);
  if (confirmed_distance == 0.0) {
    throw std::runtime_error("a stolen measurement sits on the confirmed track's prediction");
  }
  const double tentative_d2 = mahalanobis2(tentative, z);
  const double confirmed_d2 = mahalanobis2(confirmed, z);
  const double tentative_det = det(tentative.innovation_covariance);
  const double confirmed_det = det(confirmed.innovation_covariance);
  const bool scores_lower = tentative_d2 < confirmed_d2;
  const bool det_larger = tentative_det > confirmed_det;
  return {step,
          car,
          classify_steal(z, tentative, confirmed),
          tentative_distance / confirmed_distance,
          tentative_det / confirmed_det,
          confirmed_d2 > kKalmanGateChi2,
          scores_lower,
          det_larger,
          tentative.age_steps(),
          birth,
          false};
}

bool duplicate_follows(const std::vector<std::vector<CarLabel>>& confirmed_labels,
                       std::size_t step, int car) {
  const std::size_t last = std::min(step + kDuplicateWindowSteps, confirmed_labels.size() - 1);
  for (std::size_t j = step; j <= last; ++j) {
    if (std::count(confirmed_labels[j].begin(), confirmed_labels[j].end(), CarLabel(car)) >= 2) {
      return true;
    }
  }
  return false;
}

SeedResult run_seed(int seed, Scenario scenario_kind, kite::AssociationMode association) {
  const kite::GeneratedScenario scenario =
      kite::generate(scenario_config(scenario_kind, static_cast<std::uint64_t>(seed)));
  kite::KalmanTracker tracker = make_kalman_tracker(association);
  tracker.record_steps(true);

  SeedResult result{seed, 0, {}, {}, 0};
  std::map<int, CarLabel> latest_label;
  std::map<int, Birth> births;
  std::vector<std::vector<CarLabel>> confirmed_labels;
  for (std::size_t k = 0; k < scenario.steps.size(); ++k) {
    const kite::ScenarioStep& step = scenario.steps[k];
    const std::vector<kite::TrackState> output = tracker.update(step.t, step.measurements);
    const kite::StepRecord& record = tracker.step_record().value();

    std::vector<Label> kinds;
    std::vector<CarLabel> labels;
    for (const kite::Measurement& z : step.measurements) {
      kinds.push_back(classify_measurement(z.position, step.truth));
      labels.push_back(kinds.back().as_car());
      ++result.measurements;
      result.unlabelled.add(kinds.back());
    }

    for (const kite::TrackRecord& tentative : record.tracks) {
      if (tentative.stage != kite::TrackStage::tentative || !tentative.measurement ||
          !labels[*tentative.measurement]) {
        continue;
      }
      const int car = *labels[*tentative.measurement];
      const kite::Vec2 z = step.measurements[*tentative.measurement].position;
      const kite::TrackRecord* victim = nullptr;
      for (const kite::TrackRecord& confirmed : record.tracks) {
        if (confirmed.stage == kite::TrackStage::confirmed && !confirmed.measurement &&
            latest_label.at(confirmed.track_id) == CarLabel(car) &&
            (victim == nullptr ||
             distance(z, confirmed.predicted) < distance(z, victim->predicted))) {
          victim = &confirmed;
        }
      }
      if (victim != nullptr) {
        result.steals.push_back(
            make_steal(k, car, z, tentative, *victim, births.at(tentative.track_id)));
      }
    }

    for (const kite::TrackRecord& track : record.tracks) {
      if (!track.measurement) {
        continue;
      }
      const CarLabel taken = labels[*track.measurement];
      CarLabel& previous = latest_label.at(track.track_id);
      if (track.stage == kite::TrackStage::confirmed && taken && previous && taken != previous) {
        ++result.reverse_steals;
      }
      previous = taken;
    }
    for (std::size_t m = 0; m < step.measurements.size(); ++m) {
      if (const std::optional<int> id = record.started_track[m]) {
        latest_label[*id] = labels[m];
        births[*id] = Birth{kinds[m], inside_a_gate(record, step.measurements[m].position)};
      }
    }
    std::vector<CarLabel> after;
    for (const kite::TrackState& track : output) {
      after.push_back(latest_label.at(track.track_id));
    }
    confirmed_labels.push_back(after);
  }
  for (Steal& steal : result.steals) {
    steal.duplicate = duplicate_follows(confirmed_labels, steal.step, steal.car);
  }
  return result;
}

template <typename Predicate>
int count_if(const std::vector<Steal>& steals, Predicate predicate) {
  return static_cast<int>(std::count_if(steals.begin(), steals.end(), predicate));
}

std::vector<std::string> median_min_max(std::vector<double> values) {
  if (values.empty()) {
    return {kite::format_fixed(std::nullopt), kite::format_fixed(std::nullopt),
            kite::format_fixed(std::nullopt)};
  }
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  const double median = n % 2 == 1 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2.0;
  return {kite::format_fixed(median), kite::format_fixed(values.front()),
          kite::format_fixed(values.back())};
}

std::vector<Steal> of_class(const std::vector<Steal>& steals, StealClass kind) {
  std::vector<Steal> result;
  std::copy_if(steals.begin(), steals.end(), std::back_inserter(result),
               [kind](const Steal& s) { return s.kind == kind; });
  return result;
}

template <typename Field>
std::vector<double> collect(const std::vector<Steal>& steals, Field field) {
  std::vector<double> values;
  for (const Steal& steal : steals) {
    values.push_back(field(steal));
  }
  return values;
}

std::vector<std::string> measurement_cells(const std::string& label, Scenario scenario,
                                           int measurements, const LabelCounts& unlabelled) {
  std::vector<std::string> cells{label, count(measurements)};
  append(cells, label_cells(scenario, unlabelled));
  return cells;
}

std::string seed_row(const std::string& label, Scenario scenario, const std::vector<Steal>& steals,
                     int measurements, const LabelCounts& unlabelled) {
  std::vector<std::string> cells = measurement_cells(label, scenario, measurements, unlabelled);
  cells.push_back(count(static_cast<int>(steals.size())));
  for (const StealClass kind : kClasses) {
    cells.push_back(count(count_if(steals, [kind](const Steal& s) { return s.kind == kind; })));
  }
  cells.push_back(count(count_if(steals, [](const Steal& s) { return s.duplicate; })));
  return row(cells);
}

std::string summary_row(const std::string& label, Scenario scenario,
                        const std::vector<Steal>& steals, int measurements,
                        const LabelCounts& unlabelled, int reverse_steals) {
  std::vector<std::string> cells = measurement_cells(label, scenario, measurements, unlabelled);
  cells.push_back(count(static_cast<int>(steals.size())));
  cells.push_back(count(count_if(steals, [](const Steal& s) { return s.duplicate; })));
  cells.push_back(count(reverse_steals));
  return row(cells);
}

std::string header(Scenario scenario, const std::vector<std::string>& first,
                   const std::vector<std::string>& rest) {
  std::vector<std::string> names = first;
  append(names, label_headers(scenario));
  append(names, rest);
  return table_header(names);
}

std::string score_row(const std::string& label, const std::vector<Steal>& steals) {
  std::vector<std::string> cells{
      label, count(static_cast<int>(steals.size())),
      count(count_if(steals, [](const Steal& s) { return s.confirmed_outside_gate; })),
      count(count_if(steals, [](const Steal& s) { return s.tentative_scores_lower; })),
      count(count_if(steals, [](const Steal& s) { return s.tentative_det_larger; }))};
  append(cells, median_min_max(collect(steals, [](const Steal& s) { return s.distance_ratio; })));
  append(cells, median_min_max(collect(steals, [](const Steal& s) { return s.det_ratio; })));
  return row(cells);
}

std::string birth_row(const std::string& label, Scenario scenario, const std::vector<Steal>& steals) {
  std::vector<std::string> cells{label, count(static_cast<int>(steals.size()))};
  append(cells, median_min_max(collect(
                    steals, [](const Steal& s) { return static_cast<double>(s.tentative_age); })));
  cells.push_back(count(count_if(steals, [](const Steal& s) { return s.birth.label.as_car() == s.car; })));
  cells.push_back(count(count_if(steals, [](const Steal& s) {
    return s.birth.label.as_car() && s.birth.label.as_car() != s.car;
  })));
  LabelCounts unlabelled;
  for (const Steal& steal : steals) {
    unlabelled.add(steal.birth.label);
  }
  append(cells, label_cells(scenario, unlabelled));
  cells.push_back(count(count_if(steals, [](const Steal& s) { return !s.birth.inside_a_gate; })));
  cells.push_back(count(count_if(steals, [](const Steal& s) { return s.birth.inside_a_gate; })));
  return row(cells);
}

}

int main(int argc, char** argv) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::cout << kUsage;
    return 0;
  }
  Options options;
  try {
    options = parse_options(argc, argv);
  } catch (const UsageError& error) {
    std::cerr << error.what() << "\n" << kUsage;
    return 2;
  }

  const Scenario scenario = options.scenario;
  std::vector<SeedResult> seeds;
  for (int seed = kFirstTuningSeed; seed <= kLastTuningSeed; ++seed) {
    seeds.push_back(run_seed(seed, scenario, options.association));
  }
  std::vector<Steal> all;
  int measurements = 0;
  LabelCounts unlabelled;
  int reverse_steals = 0;
  for (const SeedResult& seed : seeds) {
    all.insert(all.end(), seed.steals.begin(), seed.steals.end());
    measurements += seed.measurements;
    unlabelled += seed.unlabelled;
    reverse_steals += seed.reverse_steals;
  }
  const std::string caption = "Association " + std::string(association_name(options.association)) +
                              scenario_suffix(scenario) + ", seeds 101-120: " +
                              label_summary(scenario, measurements, unlabelled) + ".\n\n";

  if (options.summary) {
    std::string out =
        caption + header(scenario, {"seed", "measurements"}, {"steals", "duplicates", "reverse steals"});
    for (const SeedResult& seed : seeds) {
      out += summary_row(count(seed.seed), scenario, seed.steals, seed.measurements, seed.unlabelled,
                         seed.reverse_steals);
    }
    out += summary_row("total", scenario, all, measurements, unlabelled, reverse_steals);
    std::cout << out;
    return 0;
  }

  std::string out = caption + header(scenario, {"seed", "measurements"},
                                     {"steals", "H1", "nearer", "other", "duplicates"});
  for (const SeedResult& seed : seeds) {
    out += seed_row(count(seed.seed), scenario, seed.steals, seed.measurements, seed.unlabelled);
  }
  out += seed_row("total", scenario, all, measurements, unlabelled);

  out += "\n" + caption +
         table_header({"class", "steals", "confirmed outside gate", "tentative scores lower",
                       "tentative det S larger", "distance ratio median", "distance ratio min",
                       "distance ratio max", "det S ratio median", "det S ratio min",
                       "det S ratio max"});
  for (const StealClass kind : kClasses) {
    out += score_row(class_name(kind), of_class(all, kind));
  }
  out += score_row("all", all);

  std::vector<std::string> birth_names{"class",
                                       "steals",
                                       "tentative age median",
                                       "tentative age min",
                                       "tentative age max",
                                       "born from same car",
                                       "born from other car"};
  append(birth_names, label_headers(scenario, "born "));
  append(birth_names, {"born outside every gate", "born inside a gate"});
  out += "\n" + caption + table_header(birth_names);
  for (const StealClass kind : kClasses) {
    out += birth_row(class_name(kind), scenario, of_class(all, kind));
  }
  out += birth_row("all", scenario, all);

  std::cout << out;
  return 0;
}
