#pragma once

#include <filesystem>
#include <vector>

#include "kite/types.hpp"

namespace kite {

struct LogRecords {
  std::vector<GroundTruthState> truth;
  std::vector<Measurement> measurements;
  std::vector<TrackState> tracks;
};

void write_log(const std::filesystem::path& path, const LogRecords& records);

LogRecords read_log(const std::filesystem::path& path);

}
