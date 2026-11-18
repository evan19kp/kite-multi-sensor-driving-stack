#include "kite/log.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace {

namespace fs = std::filesystem;

constexpr const char* kHeaderLine = "# kite-log v0\n";

class LogFiles : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    dir_ = fs::temp_directory_path() /
           ("kite_log_test_" +
            std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
            "_" + std::to_string(ticks));
    fs::create_directories(dir_);
  }

  void TearDown() override {
    std::error_code ignored;
    fs::remove_all(dir_, ignored);
  }

  fs::path path() const { return dir_ / "log.txt"; }

  void write_text(const std::string& text) const {
    std::ofstream file(path(), std::ios::binary);
    file << text;
  }

  std::string read_text() const {
    std::ifstream file(path(), std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
  }

  std::string read_error(const std::string& text) const {
    write_text(text);
    try {
      kite::read_log(path());
    } catch (const std::runtime_error& error) {
      return error.what();
    }
    ADD_FAILURE() << "read_log accepted: " << text;
    return "";
  }

  fs::path dir_;
};

kite::LogRecords tiny_log() {
  kite::LogRecords records;
  records.truth = {{0.0, 0, 1, {12.5, -3.25}, {8.0, 0.0}},
                   {0.1, 0, 1, {13.3, -3.25}, {8.0, 0.0}}};
  records.measurements = {{0.1, 0, 0, {13.31, -3.2}}};
  records.tracks = {{0.1, 0, 1, {13.3, -3.25}, {7.9, 0.05}}};
  return records;
}

const std::string kTinyLogBytes =
    "# kite-log v0\n"
    "gt,0,0,1,12.5,-3.25,8,0\n"
    "gt,0.1,0,1,13.3,-3.25,8,0\n"
    "meas,0.1,0,0,13.31,-3.2\n"
    "track,0.1,0,1,13.3,-3.25,7.9,0.05\n";

TEST_F(LogFiles, WritesExactBytesForTinyLog) {
  kite::write_log(path(), tiny_log());
  EXPECT_EQ(read_text(), kTinyLogBytes);
}

TEST_F(LogFiles, EmptyLogIsJustTheHeader) {
  kite::write_log(path(), kite::LogRecords{});
  EXPECT_EQ(read_text(), kHeaderLine);
  const kite::LogRecords back = kite::read_log(path());
  EXPECT_TRUE(back.truth.empty());
  EXPECT_TRUE(back.measurements.empty());
  EXPECT_TRUE(back.tracks.empty());
}

TEST_F(LogFiles, RoundTripsTinyLog) {
  const kite::LogRecords records = tiny_log();
  kite::write_log(path(), records);
  const kite::LogRecords back = kite::read_log(path());
  EXPECT_EQ(back.truth, records.truth);
  EXPECT_EQ(back.measurements, records.measurements);
  EXPECT_EQ(back.tracks, records.tracks);
}

TEST_F(LogFiles, RoundTripsAwkwardDoublesBitExact) {
  const double values[] = {0.1,
                           -1e-300,
                           1e300,
                           1.0 / 3.0,
                           std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::min(),
                           std::numeric_limits<double>::denorm_min(),
                           0.0,
                           -123456789.123456789};
  kite::LogRecords records;
  int id = -2;
  for (const double v : values) {
    records.truth.push_back({v, id, id + 1, {v, -v}, {-v, v}});
    records.measurements.push_back({v, id, id + 2, {-v, v}});
    records.tracks.push_back({v, id, id + 3, {v, v}, {-v, -v}});
    ++id;
  }
  records.truth.push_back({1.0, std::numeric_limits<int>::max(),
                           std::numeric_limits<int>::min(), {0.0, 0.0}, {0.0, 0.0}});
  kite::write_log(path(), records);
  const kite::LogRecords back = kite::read_log(path());
  EXPECT_EQ(back.truth, records.truth);
  EXPECT_EQ(back.measurements, records.measurements);
  EXPECT_EQ(back.tracks, records.tracks);
}

TEST_F(LogFiles, RoundTripsNegativeZeroSign) {
  kite::LogRecords records;
  records.measurements = {{-0.0, 0, 0, {-0.0, 0.0}}};
  kite::write_log(path(), records);
  const kite::LogRecords back = kite::read_log(path());
  ASSERT_EQ(back.measurements.size(), 1u);
  EXPECT_TRUE(std::signbit(back.measurements[0].t));
  EXPECT_TRUE(std::signbit(back.measurements[0].position.x));
  EXPECT_FALSE(std::signbit(back.measurements[0].position.y));
}

TEST_F(LogFiles, WritingAgainReplacesTheFile) {
  kite::write_log(path(), tiny_log());
  kite::write_log(path(), kite::LogRecords{});
  EXPECT_EQ(read_text(), kHeaderLine);
}

TEST_F(LogFiles, ReadKeepsFileOrderPerRecordType) {
  write_text(std::string(kHeaderLine) +
             "meas,2,0,1,1,1\n"
             "gt,1,0,5,0,0,0,0\n"
             "meas,1,0,1,2,2\n"
             "track,1,0,9,0,0,0,0\n"
             "gt,0,0,5,0,0,0,0\n");
  const kite::LogRecords back = kite::read_log(path());
  ASSERT_EQ(back.measurements.size(), 2u);
  EXPECT_EQ(back.measurements[0].t, 2.0);
  EXPECT_EQ(back.measurements[1].t, 1.0);
  ASSERT_EQ(back.truth.size(), 2u);
  EXPECT_EQ(back.truth[0].t, 1.0);
  EXPECT_EQ(back.truth[1].t, 0.0);
  ASSERT_EQ(back.tracks.size(), 1u);
  EXPECT_EQ(back.tracks[0].track_id, 9);
}

TEST_F(LogFiles, MissingFileThrows) {
  EXPECT_THROW(kite::read_log(dir_ / "absent.txt"), std::runtime_error);
}

TEST_F(LogFiles, UnwritablePathThrows) {
  EXPECT_THROW(kite::write_log(dir_ / "no_such_folder" / "log.txt", tiny_log()), std::runtime_error);
}

TEST_F(LogFiles, EmptyFileReportsMissingHeaderOnLine1) {
  EXPECT_THAT(read_error(""), ::testing::HasSubstr("line 1:"));
}

TEST_F(LogFiles, WrongHeaderReportsLine1) {
  for (const char* text : {"# kite-log v1\n", "kite-log v0\n", "# kite-log v0 \n", "gt,0,0,1,0,0,0,0\n"}) {
    EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 1:")) << text;
  }
}

TEST_F(LogFiles, UnknownRecordTypeNamesItsLine) {
  const std::string text = std::string(kHeaderLine) + "gt,0,0,1,0,0,0,0\nmeas,0,0,1,0,0\nfoo,1,2\n";
  EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 4:"));
  EXPECT_THAT(read_error(std::string(kHeaderLine) + "\n"), ::testing::HasSubstr("line 2:"));
}

TEST_F(LogFiles, WrongFieldCountNamesItsLine) {
  const char* bad_records[] = {"gt,0,0,1,0,0,0",    "gt,0,0,1,0,0,0,0,0", "meas,0,0,1,0",
                               "meas,0,0,1,0,0,0",  "track,0,0,1,0,0,0",  "track,0,0,1,0,0,0,0,0",
                               "gt",                "track"};
  for (const char* record : bad_records) {
    const std::string text = std::string(kHeaderLine) + "meas,0,0,1,0,0\n" + record + "\n";
    EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 3:")) << record;
  }
}

TEST_F(LogFiles, NonNumbersNameTheirLine) {
  const char* bad_records[] = {"gt,abc,0,1,0,0,0,0",    "gt,0,0,1,0,0,0,1.0x",  "meas,0,0,1,,0",
                               "meas,0,0,1,0, 2",       "meas,0,0,1,+1,0",      "meas,0,0,1,1e999,0",
                               "track,0,zero,1,0,0,0,0", "track,0,0,1.5,0,0,0,0", "gt,0,0,99999999999,0,0,0,0",
                               "meas,0,0,1,0,0\r"};
  for (const char* record : bad_records) {
    const std::string text = std::string(kHeaderLine) + "gt,0,0,1,0,0,0,0\n" + record + "\n";
    EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 3:")) << record;
  }
}

TEST_F(LogFiles, NonCanonicalNumbersNameTheirLine) {
  const char* bad_records[] = {"gt,1.50,0,1,0,0,0,0", "meas,0,0,1,1E5,0",     "meas,0,0,1,1e5,0",
                               "meas,0,0,1,.5,0",     "meas,0,0,1,5.,0",      "meas,0,0,1,0,00.5",
                               "meas,0,0,1,1e+5,0",   "meas,0,0,1,1.0,0",     "meas,0,0,1,0.0,0",
                               "gt,0,007,1,0,0,0,0",  "track,0,0,-0,0,0,0,0", "track,0,0,-007,0,0,0,0"};
  for (const char* record : bad_records) {
    const std::string text = std::string(kHeaderLine) + "gt,0,0,1,0,0,0,0\n" + record + "\n";
    EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 3:")) << record;
  }
}

TEST_F(LogFiles, LastLineWithoutNewlineNamesItsLine) {
  std::string text = kTinyLogBytes;
  text.pop_back();
  EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 5:"));

  text.pop_back();
  text.pop_back();
  text.pop_back();
  EXPECT_THAT(text, ::testing::EndsWith("7.9,0"));
  EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 5:"));

  EXPECT_THAT(read_error("# kite-log v0"), ::testing::HasSubstr("line 1:"));
}

TEST_F(LogFiles, NonFiniteValuesNameTheirLine) {
  const char* bad_records[] = {"gt,nan,0,1,0,0,0,0", "meas,0,0,1,inf,0", "track,0,0,1,0,0,-inf,0",
                               "meas,0,0,1,0,NaN",   "meas,0,0,1,0,infinity", "gt,0,0,1,0,0,0,-nan"};
  for (const char* record : bad_records) {
    const std::string text = std::string(kHeaderLine) + "gt,0,0,1,0,0,0,0\n" + record + "\n";
    EXPECT_THAT(read_error(text), ::testing::HasSubstr("line 3:")) << record;
  }
}

TEST_F(LogFiles, WriterRejectsNonFiniteAndLeavesNoFile) {
  const double bad[] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()};
  for (const double v : bad) {
    kite::LogRecords gt = tiny_log();
    gt.truth[1].velocity.y = v;
    kite::LogRecords meas = tiny_log();
    meas.measurements[0].t = v;
    kite::LogRecords track = tiny_log();
    track.tracks[0].position.x = v;
    for (const kite::LogRecords* records : {&gt, &meas, &track}) {
      EXPECT_THROW(kite::write_log(path(), *records), std::runtime_error);
      EXPECT_FALSE(fs::exists(path()));
    }
  }
}

TEST_F(LogFiles, WriterNonFiniteErrorNamesTheFileLine) {
  kite::LogRecords records = tiny_log();
  records.measurements[0].position.y = std::numeric_limits<double>::quiet_NaN();
  try {
    kite::write_log(path(), records);
    FAIL() << "write_log accepted a NaN";
  } catch (const std::runtime_error& error) {
    EXPECT_THAT(error.what(), ::testing::HasSubstr("line 4:"));
  }
}

}
