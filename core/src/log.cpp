#include "kite/log.hpp"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace kite {
namespace {

constexpr std::string_view kHeader = "# kite-log v0";

[[noreturn]] void fail(std::size_t line, const std::string& what) {
  throw std::runtime_error("line " + std::to_string(line) + ": " + what);
}

std::string format_number(int value) {
  char buffer[16];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
  return std::string(buffer, result.ptr);
}

std::string format_number(double value) {
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
  return std::string(buffer, result.ptr);
}

void put_int(std::string& out, int value) {
  out.push_back(',');
  out += format_number(value);
}

void put_double(std::string& out, double value, std::size_t line) {
  if (!std::isfinite(value)) {
    fail(line, "cannot write a non-finite value");
  }
  out.push_back(',');
  out += format_number(value);
}

void put_vec(std::string& out, const Vec2& value, std::size_t line) {
  put_double(out, value.x, line);
  put_double(out, value.y, line);
}

std::string format_log(const LogRecords& records) {
  std::string text(kHeader);
  text.push_back('\n');
  std::size_t line = 1;
  for (const GroundTruthState& r : records.truth) {
    ++line;
    text += "gt";
    put_double(text, r.t, line);
    put_int(text, r.ego_id);
    put_int(text, r.actor_id);
    put_vec(text, r.position, line);
    put_vec(text, r.velocity, line);
    text.push_back('\n');
  }
  for (const Measurement& r : records.measurements) {
    ++line;
    text += "meas";
    put_double(text, r.t, line);
    put_int(text, r.ego_id);
    put_int(text, r.sensor_id);
    put_vec(text, r.position, line);
    text.push_back('\n');
  }
  for (const TrackState& r : records.tracks) {
    ++line;
    text += "track";
    put_double(text, r.t, line);
    put_int(text, r.ego_id);
    put_int(text, r.track_id);
    put_vec(text, r.position, line);
    put_vec(text, r.velocity, line);
    text.push_back('\n');
  }
  return text;
}

struct Fields {
  std::vector<std::string_view> parts;
  std::size_t line;

  void expect_count(std::size_t count) const {
    if (parts.size() != count) {
      fail(line, "expected " + std::to_string(count) + " fields for '" +
                     std::string(parts[0]) + "', found " +
                     std::to_string(parts.size()));
    }
  }

  double number(std::size_t index) const {
    const std::string_view field = parts[index];
    double value = 0.0;
    const char* end = field.data() + field.size();
    const auto result = std::from_chars(field.data(), end, value);
    if (result.ec != std::errc() || result.ptr != end) {
      fail(line, "field " + std::to_string(index + 1) + " is not a number: '" +
                     std::string(field) + "'");
    }
    if (!std::isfinite(value)) {
      fail(line, "field " + std::to_string(index + 1) +
                     " is not finite: '" + std::string(field) + "'");
    }
    if (format_number(value) != field) {
      fail(line, "field " + std::to_string(index + 1) +
                     " is not in canonical form: '" + std::string(field) + "'");
    }
    return value;
  }

  int integer(std::size_t index) const {
    const std::string_view field = parts[index];
    int value = 0;
    const char* end = field.data() + field.size();
    const auto result = std::from_chars(field.data(), end, value);
    if (result.ec != std::errc() || result.ptr != end) {
      fail(line, "field " + std::to_string(index + 1) +
                     " is not an integer: '" + std::string(field) + "'");
    }
    if (format_number(value) != field) {
      fail(line, "field " + std::to_string(index + 1) +
                     " is not in canonical form: '" + std::string(field) + "'");
    }
    return value;
  }
};

Fields split_fields(std::string_view text, std::size_t line) {
  Fields fields{{}, line};
  std::size_t start = 0;
  for (;;) {
    const std::size_t comma = text.find(',', start);
    if (comma == std::string_view::npos) {
      fields.parts.push_back(text.substr(start));
      return fields;
    }
    fields.parts.push_back(text.substr(start, comma - start));
    start = comma + 1;
  }
}

void parse_record(const std::string& text, std::size_t line,
                  LogRecords& records) {
  const Fields f = split_fields(text, line);
  const std::string_view type = f.parts[0];
  if (type == "gt") {
    f.expect_count(8);
    records.truth.push_back({f.number(1),
                             f.integer(2),
                             f.integer(3),
                             {f.number(4), f.number(5)},
                             {f.number(6), f.number(7)}});
  } else if (type == "meas") {
    f.expect_count(6);
    records.measurements.push_back(
        {f.number(1), f.integer(2), f.integer(3), {f.number(4), f.number(5)}});
  } else if (type == "track") {
    f.expect_count(8);
    records.tracks.push_back({f.number(1),
                              f.integer(2),
                              f.integer(3),
                              {f.number(4), f.number(5)},
                              {f.number(6), f.number(7)}});
  } else {
    fail(line, "unknown record type '" + std::string(type) + "'");
  }
}

}

void write_log(const std::filesystem::path& path, const LogRecords& records) {
  const std::string text = format_log(records);
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    throw std::runtime_error("cannot open " + path.string() + " for writing");
  }
  file.write(text.data(), static_cast<std::streamsize>(text.size()));
  file.close();
  if (!file) {
    throw std::runtime_error("failed to write " + path.string());
  }
}

LogRecords read_log(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("cannot open " + path.string() + " for reading");
  }
  std::string text;
  if (!std::getline(file, text)) {
    fail(1, "missing header '" + std::string(kHeader) + "'");
  }
  if (text != kHeader) {
    fail(1, "expected header '" + std::string(kHeader) + "'");
  }
  if (file.eof()) {
    fail(1, "line does not end with a newline");
  }
  LogRecords records;
  std::size_t line = 1;
  while (std::getline(file, text)) {
    ++line;
    if (file.eof()) {
      fail(line, "line does not end with a newline");
    }
    parse_record(text, line, records);
  }
  if (!file.eof()) {
    throw std::runtime_error("failed to read " + path.string());
  }
  return records;
}

}
