#pragma once

#include <charconv>
#include <cstdint>
#include <string>

namespace kite::tool_format {

inline std::string format_int(std::int64_t value) {
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  return std::string(buffer, result.ptr);
}

}  // namespace kite::tool_format
