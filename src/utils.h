#pragma once

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

inline std::string convertToSnakeCase(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::replace(str.begin(), str.end(), ' ', '_');
  return str;
}

inline std::string generateDeviceId() {
  uint64_t chipId = ESP.getEfuseMac(); // Get chip ID
  std::stringstream ss;
  ss << "lut_" << std::hex << std::setfill('0') << std::setw(8)
     << (chipId & 0xFFFFFF); // Take last 8 digits
  return ss.str();
}