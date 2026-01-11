#pragma once
#include <fstream>
#include <string>
#include <vector>

namespace common {
namespace util {

bool ReadCSV(const std::string &path, std::vector<std::vector<double>> &table,
             bool skip_header = false);

bool WriteCSV(const std::string &path, std::vector<std::vector<double>> &table,
              const std::vector<std::string> &header);

}    // namespace util
}    // namespace common
