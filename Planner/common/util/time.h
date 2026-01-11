#pragma once
#include <chrono>

namespace common {
namespace util {

using namespace std::chrono;

inline double GetCurrentTimestamp() {
    return ((double)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() /
            1000);
}

inline double GetHighCurrentTimestamp() {
    return (static_cast<double>(
                duration_cast<microseconds>(system_clock::now().time_since_epoch()).count()) /
            1000000);
}

inline double GetHighestCurrentTimestamp() {
    return (
        static_cast<double>(
            duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count()) /
        1000000000.0);
}

}    // namespace util
}    // namespace common