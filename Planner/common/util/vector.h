#pragma once
#include <array>
#include <vector>

namespace common {
namespace util {

template <int N>
inline std::array<double, N> LinSpaced(double start, double end) {
    std::array<double, N> res;
    double step = (end - start) / (N - 1);

    for (int i = 0; i < N; i++) {
        res[i] = start + step * i;
    }

    return res;
}

inline std::vector<double> LinSpaced(double start, double end, int count) {
    if (count <= 0) {
        std::vector<double> res(1, 0.0);
        res[0] = start;
        return res;
    }

    std::vector<double> res(count, 0);
    double step = (end - start) / (count - 1);

    for (int i = 0; i < count; i++) {
        res[i] = start + step * i;
    }

    return res;
}

inline std::vector<double> ARange(double start, double end, double step) {
    std::vector<double> res;

    for (int i = 0;; i++) {
        auto val = start + step * i;
        if (val > end) {
            break;
        }
        res.push_back(val);
    }

    return res;
}

}    // namespace util
}    // namespace common
