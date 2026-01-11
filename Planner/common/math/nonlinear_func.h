#ifndef NONLINEAR_FUNC_H_
#define NONLINEAR_FUNC_H_

#include <cmath>
#include <stdexcept>

namespace common {
namespace math {
// inline double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

inline double IncreasePiecewiseFunc(const double x, const double a = 0.2, const double b = 0.6,
                                    const double k = 10.0, const double C = 4.0,
                                    const double alpha = 5.0) {
    /* a : 死区结束点
     * b : 线性段与指数段分界点
     * k : 线性段斜率
     * C : 指数段初始值系数
     * alpha :指数段放大/缩放因子
     * */

    if (x < a) {    // 1. 死区
        return 0.0;
    } else if (x < b) {    // 2. 线性段  f(x) = k * (x - a)
        return k * (x - a);
    } else {    // 3. 指数段  f(x) = C * exp(alpha * (x - b))
        return C * std::exp(alpha * (x - b));
    }
}

inline double PiecewiseFuncOne(const double x, const double kInitialHighValue = 10.0,
                               const double kDeadZoneValue = 0.0, const double kSeg1End = 1.0,
                               const double kSeg2End = 2.0, const double kSeg3End = 3.0,
                               const double kSeg4End = 4.0) {
    /* kInitialHighValue : 起始的较大值
     * kDeadZoneValue : 死区恒值
     * kSeg1End : 段 1 到 段 2 的分界
     * kSeg2End : 段 2 到 段 3 的分界
     * kSeg3End : 段 3 到 段 4 的分界
     * kSeg4End : 段 4 到 段 5 的分界
     * */

    if (x < 0.0) {
        return 0.0;
    }

    // 段 1
    if (x < kSeg1End) {
        double t = (x - 0.0) / (kSeg1End - 0.0);
        return kInitialHighValue * (1.0 - t);
    }

    //  段 2: 死区
    if (x < kSeg2End) {
        return kDeadZoneValue;
    }

    // 段 3
    if (x < kSeg3End) {
        double t = (x - kSeg2End) / (kSeg3End - kSeg2End);
        return kInitialHighValue * t;
    }

    // 段 4
    if (x < kSeg4End) {
        double t = (x - kSeg3End) / (kSeg4End - kSeg3End);
        return kInitialHighValue * (1.0 - t);
    }

    // 段 5
    return kDeadZoneValue;
}

inline double PiecewiseFuncTwo(const double x) {
    // 1) 分段边界
    const double kSeg0 = 0.0;    // x < 0
    const double kSeg1 = 0.2;    // 0 <= x < 0.2
    const double kSeg2 = 0.4;    // 0.2 <= x < 0.4
    const double kSeg3 = 0.6;    // 0.4 <= x < 0.6
                                 // x >= 0.6

    // 2) 第一段：指数衰减的起止值
    //    f(0) = 10000, f(0.2) = 100
    const double kStartExpValue = 10000.0;
    const double kEndExpValue = 100.0;
    // 通过公式：start * exp(kExpRate1 * 0.2) = end
    // => exp(0.2 * kExpRate1) = end/start = 100/10000 = 0.01
    // => 0.2 * kExpRate1 = ln(0.01) => kExpRate1 = ln(0.01)/0.2
    const double kExpRate1 = -23.0;
    // 约为 -23.02585093

    // 3) 第二段：线性下降 (0.2 ~ 0.4)
    //    从 100 -> 0
    const double kLinearDownStart = 100.0;    // f(0.2)
    const double kLinearDownEnd = 0.0;        // f(0.4)
    const double slopeDown = (kLinearDownEnd - kLinearDownStart) / (kSeg2 - kSeg1);
    // slopeDown = (0 - 100) / (0.4 - 0.2) = -500

    // 4) 第三段：线性上升 (0.4 ~ 0.6)
    //    从 0 -> 100
    const double kLinearUpStart = 0.0;    // f(0.4)
    const double kLinearUpEnd = 100.0;    // f(0.6)
    const double slopeUp = (kLinearUpEnd - kLinearUpStart) / (kSeg3 - kSeg2);
    // slopeUp = (100 - 0) / (0.6 - 0.4) = 500

    // 5) 第四段：缓慢指数衰减 (>= 0.6)
    //    从 100 -> 0
    //    设 f(0.6) = 100, 用 f(x)=100 * exp(-alpha*(x-0.6)), alpha>0
    const double alpha = 1.0;

    // (1) x < 0
    if (x < 0.0) {
        return 0.0;
    }

    // (2) [0, 0.2)
    if (x < kSeg1) {
        // f(x) = 10000 * exp(kExpRate1 * x)
        return kStartExpValue * std::exp(kExpRate1 * (x - 0.0));
    }

    // (3) [0.2, 0.4)
    if (x < kSeg2) {
        // 线性下降
        return kLinearDownStart + slopeDown * (x - kSeg1);
    }

    // (4) [0.4, 0.6)
    if (x < kSeg3) {
        // 线性上升
        return kLinearUpStart + slopeUp * (x - kSeg2);
    }

    // (5) [0.6, +∞)
    // 缓慢指数衰减到 0
    return kLinearUpEnd * std::exp(-alpha * (x - kSeg3));
}

inline double LinearDecayFunction(double x, double x1 = 0.3, double y1 = 0.5, double x2 = 0.5,
                                  double y2 = 0.4) {
    if (x <= 0) {
        // throw std::invalid_argument("x must be non-negative.");
        return 1e8;
    }
    if (x <= x1) {
        return 1.0 + (y1 - 1.0) * (x / x1);
    } else if (x <= x2) {
        return y1 + (y2 - y1) * ((x - x1) / (x2 - x1));
    } else {
        return 0.0;
    }
}

inline double LinearDecayTwoFunction(double x, double x1 = 15, double x2 = 30,
                                     double initial_value = 10.0) {
    if (x < 0) {
        return initial_value;
    }

    if (x <= x1) {
        return initial_value;
    } else if (x <= x2) {
        return initial_value - (initial_value * ((x - x1) / (x2 - x1)));
    } else {
        return 0.0;
    }
}

inline double DecayLinearFunction(double x, double x1 = 1.0) {
    if (x < 0) {
        throw std::invalid_argument("x must be greater than or equal to 0");
    }

    double y;
    if (x < x1) {
        y = 1 - x / x1;
    } else {
        y = 0;
    }

    return y;
}

}    // namespace math
}    // namespace common

#endif