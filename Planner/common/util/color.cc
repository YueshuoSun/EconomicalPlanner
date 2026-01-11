#include "common/util/color.h"

#include <cmath>

namespace common {
namespace util {

Color Color::Black(0.0, 0.0, 0.0);
Color Color::Grey(0.7, 0.7, 0.7);
Color Color::White(1.0, 1.0, 1.0);
Color Color::Red(1.0, 0.0, 0.0);
Color Color::Green(0.0, 1.0, 0.0);
Color Color::Blue(0.0, 0.0, 1.0);
Color Color::Cyan(0.0, 1.0, 1.0);
Color Color::CreamLike(1.0, 1.0, 0.8);
Color Color::Yellow(1.0, 1.0, 0.0);
Color Color::Magenta(1.0, 0.0, 1.0);
Color Color::Orange(1.0, 0.647, 0.0);

Color Color::Green_translucent(0.0, 1.0, 0.0, 0.10);
Color Color::Red_translucent(1.0, 0.0, 0.0, 0.10);

Color Color::Purple(0.5, 0.0, 0.5);        // 标准紫（冷色系中间色）
Color Color::Chartreuse(0.5, 1.0, 0.0);    // 黄绿色（高可见度）
Color Color::NavyBlue(0.0, 0.0, 0.5);      // 深蓝（暗色系对比）
Color Color::Pink(1.0, 0.75, 0.8);         // 粉红（浅暖色）
Color Color::Olive(0.5, 0.5, 0.0);         // 橄榄绿（独特色调）

Color Color::fromHSV(int H, double S, double V) {
    double C = S * V;
    double X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
    double m = V - C;
    double Rs, Gs, Bs;

    if (H >= 0 && H < 60) {
        Rs = C;
        Gs = X;
        Bs = 0;
    } else if (H >= 60 && H < 120) {
        Rs = X;
        Gs = C;
        Bs = 0;
    } else if (H >= 120 && H < 180) {
        Rs = 0;
        Gs = C;
        Bs = X;
    } else if (H >= 180 && H < 240) {
        Rs = 0;
        Gs = X;
        Bs = C;
    } else if (H >= 240 && H < 300) {
        Rs = X;
        Gs = 0;
        Bs = C;
    } else {
        Rs = C;
        Gs = 0;
        Bs = X;
    }

    return Color(Rs + m, Gs + m, Bs + m);
}

void Color::toHSV(float &fH, float &fS, float &fV) const {
    float fCMax = std::max(std::max(r_, g_), b_);
    float fCMin = std::min(std::min(r_, g_), b_);
    float fDelta = fCMax - fCMin;

    if (fDelta > 0) {
        if (fCMax == r_) {
            fH = 60 * (std::fmod(((g_ - b_) / fDelta), 6));
        } else if (fCMax == g_) {
            fH = 60 * (((b_ - r_) / fDelta) + 2);
        } else if (fCMax == b_) {
            fH = 60 * (((r_ - g_) / fDelta) + 4);
        }

        if (fCMax > 0) {
            fS = fDelta / fCMax;
        } else {
            fS = 0;
        }

        fV = fCMax;
    } else {
        fH = 0;
        fS = 0;
        fV = fCMax;
    }

    if (fH < 0) {
        fH = 360 + fH;
    }
}

Color Color::GenerateRateToRGBSoft(double rate) {
    rate = std::max(0.0, std::min(1.0, rate));

    std::vector<Color> colors = {
        {237.0 / 255.0, 28.0 / 255.0, 36.0 / 255.0},     // 柔和的红
        {255.0 / 255.0, 127.0 / 255.0, 39.0 / 255.0},    // 柔和的橙
        {255.0 / 255.0, 242.0 / 255.0, 0.0 / 255.0},     // 柔和的黄
        {34.0 / 255.0, 177.0 / 255.0, 76.0 / 255.0},     // 柔和的绿
        {0.0 / 255.0, 162.0 / 255.0, 232.0 / 255.0},     // 柔和的青
        {63.0 / 255.0, 72.0 / 255.0, 204.0 / 255.0},     // 柔和的蓝
        {163.0 / 255.0, 73.0 / 255.0, 164.0 / 255.0}     // 柔和的紫
    };

    int numColors = colors.size();
    double segmentLength = 1.0 / (numColors - 1);

    int segmentIndex = static_cast<int>(rate / segmentLength);
    if (segmentIndex >= numColors - 1) {
        segmentIndex = numColors - 2;
    }

    double localRate = (rate - segmentIndex * segmentLength) / segmentLength;

    Color color1 = colors[segmentIndex];
    Color color2 = colors[segmentIndex + 1];
    Color rgb;
    rgb.r_ = color1.r_ + (color2.r_ - color1.r_) * localRate;
    rgb.g_ = color1.g_ + (color2.g_ - color1.g_) * localRate;
    rgb.b_ = color1.b_ + (color2.b_ - color1.b_) * localRate;
    rgb.a_ = 1.0;
    return rgb;
}

}    // namespace util
}    // namespace common
