#pragma once

#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
using std_msgs::ColorRGBA;

namespace common {
namespace util {

class Color {
   public:
    Color() = default;

    Color(float r, float g, float b) : r_(r), g_(g), b_(b), a_(1.0) {}

    Color(float r, float g, float b, float a) : r_(r), g_(g), b_(b), a_(a) {}

    std::string toPlotColor() const {
        char buf[10];
        snprintf(buf, 10, "#%02x%02x%02x", uint8_t(r_ * 255), uint8_t(g_ * 255), uint8_t(b_ * 255));
        return buf;
    }

    Color GenerateRateToRGBSoft(double rate);

    ColorRGBA toColorRGBA() const {
        ColorRGBA color;
        color.r = r_;
        color.g = g_;
        color.b = b_;
        color.a = a_;
        return color;
    }

    static Color Black;
    static Color Grey;
    static Color White;
    static Color Red;
    static Color Green;
    static Color Blue;
    static Color Cyan;
    static Color CreamLike;
    static Color Yellow;
    static Color Magenta;
    static Color Orange;
    static Color Purple;        // 标准紫（冷色系中间色）
    static Color Chartreuse;    // 黄绿色（高可见度）
    static Color NavyBlue;      // 深蓝（暗色系对比）
    static Color Pink;          // 粉红（浅暖色）
    static Color Olive;         // 橄榄绿（独特色调）

    static Color Green_translucent;
    static Color Red_translucent;

    static Color fromRGB(float r, float g, float b) {
        return Color(std::min(r, 1.0f), std::min(g, 1.0f), std::min(b, 1.0f));
    }

    /*
     * H(Hue): 0 - 360 degree (integer)
     * S(Saturation): 0 - 1.00 (double)
     * V(Value): 0 - 1.00 (double)
     */
    static Color fromHSV(int H, double S, double V);

    /*! \brief Convert RGB to HSV color space

    Converts a given set of RGB values `r', `g', `b' into HSV
    coordinates. The input RGB values are in the range [0, 1], and the
    output HSV values are in the ranges h = [0, 360], and s, v = [0,
    1], respectively.

    \param fH Hue component, used as output, range: [0, 360]
    \param fS Hue component, used as output, range: [0, 1]
    \param fV Hue component, used as output, range: [0, 1]

  */
    void toHSV(float& fH, float& fS, float& fV) const;

    inline double r() { return r_; }
    inline double g() { return g_; }
    inline double b() { return b_; }

    void set_a(double alpha) { a_ = alpha; }

   private:
    double r_, g_, b_, a_;
};

}    // namespace util
}    // namespace common
