#pragma once
#include "aabox2d.h"
#include "line_segment2d.h"
#include "polygon2d.h"

namespace common {
namespace math {

class Circle2d {
   public:
    Circle2d() = default;
    Circle2d(double x, double y, double r) : center_(x, y), r_(r), r_square_(r * r) {}

    inline double min_x() const { return center_.x() - r_; }

    inline double max_x() const { return center_.x() + r_; }

    inline double min_y() const { return center_.y() - r_; }

    inline double max_y() const { return center_.y() + r_; }

    inline double radius() const { return r_; }

    inline Vec2d center() const { return center_; }

    inline bool IsPointIn(const Vec2d &point) const {
        return center_.DistanceSquareTo(point) <= r_square_;
    }

    inline bool HasOverlap(const Polygon2d &box) const { return false; }

   private:
    Vec2d center_;
    double r_, r_square_;
};

}    // namespace math
}    // namespace common
