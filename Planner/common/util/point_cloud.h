#pragma once
#include <memory>

#include "common/math/box2d.h"
#include "common/math/circle_2d.h"
#include "common/math/vec2d.h"

namespace common {
namespace util {

using common::math::Box2d;
using common::math::Circle2d;
using common::math::Vec2d;

class PointCloud {
   public:
    inline void Clear() {
        points_.clear();
        max_x_ = std::numeric_limits<double>::max();
        min_x_ = std::numeric_limits<double>::lowest();
        max_y_ = std::numeric_limits<double>::max();
        min_y_ = std::numeric_limits<double>::lowest();
    }

    void AddPoints(const std::vector<Vec2d> &points, double sample_step = 0.5);

    bool CheckInBox(const Box2d &box) const;
    bool CheckInAABB(const common::math::AABox2d &aabb) const;
    bool CheckInCircle(const Circle2d &circle) const;

    const std::vector<Vec2d> &points() const { return points_; }

    void SetPoints(const std::vector<Vec2d> &points);

    inline std::array<double, 4> bounds() { return {min_x_, max_x_, min_y_, max_y_}; }

   private:
    double max_x_ = std::numeric_limits<double>::max();
    double min_x_ = std::numeric_limits<double>::lowest();
    double max_y_ = std::numeric_limits<double>::max();
    double min_y_ = std::numeric_limits<double>::lowest();

    std::vector<Vec2d> points_;
};

}    // namespace util
}    // namespace common