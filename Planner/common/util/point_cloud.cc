#include "point_cloud.h"

#include <algorithm>
#include <iostream>

#include "common/math/line_segment2d.h"

namespace common {
namespace util {

void PointCloud::AddPoints(const std::vector<Vec2d> &points, double sample_step) {
    for (size_t i = 0; i < points.size() - 1; i++) {
        size_t next_index = i + 1;
        double distance =
            hypot(points[next_index].x() - points[i].x(), points[next_index].y() - points[i].y());

        if (distance < sample_step) {
            points_.push_back(points[i]);
        } else {
            auto pts = common::math::LineSegment2d(points[i], points[next_index])
                           .SamplePoints(sample_step);
            for (auto &pt : pts) {
                points_.push_back(pt);
            }
        }
    }
    SetPoints(points_);
}

void PointCloud::SetPoints(const std::vector<Vec2d> &points) {
    points_ = points;

    std::sort(points_.begin(), points_.end(),
              [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });

    min_x_ = points_.front().x();
    max_x_ = points_.back().x();

    min_y_ = max_y_ = points_.front().y();
    for (auto &pt : points_) {
        min_y_ = std::min(pt.y(), min_y_);
        max_y_ = std::max(pt.y(), max_y_);
    }
}

bool PointCloud::CheckInBox(const Box2d &box) const {
    if (points_.empty()) {
        return false;
    }
    if (box.max_x() < points_.front().x() || box.min_x() > points_.back().x()) {
        return false;
    }

    auto comp = [](double val, const Vec2d &a) { return val < a.x(); };

    auto check_start = std::upper_bound(points_.begin(), points_.end(), box.min_x(), comp);
    auto check_end = std::upper_bound(points_.begin(), points_.end(), box.max_x(), comp);

    std::advance(check_start, -1);

    for (auto iter = check_start; iter != check_end; iter++) {
        if (box.IsPointIn(*iter)) {
            return true;
        }
    }

    return false;
}

bool PointCloud::CheckInAABB(const common::math::AABox2d &aabb) const {
    if (points_.empty()) {
        return false;
    }
    if (aabb.max_x() < points_.front().x() || aabb.min_x() > points_.back().x()) {
        return false;
    }

    auto comp = [](double val, const Vec2d &a) { return val < a.x(); };

    auto check_start = std::upper_bound(points_.begin(), points_.end(), aabb.min_x(), comp);
    auto check_end = std::upper_bound(points_.begin(), points_.end(), aabb.max_x(), comp);

    std::advance(check_start, -1);

    for (auto iter = check_start; iter != check_end; iter++) {
        if (aabb.IsPointIn(*iter)) {
            return true;
        }
    }
    return false;
}

bool PointCloud::CheckInCircle(const Circle2d &circle) const {
    if (points_.empty()) {
        return false;
    }
    if (circle.max_x() < points_.front().x() || circle.min_x() > points_.back().x()) {
        return false;
    }

    auto comp = [](double val, const Vec2d &a) { return val < a.x(); };

    auto check_start = std::upper_bound(points_.begin(), points_.end(), circle.min_x(), comp);
    auto check_end = std::upper_bound(points_.begin(), points_.end(), circle.max_x(), comp);

    std::advance(check_start, -1);

    for (auto iter = check_start; iter != check_end; iter++) {
        if (circle.IsPointIn(*iter)) {
            return true;
        }
    }

    return false;
}

}    // namespace util
}    // namespace common
