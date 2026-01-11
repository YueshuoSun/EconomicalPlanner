#include "environment.h"

#include "math/box2d.h"
#include "math/polygon2d.h"
#include "visualization_plot.h"

bool Environment::CheckPoseCollision(const common::math::Pose& pose, double radius_buffer) {
    common::math::Box2d vehicle_footprint = vehicle.GenerateBox(pose);

    if (CheckCollision(vehicle_footprint)) {
        return true;
    }
    return false;
}

bool Environment::CheckAABBCollision(const common::math::Box2d& box) {
    for (size_t i{0U}; i < obs_border_point_x.size(); ++i) {
        if (obs_border_point_x[i] >= box.min_x() && obs_border_point_x[i] <= box.max_x() &&
            obs_border_point_y[i] >= box.min_y() && obs_border_point_y[i] <= box.max_y()) {
            return true;
        }
    }
    return false;
}

bool Environment::CheckCollisionStaticAndDynamic(const common::math::Box2d& box) {
    // is collison with static obstacle
    if (points.CheckInBox(box)) {
        return true;
    }
    return std::any_of(obstacles.begin(), obstacles.end(), [&](common::Obstacle& ob) {
        if (ob.is_virtual()) {
            return false;
        }

        // auto relative_xy = box.center() -
        // ob.GetPolygon().center(); if (hypot(relative_xy.x(),
        // relative_xy.y()) > 15.0) {
        //   return false;
        // }

        return ob.GetPolygon().HasOverlap(box);
    });
}

bool Environment::CheckCollision(const common::math::Box2d& box, const double t) {
    // is collison with static obstacle
    if (points.CheckInBox(box)) {
        return true;
    }

    // is collison with dynamic and static obstacles
    return std::any_of(obstacles.begin(), obstacles.end(),
                       [&](common::Obstacle& ob) { return ob.GetPolygon(t).HasOverlap(box); });
}

bool Environment::CheckCollisionDynamic(const common::math::Box2d& box, const double t,
                                        bool is_plot) {
    // is collison with dynamic obstacles
    return std::any_of(obstacles.begin(), obstacles.end(), [&](common::Obstacle& ob) {
        if (!ob.is_moving()) return false;
        if (is_plot) {
            VisualizationPlot::PlotPolygon(common::math::Polygon2d(box), 0.1, Color::White, 1,
                                           "collision_footprint");
            VisualizationPlot::PlotPolygon(ob.GetPolygon(t), 0.1, Color::Green, 2,
                                           "collision_obstacles");
            VisualizationPlot::Trigger();
        }

        return ob.GetPolygon(t).HasOverlap(box);
    });
}

bool Environment::CheckCollisionStaticAndVirtual(const common::math::Box2d& box) {
    if (points.CheckInBox(box)) {
        return true;
    }
    return std::any_of(obstacles.begin(), obstacles.end(), [&](common::Obstacle& ob) {
        if (ob.is_moving()) {
            return false;
        }
        return ob.GetPolygon().HasOverlap(box);
    });
}

bool Environment::CheckCollision(const common::math::Box2d& box) {
    if (points.CheckInBox(box)) {
        return true;
    }

    return std::any_of(obstacles.begin(), obstacles.end(), [&](common::Obstacle& ob) {
        return ob.GetPolygon().HasOverlap(box);
    });
}

void Environment::GenerateStaticObsPoints() {
    static_obs_points.clear();
    for (auto& ob : obstacles) {
        if (ob.is_moving()) {
            continue;
        }
        for (auto& pt : ob.GetPolygon().points()) {
            static_obs_points.push_back(pt);
        }
    }
    // static_obs_points.insert(static_obs_points.end(), road_bound_pts.points().begin(),
    // road_bound_pts.points().end()); static_obs_points.insert(static_obs_points.end(),
    // left_road_bound_origin_pts.begin(), left_road_bound_origin_pts.end());
    // static_obs_points.insert(static_obs_points.end(), right_road_bound_origin_pts.begin(),
    // right_road_bound_origin_pts.end());

    for (size_t i = 0; i < points.points().size(); i += 2) {
        static_obs_points.push_back(points.points()[i]);
    }
}

void Environment::GetObsXY(std::vector<double>& obs_x, std::vector<double>& obs_y) {
    obs_x_.clear();
    obs_y_.clear();
    if (obs_x_.empty()) {
        for (auto obs : obstacles) {
            auto vx = obs.GetObsX();
            auto vy = obs.GetObsY();
            obs_x_.insert(obs_x_.end(), vx.begin(), vx.end());
            obs_y_.insert(obs_y_.end(), vy.begin(), vy.end());
        }

        for (auto b : points.points()) {
            obs_x_.push_back(b.x());
            obs_y_.push_back(b.y());
        }
    }
    obs_x = std::ref(obs_x_);
    obs_y = std::ref(obs_y_);
}

void Environment::GetObsPoints(std::vector<Vec2d>& obs_points) {
    obs_points_.clear();
    if (obs_points_.empty()) {
        obs_points_ = points.points();
        for (auto obs : obstacles) {
            auto obs_p = obs.GetObsPoints();
            obs_points_.insert(obs_points_.end(), obs_p.begin(), obs_p.end());
        }
    }
    obs_points = std::ref(obs_points_);
    obs_points_sort_.SetPoints(obs_points);
}

void Environment::Visualize() {
    std::vector<double> xs, ys, thetas;
    for (auto& pt : points.points()) {
        xs.push_back(pt.x());
        ys.push_back(pt.y());
    }
    VisualizationPlot::PlotPoints(xs, ys, 0.01, Color::Yellow, 9999, "Border");

    VisualizationPlot::Trigger();
}