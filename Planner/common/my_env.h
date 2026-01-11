#pragma once
#include <deque>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

#include "common/data/discretized_trajectory.h"
#include "common/environment.h"
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/obstacle.h"
#include "visualization_plot.h"

using json = nlohmann::json;

namespace common {
namespace math {

inline void to_json(json& j, const Vec2d& p) { j = json{p.x(), p.y()}; }
inline void from_json(const json& j, Vec2d& p) {
    p.set_x(j[0]);
    p.set_y(j[1]);
}

inline void to_json(json& j, const Pose& p) { j = json{p.x(), p.y(), p.theta()}; }
inline void from_json(const json& j, Pose& p) {
    p.set_x(j[0]);
    p.set_y(j[1]);
    p.set_theta(j[2]);
}

inline void to_json(json& j, const Polygon2d& p) { j = p.points(); }
inline void from_json(const json& j, Polygon2d& p) { p = Polygon2d(j.get<std::vector<Vec2d>>()); }

}    // namespace math
}    // namespace common

namespace common {
namespace data {
inline void to_json(json& j, const common::data::TrajectoryPoint& p) {
    j["x"] = p.x;
    j["y"] = p.y;
    j["theta"] = p.theta;
    j["v"] = p.v;
    j["phi"] = p.phi;
    j["a"] = p.a;
    j["omega"] = p.omega;
    j["s"] = p.s;
}
inline void from_json(const json& j, common::data::TrajectoryPoint& p) {
    p.x = j["x"];
    p.y = j["y"];
    p.theta = j["theta"];
    p.v = j["v"];
    p.phi = j["phi"];
    p.a = j["a"];
    p.omega = j["omega"];
    p.s = j["s"];
}

}    // namespace data
}    // namespace common

namespace common {
inline void to_json(json& j, const Obstacle& ob) { j = ob.GetPolygon().points(); }
inline void from_json(const json& j, Obstacle& ob) {
    ob = Obstacle(Polygon2d(j.get<std::vector<Vec2d>>()));
}
}    // namespace common

struct MyEnvironment {
   public:
    std::vector<common::math::Vec2d> points;
    std::vector<common::math::Vec2d> inner_points;
    std::vector<common::math::Vec2d> outer_points;
    std::vector<common::Obstacle> obstacles;
    std::vector<DynamicAABB> obstacles_struct;    // 用于存储动态障碍物的结构体
    common::data::Trajectory road;

    int all_obstacles_num;

    MyEnvironment() { env_ = std::make_shared<Environment>(); }

    MyEnvironment(const std::vector<common::math::Vec2d>& points,
                  const std::vector<common::Obstacle>& obstacles)
        : points(points), obstacles(obstacles) {
        env_ = std::make_shared<Environment>();
    }

    const Env& env() const { return env_; }

    bool Read(const std::string& env_file);

    void Save(const std::string& env_file) const;

    void Visualize() const {
#ifndef PLAY_BAG
        for (int i = 0; i < obstacles.size(); ++i) {
            VisualizationPlot::PlotPolygon(obstacles_struct[i].dynamic_aabb, 0.01, Color::Grey, i,
                                           "Obstacles");
            // if (!obstacles_struct.empty()) {
            //     VisualizationPlot::PlotBox(obstacles_struct[i].obs_box, Color::Grey, i,
            //                                "Obstacles fill");
            // }
            VisualizationPlot::PlotFilledPolygon(obstacles_struct[i].dynamic_aabb, Color::Grey, i,
                                                 "Obstacles fill");
        }
        // if (all_obstacles_num < obstacles.size()) {
        //     VisualizationPlot::PlotPolygon(obstacles[8].GetPolygon(), 0.01, Color::Grey, 8,
        //                                    "Obstacles");
        //     // VisualizationPlot::PlotBox(common::math::Box2d(common::math::AABox2d(
        //     //                                obstacles[8].GetPolygon().GetAllVertices())),
        //     //                            Color::Grey, 8, "vehicle fill");
        // } else {
        //     VisualizationPlot::PlotPolygon(obstacles.front().GetPolygon(), 0.00, Color::Grey, 8,
        //                                    "Obstacles");
        //     // VisualizationPlot::PlotBox(common::math::Box2d(common::math::AABox2d(
        //     //                                obstacles[8].GetPolygon().GetAllVertices())),
        //     //                            Color::Grey, 8, "vehicle fill");
        // }

#endif

        // std::vector<double> xs, ys;
        // if (!points.empty()) {
        //     for (auto& i : points) {
        //         xs.push_back(i.x());
        //         ys.push_back(i.y());
        //     }
        //     VisualizationPlot::PlotPoints(xs, ys, 0.01, Color::CreamLike, 1, "bounds");
        // }

        if (!inner_points.empty()) {
            std::vector<double> xs, ys;
            for (const auto& p : inner_points) {
                xs.push_back(p.x());
                ys.push_back(p.y());
            }
            // Use the same glossy effect for the inner boundary
            // VisualizationPlot::PlotGradientLine(xs, ys, 0.01, Color::CreamLike,
            //                                     Color(0.7, 0.7, 1.0, 0.5), 2, "road_boundaries");

            VisualizationPlot::PlotGradientLine(xs, ys, 0.025, Color::White, Color::Yellow, 2,
                                                "road_boundaries");
        }

        if (!outer_points.empty()) {
            std::vector<double> xs, ys;
            for (const auto& p : outer_points) {
                xs.push_back(p.x());
                ys.push_back(p.y());
            }
            // Use a bright, shiny color that fades to a darker, semi-transparent shade
            VisualizationPlot::PlotGradientLine(xs, ys, 0.025, Color::White, Color::Yellow, 1,
                                                "road_boundaries");
        } else {
            ROS_WARN("No outer points available for visualization.");
        }

        VisualizationPlot::Trigger();
    }

   private:
    Env env_;
};

inline void to_json(json& j, const MyEnvironment& p) {
    j["border"] = p.points;
    j["obstacles"] = p.obstacles;
    j["road"] = p.road;
}

inline void from_json(const json& j, MyEnvironment& p) {
    if (j.contains("border")) {
        j["border"].get_to(p.points);
    }

    if (j.contains("obstacles")) {
        j["obstacles"].get_to(p.obstacles);
    }

    j["road"].get_to(p.road);
}
