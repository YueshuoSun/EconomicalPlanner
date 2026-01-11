#pragma once

#include <algorithm>
#include <casadi/casadi.hpp>

#include "common/data/discretized_trajectory.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/obstacle.h"
#include "common/util/point_cloud.h"
#include "common/vehicle_param.h"

struct DynamicAABB {
    Polygon2d dynamic_aabb;
    common::math::Box2d obs_box;
    bool is_generate;
    bool is_sudden_obs = false;
    double change_time;
    Vec2d center_xy;

    double length = 0.0;     // 障碍物长度
    double width = 0.0;      // 障碍物宽度
    double heading = 0.0;    // 障碍物朝向
    bool is_dynamic = false;

    double radius;

    double vx = 0.0;    // 障碍物速度x分量
    double vy = 0.0;    // 障碍物速度y分量
};

struct DynamicVec2d {
    common::math::Vec2d dynamic_vec2d;
    bool is_generate;
    double change_time;
};

struct Environment {
    double liom_cost;
    double nlp_cost;
    size_t good_index = 0;

    std::vector<common::Obstacle> obstacles;
    std::vector<DynamicAABB> obstacles_struct;
    common::util::PointCloud points;
    VehicleParam vehicle;

    Vec2d target_use;

    std::vector<Vec2d> new_target;
    std::vector<DynamicVec2d> new_target_all;
    std::vector<double> local_target_s_list;
    std::vector<Vec2d> local_target_xy_list;

    std::vector<Vec2d> center_line;

    std::vector<common::data::Trajectory> sample_paths;

    common::data::DiscretizedTrajectory reference;

    std::vector<double> obs_border_point_x, obs_border_point_y;

    int num_of_continuous_failure = 0;

    Polygon2d inner_polygon;
    std::vector<Vec2d> inner_corners;
    std::vector<Vec2d> outer_corners;
    common::util::PointCloud outer_points;

    casadi::DM params_vector_norm;
    casadi::DM sol_x;
    casadi::DM sol_jac_x_p;
    common::data::TrajectoryPoint nlp_start;

    common::data::Trajectory normal_traj;

    int counter = 0;

    // new
    std::vector<Vec2d> static_obs_points;

    void GenerateStaticObsPoints();

    bool CheckPoseCollision(const common::math::Pose &pose, double radius_buffer = 0.0);

    bool CheckCollision(const common::math::Box2d &box);

    bool CheckCollisionStaticAndDynamic(const common::math::Box2d &box);

    bool CheckCollision(const common::math::Box2d &box, const double t);

    bool CheckCollisionDynamic(const common::math::Box2d &box, const double t,
                               bool is_plot = false);

    bool CheckCollisionStaticAndVirtual(const common::math::Box2d &box);

    bool CheckAABBCollision(const common::math::Box2d &box);

    void Visualize();

    void Clear() {
        obstacles.clear();
        points.Clear();
        reference.Clear();
    }

    void GetObsXY(std::vector<double> &obs_x, std::vector<double> &obs_y);

    void GetObsPoints(std::vector<Vec2d> &obs_points);

    std::vector<double> obs_x_, obs_y_;
    std::vector<common::math::Vec2d> obs_points_;
    common::util::PointCloud obs_points_sort_;

    double x_min = std::numeric_limits<double>::min(), y_min = std::numeric_limits<double>::min();
    double x_max = std::numeric_limits<double>::max(), y_max = std::numeric_limits<double>::max();

    std::array<double, 4> xy_bound() { return {x_min, x_max, y_min, y_max}; }
};

using Env = std::shared_ptr<Environment>;
