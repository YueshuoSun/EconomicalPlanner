#pragma once
#include "common/data/discretized_trajectory.h"
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/util/time.h"

namespace common {

class Obstacle {
   public:
    Obstacle() = default;
    Obstacle(const math::Polygon2d &polygon, std::vector<math::PoseStamped> &trajectory)
        : polygon_(polygon), trajectory_(trajectory) {}
    explicit Obstacle(const math::Polygon2d &polygon) : polygon_(polygon) { GetPoints(); }
    explicit Obstacle(const math::Polygon2d &polygon, const bool &is_move)
        : polygon_(polygon), is_move_(is_move) {}
    bool is_moving() const { return !trajectory_.empty() || is_move_; }
    double generate_time() const { return generate_time_; }

    inline void GetPoints() {
        int num = polygon_.num_points();
        obs_points_ = polygon_.points();

        point_x.resize(num);
        point_y.resize(num);
        for (size_t i{0U}; i < num; ++i) {
            point_x[i] = obs_points_[i].x();
            point_y[i] = obs_points_[i].y();
        }
        if (hypot(point_x[0] - point_x[1], point_y[0] - point_y[1]) < obs_sample_step) {
            return;
        } else {
            for (size_t i = 0; i < num - 1; ++i) {
                std::vector<common::math::Vec2d> obs_line_point =
                    common::math::LineSegment2d(obs_points_[i], obs_points_[i + 1])
                        .ObsSamplePoints(obs_sample_step);
                for (size_t i = 0; i < obs_line_point.size(); ++i) {
                    point_x.push_back(obs_line_point.at(i).x());
                    point_y.push_back(obs_line_point.at(i).y());
                    obs_points_.emplace_back(obs_line_point.at(i));
                }
            }
            std::vector<common::math::Vec2d> obs_line_point =
                common::math::LineSegment2d(obs_points_[num - 1], obs_points_[0])
                    .ObsSamplePoints(obs_sample_step);
            for (size_t i = 0; i < obs_line_point.size(); ++i) {
                point_x.push_back(obs_line_point.at(i).x());
                point_y.push_back(obs_line_point.at(i).y());
                obs_points_.emplace_back(obs_line_point.at(i));
            }
        }
    }

    inline std::vector<double> GetObsX() { return point_x; }
    inline std::vector<double> GetObsY() { return point_y; }
    inline std::vector<common::math::Vec2d> GetObsPoints() { return obs_points_; }

    math::Polygon2d GetPolygon(double relative_time = 0.0) const;
    math::Pose GetVecPose(double relative_time = 0.0) const;

    void CalculateSLBound(const common::data::DiscretizedTrajectory &ref_line) {
        if (ref_line.data().empty()) {
            return;
        }
        if (is_move_) {
            return;
        }

        start_s_ = DBL_MAX, end_s_ = -DBL_MAX;
        start_l_ = DBL_MAX, end_l_ = -DBL_MAX;
        for (auto &pt : polygon_.points()) {
            auto proj = ref_line.GetProjection(pt);
            if (proj.x() > end_s_) end_s_ = proj.x();
            if (proj.x() < start_s_) start_s_ = proj.x();
            if (proj.y() > end_l_) end_l_ = proj.y();
            if (proj.y() < start_l_) start_l_ = proj.y();
        }
    }

    double start_s() { return start_s_; }
    double end_s() { return end_s_; }
    double start_l() { return start_l_; }
    double end_l() { return end_l_; }

    void set_st_start_s(double s) { st_start_s_ = s; }
    void set_st_end_s(double s) { st_end_s_ = s; }
    void set_st_start_t(double t) { st_start_t_ = t; }
    void set_st_end_t(double t) { st_end_t_ = t; }

    double st_start_s() { return st_start_s_; }
    double st_end_s() { return st_end_s_; }
    double st_start_t() { return st_start_t_; }
    double st_end_t() { return st_end_t_; }

    bool IsOverLapST() { return FLAGS_is_overlap_st_; }
    void set_flags_is_overlap_st(bool flag) { FLAGS_is_overlap_st_ = flag; }

    bool is_virtual() { return is_virtual_; }
    void set_virtual(bool flag) { is_virtual_ = flag; }

    void set_v(double v) { v_ = v; }

   private:
    math::Polygon2d polygon_;
    std::vector<math::PoseStamped> trajectory_;
    double v_ = 0.0;
    bool is_move_ = false;
    double generate_time_ = common::util::GetCurrentTimestamp();
    double start_s_, end_s_, start_l_, end_l_;
    double st_start_s_, st_end_s_, st_start_t_, st_end_t_;

    std::vector<double> point_x;
    std::vector<double> point_y;
    std::vector<common::math::Vec2d> obs_points_;

    double obs_sample_step = 0.01;

    bool FLAGS_is_overlap_st_ = false;
    bool is_virtual_ = false;
};

}    // namespace common