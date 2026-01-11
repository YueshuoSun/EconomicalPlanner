#pragma once
#include <bitset>
#include <cmath>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "common/data/discretized_trajectory.h"
#include "common/environment.h"
#include "common/math/nonlinear_func.h"
#include "common/math/pose.h"
#include "common/math/trajectory1d.h"
#include "common/math/vec2d.h"
#include "common/visualization_plot.h"
#include "dp_path_decider_config.h"

namespace dp_path_decider {
using common::math::Polygon2d;
using common::math::Pose;
using common::math::Vec2d;

const double Inf = std::numeric_limits<double>::max();
const double NInf = std::numeric_limits<double>::min();

class DpPathDecider {
   public:
    DpPathDecider() = default;
    DpPathDecider(const DpDeciderConfig &config, Env env);

    bool Plan(const common::data::DiscretizedTrajectory &reference_line, const common::data::TrajectoryPoint &start_point,
              const common::data::Trajectory &last_traj, common::data::Trajectory &traj);
    void Clear();

    void Resample(const common::data::TrajectoryPoint &start_point, const std::vector<double> &old_s, const std::vector<double> &old_l,
                  common::data::Trajectory &result);

   private:
    struct StateCell {
        double cost = Inf;
        double current_s = NInf;
        double current_l = NInf;
        int parent_s_ind = -1;
        int parent_l_ind = -1;

        StateCell() = default;
        StateCell(double cost, double cur_s, double cur_l, int parent_s_ind, int parent_l_ind)
            : cost(cost), current_s(cur_s), current_l(cur_l), parent_s_ind(parent_s_ind), parent_l_ind(parent_l_ind) {}
    };

    struct StateIndex {
        int s = -1, l = -1;

        StateIndex() = default;
        StateIndex(int ss, int ll) : s(ss), l(ll) {}
    };

    struct StartState {
        double start_s = 0.0;
        double start_l = 0.0;
        double start_theta = 0.0;
    };
    Env env_;
    DpDeciderConfig config_;
    int NS_;
    int old_nl_ = -999;
    int new_nl_ = -999;
    // int NL_;
    std::vector<double> station_;
    // std::vector<double> lateral_;
    common::data::DiscretizedTrajectory reference_line_;
    common::data::DiscretizedTrajectory local_reference_line_;
    std::vector<double> sl_time_;

    double horizon_;
    double start_s_;

    std::vector<double> local_target_s_list_;
    std::vector<Vec2d> local_target_xy_list_;

    StartState state_;
    std::vector<std::vector<StateCell>> state_space_;
    ros::NodeHandle node_handle_;

    double init_dl = 0.0;

    // new
    std::vector<std::vector<double>> lateral_vec_;
    common::math::AABox2d aabb_;
    std::vector<Vec2d> obs_points_;

    std::vector<double> GetSpecificCost(const std::vector<double> &local_obs_s_list, const std::vector<double> &local_obs_suggested_l_list,
                                        const double parent_s, const double parent_l, const double cur_s, const double cur_l,
                                        const bool isFirstSeg, const std::vector<double> &last_traj_s,
                                        const std::vector<double> &last_traj_l);

    std::pair<double, double> GetCost(const std::vector<double> &local_obs_s_list, const std::vector<double> &local_obs_suggested_l_list,
                                      const StateIndex parent_ind, const StateIndex cur_ind, const bool isFirstSeg,
                                      const std::vector<double> &last_traj_s, const std::vector<double> &last_traj_l);

    void GenerateThetaForXYInDp(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &theta,
                                const bool isFirstSeg);

    common::data::Trajectory GenerateQuinticPolynomialPath(const double parent_s, const double parent_l, const double cur_s,
                                                           const double cur_l, std::vector<double> &s_vec, std::vector<double> &l_vec,
                                                           const double step, const bool isFirstSeg);

    double DotProduct(const Vec2d &p1, const Vec2d &p2, const Vec2d &p3);

    bool IsInsideRectangle(const std::vector<Vec2d> &ABCD, const Vec2d &M);
};

}    // namespace dp_path_decider