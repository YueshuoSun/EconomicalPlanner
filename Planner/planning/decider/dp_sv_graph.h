#ifndef DPSVGRAPH_H
#define DPSVGRAPH_H

#include <fstream>
#include <vector>

#include "../../common/math/aabox2d.h"
#include "../../common/math/box2d.h"
#include "../../common/math/vec2d.h"
#include "common/environment.h"
#include "dp_sv_graph_config.h"

namespace dp_velocity_decider {

using common::math::kMathEpsilon;

const double inf_sv = std::numeric_limits<double>::max();

struct SVStateCell {
    int parent_v_ind = -1;
    double cost = inf_sv;
    double v = 0.0;
    double t = 0.0;
    double s = 0.0;
};

class DpSVGraph {
   public:
    DpSVGraph(const DpSVGraphConfig &config, Env env);
    bool Plan(const common::data::DiscretizedTrajectory &reference_line, const common::data::TrajectoryPoint &start,
              common::data::Trajectory &traj);

   private:
    void EstimateSpeedLimit(const common::data::DiscretizedTrajectory &reference_line, const common::data::Trajectory &path);

    std::pair<double, double> GetCost(const int parent_v_ind, const int s_ind, const int v_ind, const double v_guide, const bool is_near);

    void GenerateIG(const std::vector<double> &old_s, const std::vector<double> &old_v, const std::vector<double> &old_t,
                    std::vector<double> &new_s, std::vector<double> &new_v, std::vector<double> &new_t, common::data::Trajectory &traj,
                    const common::data::TrajectoryPoint &start);

    int FindClosestIndex(const std::vector<double> &vec, double value);

    std::vector<double> s_list_;
    std::vector<std::vector<SVStateCell>> states_;

    std::vector<double> v_ub_list_;
    std::vector<std::vector<double>> velocity_vec_;
    std::vector<double> s_list_from_path_;

    double s_resolution_;
    std::vector<double> sv_time_;

    DpSVGraphConfig config_;
    Env env_;
    double start_v_;
    std::vector<double> optimalPath;
    common::data::DiscretizedTrajectory path_discre_;
    common::math::Trajectory1d sv_table_;

    double acc_max_, acc_min_;
};

};    // namespace dp_velocity_decider

#endif    // DPSVGRAPH_H