#pragma once

#include "common/math/aabox2d.h"
#include "common/math/polygon2d.h"
#include "common/my_env.h"
#include "trajectory_nlp.h"
#include "trajectory_nlp_config.h"

namespace trajectory_nlp {

using common::math::AABox2d;
using common::math::Box2d;
using common::math::Polygon2d;

class LogNlp {
   public:
    LogNlp() { env_ = std::make_shared<Environment>(); }
    States initial_guess_;
    std::vector<std::array<double, 4>> corridor_bound_;
    common::data::TrajectoryPoint start_var_;
    std::vector<common::math::Vec2d> target_;
    States nlp_result_;
    std::vector<Vec2d> center_line_;
    common::data::TrajectoryPoint start_traj_point_;
    common::data::Trajectory ref_line_;

    const Env &env() const { return env_; }

    void Save(const std::string &env_file) const;

    bool Read(const std::string &env_file);

   private:
    Env env_;
};

class TrajectoryOptimizer {
   public:
    TrajectoryOptimizer(const TrajectoryNLPConfig &config, Env env);

    bool Optimize(const common::data::Trajectory &traj, common::data::Trajectory &result,
                  const common::data::DiscretizedTrajectory &reference_line);

    bool IterationOptimize(const common::data::Trajectory &traj, common::data::Trajectory &result);

    bool IterationOptimize(const common::data::Trajectory &traj, common::data::Trajectory &result,
                           std::vector<double> &cur_infeasibility_vector);

   private:
    TrajectoryNLPConfig config_;
    Env env_;
    VehicleParam vehicle_;
    TrajectoryNLP nlp_;
    u_int16_t infeasibility_num_ = 0;

    LogNlp log_nlp_;

    int fail_count_ = 0;
    int suc_count_ = 0;
    int v_illegal_count_ = 0;

    bool is_first_ = true;

    bool is_nlp_suc_ = false;

    bool is_snopt_suc_ = false;

    States CalculateInitialGuess(const common::data::Trajectory &traj) const;

    bool FormulateCorridorConstraints(const States &states, Constraints &constraints);

    bool GenerateAABoxWithCutoff(double &x, double &y, double radius, AABox2d &box, size_t point_index) const;

    common::data::Trajectory ConvertStatesToTrajectory(const States &states);

    void LogTime(const std::vector<double> time);

    void LogTime(const std::vector<double> time, std::string log_filename);

    bool CheckTrajFeasible(const common::data::Trajectory &result);

    std::vector<double> corridor_time_;
    std::vector<double> nlp_time_, snopt_nlp_time_;
};

inline void to_json(json &j, const States &p) {
    j["x"] = p.x;
    j["y"] = p.y;
    j["theta"] = p.theta;
    j["v"] = p.v;
    j["phi"] = p.phi;
    j["a"] = p.a;
    j["omega"] = p.omega;
}
inline void from_json(const json &j, States &p) {
    p.x = j["x"].get<std::vector<double>>();
    p.y = j["y"].get<std::vector<double>>();
    p.theta = j["theta"].get<std::vector<double>>();
    p.v = j["v"].get<std::vector<double>>();
    p.phi = j["phi"].get<std::vector<double>>();
    p.a = j["a"].get<std::vector<double>>();
    p.omega = j["omega"].get<std::vector<double>>();
}

inline void to_json(json &j, const LogNlp &p) {
    j["initial_guess"] = p.initial_guess_;
    j["corridor_bound"] = p.corridor_bound_;
    j["start_var"] = p.start_var_;
    j["nlp_result"] = p.nlp_result_;
    j["center_line"] = p.center_line_;
    j["five_pointed_star"] = p.target_;
    j["start_traj_point"] = p.start_traj_point_;
    j["ref_line"] = p.ref_line_;
}

inline void from_json(const json &j, LogNlp &p) {
    if (j.contains("initial_guess")) {
        j["initial_guess"].get_to(p.initial_guess_);
    }

    if (j.contains("corridor_bound")) {
        j["corridor_bound"].get_to(p.corridor_bound_);
    }

    if (j.contains("start_var")) {
        j["start_var"].get_to(p.start_var_);
    }

    if (j.contains("nlp_result")) {
        j["nlp_result"].get_to(p.nlp_result_);
    }

    if (j.contains("center_line")) {
        j["center_line"].get_to(p.center_line_);
    }

    if (j.contains("five_pointed_star")) {
        j["target"].get_to(p.target_);
    }

    if (j.contains("start_traj_point")) {
        j["start_traj_point"].get_to(p.start_traj_point_);
    }

    if (j.contains("ref_line")) {
        j["ref_line"].get_to(p.ref_line_);
    }
}

}    // namespace trajectory_nlp
