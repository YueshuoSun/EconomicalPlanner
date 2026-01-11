#pragma once

#include <casadi/casadi.hpp>

#include "common/data/discretized_trajectory.h"
#include "common/math/vec2d.h"
#include "common/my_env.h"
#include "trajectory_nlp_config.h"

namespace trajectory_nlp {

using namespace casadi;
using common::math::Vec2d;

struct States {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> v;
    std::vector<double> phi;
    std::vector<double> a;
    std::vector<double> omega;
    std::vector<double> t;
};

struct Constraints {
    common::data::TrajectoryPoint start;
    std::vector<std::array<double, 4>> center_bound;
};

class TrajectoryNLP {
   public:
    TrajectoryNLP(const TrajectoryNLPConfig &config, Env env);

    bool Solve(const Constraints &profile, const States &guess, bool is_first, States &result);

    bool SolveIteratively(const Constraints &profile, const States &guess, States &result, bool is_first, double w_inf_base,
                          double &infeasibility, std::vector<double> &infeasibility_vector);

    // 外部计算不可行性的方法（只计算一刀切前的部分）
    double CalculateInfeasibilityBeforeCutoff(const States &states) const;

    // 计算完整的不可行性向量
    std::vector<double> CalculateInfeasibilityVector(const States &states) const;

   private:
    TrajectoryNLPConfig config_;
    Env env_;

    // 优化变量
    SX opti_var_;
    SX var_x_, var_y_, var_theta_, var_v_, var_phi_, var_a_, var_omega_;

    // 参考轨迹
    SX x_ref_, y_ref_, theta_ref_, v_ref_;

    // 权重向量
    SX weight_a_vector_, weight_w_vector_, weight_theta_vector_, weight_xy_vector_, weight_v_vector_;
    SX p_inf_w_vector_;

    // 目标函数和约束
    SX objective_;
    SX g_kin_;
    SX g_corridor_;
    SX g_iterative_kin_;
    SX iterative_objective_;
    SX var_xc_, var_yc_;
    SX iterative_var_;
    SX p_inf_w_;
    SX infeasibility_vector_;

    // 求解器
    Function first_solver_, solver_;
    Function iter_first_solver_, iter_solver_;
    Function use_solver_;

    // 评估器函数
    Function infeasibility_evaluator_;    // 内部使用，用于LIOM求解过程
    // Function kinematic_constraint_evaluator_;    // 外部使用，用于计算运动学约束
    Function iterative_constraint_evaluator_;

    // 配置
    Dict nlp_first_config_;
    Dict nlp_config_;

    // 私有方法
    void BuildCommon();
    std::pair<Function, Function> BuildNLP();
    std::pair<Function, Function> BuildIterativeNLP();
    std::pair<DM, DM> GetVariableBounds(const Constraints &constraints) const;
    std::pair<DM, DM> GetCorridorBounds(const Constraints &profile) const;
    States GetStatesFromSolution(const DM &solution) const;
    std::vector<double> GenerationWeightNfe(double weight_near, double weight_far);
    std::vector<double> GenerationWeightLIOM(double weight_near, double weight_far);
};

}    // namespace trajectory_nlp