#include "trajectory_nlp.h"

#include "common/util/time.h"

namespace trajectory_nlp {

TrajectoryNLP::TrajectoryNLP(const TrajectoryNLPConfig &config, Env env) : config_(config), env_(std::move(env)) {
    // 初始化配置
    config_.Initialize();

    nlp_first_config_ = {{"ipopt", Dict({{"linear_solver", "ma27"}, {"print_level", 3}, {"max_wall_time", 0.070}})}};
    nlp_config_ = {{"ipopt", Dict({{"linear_solver", "ma27"}, {"print_level", 3}, {"max_wall_time", 0.070}})}};

    BuildCommon();
}

void TrajectoryNLP::BuildCommon() {
    // 定义优化变量
    var_x_ = SX::sym("x", config_.nfe);
    var_y_ = SX::sym("y", config_.nfe);
    var_theta_ = SX::sym("theta", config_.nfe);
    var_v_ = SX::sym("v", config_.nfe);
    var_phi_ = SX::sym("phi", config_.nfe);
    var_a_ = SX::sym("a", config_.nfe);
    var_omega_ = SX::sym("omega", config_.nfe);
    opti_var_ = SX::vertcat({var_x_, var_y_, var_theta_, var_v_, var_phi_, var_a_, var_omega_});

    // 参考轨迹
    x_ref_ = SX::sym("x_ref", config_.nfe);
    y_ref_ = SX::sym("y_ref", config_.nfe);
    theta_ref_ = SX::sym("theta_ref", config_.nfe);
    v_ref_ = SX::sym("v_ref", config_.nfe);

    // 权重向量（一刀切版本）
    weight_a_vector_ = SX::sym("weight_a_vector", config_.nfe);
    weight_w_vector_ = SX::sym("weight_w_vector", config_.nfe);
    weight_theta_vector_ = SX::sym("weight_theta_vector", config_.nfe);
    weight_xy_vector_ = SX::sym("weight_xy_vector", config_.nfe);
    weight_v_vector_ = SX::sym("weight_v_vector", config_.nfe);
    p_inf_w_vector_ = SX::sym("p_inf_w_vector", (config_.nfe - 1) * config_.var_num);

    // 目标函数
    objective_ = sum1(weight_a_vector_ * sq(var_a_)) + sum1(weight_w_vector_ * sq(var_omega_)) +
                 sum1(weight_xy_vector_ * (sq(var_x_ - x_ref_) + sq(var_y_ - y_ref_))) +
                 sum1(weight_theta_vector_ * sq(var_theta_ - theta_ref_)) + sum1(weight_v_vector_ * sq(var_v_ - v_ref_));
    // objective_ = sum1(weight_a_vector_ * sq(var_a_)) + sum1(weight_w_vector_ * sq(var_omega_)) +
    //              sum1(weight_xy_vector_ * (sq(var_x_ - x_ref_) + sq(var_y_ - y_ref_))) +
    //              sum1(weight_theta_vector_ * sq(var_theta_ - theta_ref_)) ;

    // 运动学约束
    double hi = config_.tf / (config_.nfe - 1);
    Slice prev = Slice(0, config_.nfe - 1);
    Slice next = Slice(1, config_.nfe);

    SX g_x_kin = var_x_(next) - (var_x_(prev) + hi * var_v_(prev) * cos(var_theta_(prev)));
    SX g_y_kin = var_y_(next) - (var_y_(prev) + hi * var_v_(prev) * sin(var_theta_(prev)));
    SX g_theta_kin = var_theta_(next) - (var_theta_(prev) + hi * var_v_(prev) * tan(var_phi_(prev)) / env_->vehicle.wheel_base);
    SX g_v_kin = var_v_(next) - (var_v_(prev) + hi * var_a_(prev));
    SX g_phi_kin = var_phi_(next) - (var_phi_(prev) + hi * var_omega_(prev));

    // 中心点约束
    SX xc = var_x_(next) + env_->vehicle.c2x * cos(var_theta_(next));
    SX yc = var_y_(next) + env_->vehicle.c2x * sin(var_theta_(next));

    var_xc_ = SX::sym("xc", config_.nfe - 1);
    var_yc_ = SX::sym("yc", config_.nfe - 1);
    iterative_var_ = SX::vertcat({opti_var_, var_xc_, var_yc_});

    g_kin_ = SX::vertcat({g_x_kin, g_y_kin, g_theta_kin, g_v_kin, g_phi_kin});
    g_iterative_kin_ = SX::vertcat({g_kin_, var_xc_ - xc, var_yc_ - yc});

    iterative_constraint_evaluator_ = Function("iter_constraint", {opti_var_, var_xc_, var_yc_}, {g_iterative_kin_}, {});

    p_inf_w_ = SX::sym("inf_w");

    auto infeasibility = sumsqr(g_iterative_kin_);
    infeasibility_vector_ = SX::sym("infeasibility_vector", config_.nfe - 1);

    for (int elem_index = 0; elem_index < config_.nfe - 1; ++elem_index) {
        infeasibility_vector_(elem_index) = 0;
        for (int var_index = 0; var_index < config_.var_num; ++var_index) {
            int i = var_index * (config_.nfe - 1) + elem_index;
            infeasibility_vector_(elem_index) += pow(g_iterative_kin_(i), 2);
        }
    }

    iterative_objective_ = objective_ + sum1(p_inf_w_vector_ * sq(g_iterative_kin_));
    infeasibility_evaluator_ = Function("inf", {iterative_var_}, {infeasibility, infeasibility_vector_}, {});
    g_corridor_ = SX::vertcat({xc, yc});

    auto nlp_solver = BuildNLP();
    first_solver_ = nlp_solver.first;
    solver_ = nlp_solver.second;

    auto iter_nlp_solver = BuildIterativeNLP();
    iter_first_solver_ = iter_nlp_solver.first;
    iter_solver_ = iter_nlp_solver.second;

    // kinematic_constraint_evaluator_ =
    //     Function("kin_constraint", {var_x_, var_y_, var_theta_, var_v_, var_phi_, var_a_, var_omega_}, {g_kin_}, {});
}

std::pair<Function, Function> TrajectoryNLP::BuildNLP() {
    SX p = SX::vertcat({x_ref_, y_ref_, theta_ref_, v_ref_, weight_a_vector_, weight_w_vector_, weight_theta_vector_, weight_xy_vector_,
                        weight_v_vector_});
    SX g = SX::vertcat({g_kin_, g_corridor_});

    SXDict nlp = {{"x", opti_var_}, {"p", p}, {"f", objective_}, {"g", g}};

    return {nlpsol("solver", "ipopt", nlp, nlp_first_config_), nlpsol("solver", "ipopt", nlp, nlp_config_)};
}

std::pair<Function, Function> TrajectoryNLP::BuildIterativeNLP() {
    SX p = SX::vertcat({p_inf_w_, x_ref_, y_ref_, theta_ref_, v_ref_, p_inf_w_vector_, weight_a_vector_, weight_w_vector_,
                        weight_theta_vector_, weight_xy_vector_, weight_v_vector_});
    SXDict nlp = {{"x", iterative_var_}, {"p", p}, {"f", iterative_objective_}};

    return {nlpsol("iterative_solver", "ipopt", nlp, nlp_first_config_), nlpsol("iterative_solver", "ipopt", nlp, nlp_config_)};
}

std::pair<DM, DM> TrajectoryNLP::GetVariableBounds(const Constraints &constraints) const {
    auto identity = DM::ones(config_.nfe, 1);

    DM lb_x, lb_y, lb_theta, lb_v, lb_phi, lb_a, lb_omega;
    DM ub_x, ub_y, ub_theta, ub_v, ub_phi, ub_a, ub_omega;

    auto bounds = env_->points.bounds();
    lb_x = bounds[0] * identity, ub_x = bounds[1] * identity;
    lb_y = bounds[2] * identity, ub_y = bounds[3] * identity;
    lb_theta = -inf * identity, ub_theta = inf * identity;
    lb_v = 0 * identity, ub_v = env_->vehicle.max_velocity * identity;
    lb_phi = env_->vehicle.min_phi * identity, ub_phi = env_->vehicle.max_phi * identity;
    lb_a = env_->vehicle.max_deceleration * identity, ub_a = env_->vehicle.max_acceleration * identity;
    lb_omega = -env_->vehicle.max_omega * identity, ub_omega = env_->vehicle.max_omega * identity;

    int end = config_.nfe - 1;
    lb_x(0) = ub_x(0) = constraints.start.x;
    lb_y(0) = ub_y(0) = constraints.start.y;
    lb_theta(0) = ub_theta(0) = constraints.start.theta;
    lb_v(0) = ub_v(0) = constraints.start.v;
    lb_phi(0) = ub_phi(0) = constraints.start.phi;

    lb_a(end) = ub_a(end) = 0.0;
    lb_omega(end) = ub_omega(end) = 0.0;

    DM lbx = DM::vertcat({lb_x, lb_y, lb_theta, lb_v, lb_phi, lb_a, lb_omega});
    DM ubx = DM::vertcat({ub_x, ub_y, ub_theta, ub_v, ub_phi, ub_a, ub_omega});
    return std::make_pair(lbx, ubx);
}

std::pair<DM, DM> TrajectoryNLP::GetCorridorBounds(const Constraints &profile) const {
    DM lb_xc, lb_yc, ub_xc, ub_yc;
    lb_xc = lb_yc = -DM::inf(config_.nfe - 1, 1);
    ub_xc = ub_yc = DM::inf(config_.nfe - 1, 1);

    for (int i = 1; i < config_.nfe - 1; ++i) {
        lb_xc(i) = profile.center_bound[i][0];
        ub_xc(i) = profile.center_bound[i][1];
        lb_yc(i) = profile.center_bound[i][2];
        ub_yc(i) = profile.center_bound[i][3];
    }

    return std::make_pair(DM::vertcat({lb_xc, lb_yc}), DM::vertcat({ub_xc, ub_yc}));
}

States TrajectoryNLP::GetStatesFromSolution(const DM &solution) const {
    States result;
    result.x.resize(config_.nfe);
    result.y.resize(config_.nfe);
    result.theta.resize(config_.nfe);
    result.v.resize(config_.nfe);
    result.phi.resize(config_.nfe);
    result.a.resize(config_.nfe);
    result.omega.resize(config_.nfe);
    result.t.resize(config_.nfe);
    for (size_t i = 0; i < config_.nfe; i++) {
        result.x[i] = double(solution(i, 0));
        result.y[i] = double(solution(config_.nfe + i, 0));
        result.theta[i] = double(solution(2 * config_.nfe + i, 0));
        result.v[i] = double(solution(3 * config_.nfe + i, 0));
        result.phi[i] = double(solution(4 * config_.nfe + i, 0));
        result.a[i] = double(solution(5 * config_.nfe + i, 0));
        result.omega[i] = double(solution(6 * config_.nfe + i, 0));
        result.t[i] = config_.tf / (config_.nfe - 1) * i;
    }
    return result;
}

bool TrajectoryNLP::Solve(const Constraints &profile, const States &guess, bool is_first, States &result) {
    if (is_first) {
        use_solver_ = first_solver_;
    } else {
        use_solver_ = solver_;
    }

    // 获取变量边界
    DM eq_bound = DM::zeros((config_.nfe - 1) * 5);
    std::pair<DM, DM> var_bounds = GetVariableBounds(profile);
    std::pair<DM, DM> corridor_bounds = GetCorridorBounds(profile);

    // 生成一刀切权重向量
    auto weight_a_vector = GenerationWeightNfe(config_.w_opti_a_near, config_.w_opti_a_far);
    auto weight_w_vector = GenerationWeightNfe(config_.w_opti_omega_near, config_.w_opti_omega_far);
    auto weight_theta_vector = GenerationWeightNfe(config_.w_opti_track_theta_near, config_.w_opti_track_theta_far);
    auto weight_xy_vector = GenerationWeightNfe(config_.w_opti_track_near, config_.w_opti_track_far);
    auto weight_v_vector = GenerationWeightNfe(config_.w_opti_track_v_near, config_.w_opti_track_v_far);

    DMDict arg, res;
    arg["lbx"] = var_bounds.first;
    arg["ubx"] = var_bounds.second;
    arg["lbg"] = DM::vertcat({eq_bound, corridor_bounds.first});
    arg["ubg"] = DM::vertcat({eq_bound, corridor_bounds.second});
    arg["x0"] = DM::vertcat({guess.x, guess.y, guess.theta, guess.v, guess.phi, guess.a, guess.omega});
    arg["p"] = DM::vertcat(
        {guess.x, guess.y, guess.theta, guess.v, weight_a_vector, weight_w_vector, weight_theta_vector, weight_xy_vector, weight_v_vector});

    res = use_solver_(arg);

    bool is_success = use_solver_.stats()["success"];
    double final_cost = -1;
    if (is_success) {
        final_cost = static_cast<double>(res.at("f"));
    }
    env_->nlp_cost = final_cost;

    DM opt = res.at("x");
    result = GetStatesFromSolution(opt);

    return is_success;
}

bool TrajectoryNLP::SolveIteratively(const Constraints &profile, const States &guess, States &result, bool is_first, double w_inf_base,
                                     double &infeasibility, std::vector<double> &infeasibility_vector) {
    auto var_bounds = GetVariableBounds(profile);
    auto corridor_bounds = GetCorridorBounds(profile);

    std::vector<double> xc(config_.nfe - 1), yc(config_.nfe - 1);
    for (int i = 0; i < config_.nfe - 1; ++i) {
        std::tie(xc[i], yc[i]) = env_->vehicle.GetDiscPositions(guess.x[i + 1], guess.y[i + 1], guess.theta[i + 1]);
    }

    // 生成一刀切权重向量
    auto p_inf_w_vector =
        GenerationWeightLIOM(config_.infeasibility_weight_near * w_inf_base, config_.infeasibility_weight_far * w_inf_base);
    auto weight_a_vector = GenerationWeightNfe(config_.w_opti_a_near, config_.w_opti_a_far);
    auto weight_w_vector = GenerationWeightNfe(config_.w_opti_omega_near, config_.w_opti_omega_far);
    auto weight_theta_vector = GenerationWeightNfe(config_.w_opti_track_theta_near, config_.w_opti_track_theta_far);
    auto weight_xy_vector = GenerationWeightNfe(config_.w_opti_track_near, config_.w_opti_track_far);
    auto weight_v_vector = GenerationWeightNfe(config_.w_opti_track_v_near, config_.w_opti_track_v_far);

    DMDict arg, res;
    arg["lbx"] = DM::vertcat({var_bounds.first, corridor_bounds.first});
    arg["ubx"] = DM::vertcat({var_bounds.second, corridor_bounds.second});
    arg["x0"] = DM::vertcat({guess.x, guess.y, guess.theta, guess.v, guess.phi, guess.a, guess.omega, xc, yc});
    arg["p"] = DM::vertcat({w_inf_base, guess.x, guess.y, guess.theta, guess.v, p_inf_w_vector, weight_a_vector, weight_w_vector,
                            weight_theta_vector, weight_xy_vector, weight_v_vector});

    if (is_first) {
        use_solver_ = iter_first_solver_;
    } else {
        use_solver_ = iter_solver_;
    }
    res = use_solver_(arg);

    bool is_success = use_solver_.stats()["success"];
    double final_cost = -1;
    if (is_success) {
        final_cost = static_cast<double>(res.at("f"));
    }
    env_->liom_cost = final_cost;

    DM opt = res.at("x");
    result = GetStatesFromSolution(opt);

    // 使用外部函数计算不可行性（只针对一刀切前的部分）
    infeasibility = CalculateInfeasibilityBeforeCutoff(result);

    // 计算完整的不可行性向量
    infeasibility_vector = CalculateInfeasibilityVector(result);

    return is_success;
}

std::vector<double> TrajectoryNLP::GenerationWeightNfe(double weight_near, double weight_far) {
    std::vector<double> penalty_weights(config_.nfe, 0);

    // 一刀切策略：前cutoff_index个点使用近距离权重，其余使用远距离权重
    for (int i = 0; i < config_.nfe; ++i) {
        if (i < config_.cutoff_index) {
            penalty_weights[i] = weight_near;
        } else {
            penalty_weights[i] = weight_far;
        }
    }

    // 末端点特殊处理（可选）
    if (config_.nfe > 0) {
        penalty_weights[config_.nfe - 1] *= config_.end_ratio;
    }

    return penalty_weights;
}

std::vector<double> TrajectoryNLP::GenerationWeightLIOM(double weight_near, double weight_far) {
    std::vector<double> penalty_weights(config_.var_num * (config_.nfe - 1), 0);

    for (int var_index = 0; var_index < config_.var_num; ++var_index) {
        for (int elem_index = 0; elem_index < config_.nfe - 1; ++elem_index) {
            int i = var_index * (config_.nfe - 1) + elem_index;

            // 一刀切策略
            if (elem_index < config_.cutoff_index) {
                penalty_weights[i] = weight_near;
            } else {
                penalty_weights[i] = weight_far;
            }
        }
    }

    return penalty_weights;
}

double TrajectoryNLP::CalculateInfeasibilityBeforeCutoff(const States &states) const {
    // 准备输入数据 - 所有7个状态变量
    DM x_val = DM(states.x);
    DM y_val = DM(states.y);
    DM theta_val = DM(states.theta);
    DM v_val = DM(states.v);
    DM phi_val = DM(states.phi);
    DM a_val = DM(states.a);
    DM omega_val = DM(states.omega);

    // 根据最终状态计算 xc 和 yc
    std::vector<double> xc_vec(config_.nfe - 1), yc_vec(config_.nfe - 1);
    for (int i = 0; i < config_.nfe - 1; ++i) {
        std::tie(xc_vec[i], yc_vec[i]) = env_->vehicle.GetDiscPositions(states.x[i + 1], states.y[i + 1], states.theta[i + 1]);
    }
    DM xc_val = DM(xc_vec);
    DM yc_val = DM(yc_vec);

    DM opti_var_val = DM::vertcat({x_val, y_val, theta_val, v_val, phi_val, a_val, omega_val});

    // 调用新的评估器
    std::vector<DM> eval_inputs = {opti_var_val, xc_val, yc_val};
    auto outputs = iterative_constraint_evaluator_(eval_inputs);
    DM g_iter_kin_val = outputs.at(0);

    // 只计算一刀切前的不可行性
    double infeasibility = 0.0;
    int total_constraints = 7;

    // 确保不越界
    int cutoff_constraint_index = std::min(config_.cutoff_index - 1, config_.nfe - 2);
    if (cutoff_constraint_index < 0) cutoff_constraint_index = 0;

    for (int elem_index = 0; elem_index < cutoff_constraint_index; ++elem_index) {
        for (int var_index = 0; var_index < total_constraints; ++var_index) {
            int idx = var_index * (config_.nfe - 1) + elem_index;
            double val = static_cast<double>(g_iter_kin_val(idx));
            infeasibility += val * val;
        }
    }

    return infeasibility;
}

std::vector<double> TrajectoryNLP::CalculateInfeasibilityVector(const States &states) const {
    // 准备所有7个状态变量
    DM x_val = DM(states.x);
    DM y_val = DM(states.y);
    DM theta_val = DM(states.theta);
    DM v_val = DM(states.v);
    DM phi_val = DM(states.phi);
    DM a_val = DM(states.a);
    DM omega_val = DM(states.omega);

    // 根据最终状态计算 xc 和 yc
    std::vector<double> xc_vec(config_.nfe - 1), yc_vec(config_.nfe - 1);
    for (int i = 0; i < config_.nfe - 1; ++i) {
        std::tie(xc_vec[i], yc_vec[i]) = env_->vehicle.GetDiscPositions(states.x[i + 1], states.y[i + 1], states.theta[i + 1]);
    }
    DM xc_val = DM(xc_vec);
    DM yc_val = DM(yc_vec);

    DM opti_var_val = DM::vertcat({x_val, y_val, theta_val, v_val, phi_val, a_val, omega_val});

    // 调用评估器
    std::vector<DM> eval_inputs = {opti_var_val, xc_val, yc_val};
    auto outputs = iterative_constraint_evaluator_(eval_inputs);
    DM g_iter_kin_val = outputs.at(0);

    std::vector<double> infeasibility_vector(config_.nfe - 1, 0.0);
    int total_constraints = 7;

    for (int elem_index = 0; elem_index < config_.nfe - 1; ++elem_index) {
        double elem_inf = 0.0;
        for (int var_index = 0; var_index < total_constraints; ++var_index) {
            int idx = var_index * (config_.nfe - 1) + elem_index;
            double val = static_cast<double>(g_iter_kin_val(idx));
            elem_inf += val * val;
        }
        infeasibility_vector[elem_index] = elem_inf;
    }

    return infeasibility_vector;
}

}    // namespace trajectory_nlp
