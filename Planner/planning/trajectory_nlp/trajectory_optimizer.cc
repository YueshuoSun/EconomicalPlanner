#include "trajectory_optimizer.h"

#include <ros/ros.h>

#include <bitset>
#include <filesystem>
#include <nlohmann/json.hpp>

#include "common/math/math_utils.h"
#include "common/math/trajectory1d.h"
#include "common/util/vector.h"
#include "common/visualization_plot.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace trajectory_nlp {

using common::math::Trajectory1d;

TrajectoryOptimizer::TrajectoryOptimizer(const TrajectoryNLPConfig &config, Env env)
    : config_(config), env_(std::move(env)), nlp_(config, env_) {
    vehicle_ = env_->vehicle;
    vehicle_.GenerateDiscs();
    env_->vehicle.GenerateDiscs();
}

bool TrajectoryOptimizer::Optimize(const common::data::Trajectory &traj, common::data::Trajectory &result,
                                   const common::data::DiscretizedTrajectory &reference_line) {
    log_nlp_.target_ = env_->new_target;
    log_nlp_.center_line_ = env_->center_line;
    log_nlp_.ref_line_ = reference_line.data();
    auto guess = CalculateInitialGuess(traj);
    log_nlp_.initial_guess_ = guess;
    States states;
    log_nlp_.nlp_result_ = states;
    Constraints constraints;
    constraints.start = traj.front();
    log_nlp_.start_var_ = constraints.start;

    double start_time = common::util::GetHighCurrentTimestamp();
    if (!FormulateCorridorConstraints(guess, constraints)) {
        return false;
    }
    corridor_time_.push_back(common::util::GetHighCurrentTimestamp() - start_time);

    ROS_INFO("corridor Time: %f", corridor_time_.back());

    double nlp_start_time = common::util::GetHighCurrentTimestamp();
    if (nlp_.Solve(constraints, guess, is_first_, states)) {
        nlp_time_.push_back(common::util::GetHighCurrentTimestamp() - nlp_start_time);
        result = ConvertStatesToTrajectory(states);

        if (is_first_) {
            is_first_ = false;
        }
        return true;
    }

    // 直接调用测试函数
    // bool test_success = nlp_.TestSolve(constraints, guess, states);
    // if (nlp_.TestSolve_With_Penalty(constraints, guess, states)) {
    //     nlp_time_.push_back(common::util::GetHighCurrentTimestamp() - nlp_start_time);
    //     result = ConvertStatesToTrajectory(states);

    //     if (is_first_) {
    //         is_first_ = false;
    //     }
    //     return true;
    // }
    nlp_time_.push_back(common::util::GetHighCurrentTimestamp() - nlp_start_time);

    return false;
}

bool TrajectoryOptimizer::IterationOptimize(const common::data::Trajectory &traj, common::data::Trajectory &result) {
    auto guess = CalculateInitialGuess(traj);
    States states_liom;
    Constraints constraints;
    constraints.start = traj.front();

    double nlp_start_time = common::util::GetHighCurrentTimestamp();

    double previous_infeasibility = inf;
    int iter = 0;
    double weight = 1.0;    // 权重在NLP内部根据一刀切策略调整

    while (iter <= config_.iter_num) {
        double start_time = clock();

        double cur_infeasibility = 0.0;
        std::vector<double> cur_infeasibility_vector;

        if (!FormulateCorridorConstraints(guess, constraints)) {
            ROS_ERROR("Failed to formulate corridor constraints at iteration %d", iter);
            return false;
        }

        bool is_success =
            nlp_.SolveIteratively(constraints, guess, states_liom, is_first_, weight, cur_infeasibility, cur_infeasibility_vector);

        // 输出详细信息
        printf("Optimize [iter=%d][success=%d], weight: %.1e, time: %.6f, infeasibility(before cutoff): %.6e\n", iter, is_success, weight,
               ((double)clock() - start_time) / CLOCKS_PER_SEC, cur_infeasibility);

        if (is_success) {
            // 更新guess为当前解
            guess = states_liom;

            // 检查不可行性是否满足阈值（只检查一刀切前的部分）
            if (cur_infeasibility < config_.infeasibility_threshold) {
                ROS_INFO("Infeasibility below threshold (%.2e < %.2e). Optimum found.", cur_infeasibility, config_.infeasibility_threshold);
                result = ConvertStatesToTrajectory(states_liom);
                is_first_ = false;
                return true;
            }
        } else {
            ROS_WARN("NLP solve failed in iteration %d. Using previous solution.", iter);
            return false;
        }

        // 为下一次迭代准备
        weight *= config_.infeasibility_weight_ratio;
        previous_infeasibility = cur_infeasibility;
        iter++;
    }

    ROS_WARN("Max iterations reached. Returning best found solution with infeasibility: %.6e", previous_infeasibility);
    result = ConvertStatesToTrajectory(guess);
    is_first_ = false;

    return false;
}

bool TrajectoryOptimizer::IterationOptimize(const common::data::Trajectory &traj, common::data::Trajectory &result,
                                            std::vector<double> &cur_infeasibility_vector) {
    auto guess = CalculateInitialGuess(traj);
    States states_liom;
    Constraints constraints;
    constraints.start = traj.front();

    double nlp_start_time = common::util::GetHighCurrentTimestamp();

    double previous_infeasibility = inf;
    int iter = 0;
    double weight = 1.0;    // 权重在NLP内部根据一刀切策略调整

    while (iter <= config_.iter_num) {
        double start_time = clock();

        double cur_infeasibility = 0.0;
        cur_infeasibility_vector.clear();

        if (!FormulateCorridorConstraints(guess, constraints)) {
            ROS_ERROR("Failed to formulate corridor constraints at iteration %d", iter);
            return false;
        }

        bool is_success =
            nlp_.SolveIteratively(constraints, guess, states_liom, is_first_, weight, cur_infeasibility, cur_infeasibility_vector);

        // 输出详细信息
        printf("Optimize [iter=%d][success=%d], weight: %.1e, time: %.6f, infeasibility(before cutoff): %.6e\n", iter, is_success, weight,
               ((double)clock() - start_time) / CLOCKS_PER_SEC, cur_infeasibility);

        if (is_success) {
            // 更新guess为当前解
            guess = states_liom;

            // 检查不可行性是否满足阈值（只检查一刀切前的部分）
            if (cur_infeasibility < config_.infeasibility_threshold) {
                ROS_INFO("Infeasibility below threshold (%.2e < %.2e). Optimum found.", cur_infeasibility, config_.infeasibility_threshold);
                result = ConvertStatesToTrajectory(states_liom);
                is_first_ = false;
                return true;
            }
        } else {
            ROS_WARN("NLP solve failed in iteration %d. Using previous solution.", iter);
            // 如果求解失败，尝试调整权重继续
        }

        // 为下一次迭代准备
        weight *= config_.infeasibility_weight_ratio;
        previous_infeasibility = cur_infeasibility;
        iter++;
    }

    ROS_WARN("Max iterations reached. Returning best found solution with infeasibility: %.6e", previous_infeasibility);
    result = ConvertStatesToTrajectory(guess);
    is_first_ = false;

    // 返回false表示没有完全收敛，但仍然返回了最佳解
    return false;
}

common::data::Trajectory TrajectoryOptimizer::ConvertStatesToTrajectory(const States &states) {
    common::data::Trajectory result;
    result.reserve(states.x.size());
    double dt = config_.tf / (states.x.size() - 1);
    for (int i = 0; i < states.x.size(); ++i) {
        common::data::TrajectoryPoint tp;
        tp.t = i * dt;
        tp.x = states.x[i];
        tp.y = states.y[i];
        tp.theta = states.theta[i];
        tp.v = states.v[i];
        tp.a = states.a[i];
        tp.phi = states.phi[i];
        tp.omega = states.omega[i];
        result.push_back(tp);
    }

    return result;
}

States TrajectoryOptimizer::CalculateInitialGuess(const common::data::Trajectory &traj) const {
    States states;
    states.x.resize(traj.size());
    states.y.resize(traj.size());
    states.theta.resize(traj.size());
    states.v.resize(traj.size());
    states.phi.resize(traj.size());
    states.a.resize(traj.size());
    states.omega.resize(traj.size());

    for (int i = 0; i < traj.size(); ++i) {
        states.x[i] = traj[i].x;
        states.y[i] = traj[i].y;
        states.theta[i] = traj[i].theta;
        states.v[i] = traj[i].v;
        states.phi[i] = traj[i].phi;
        states.a[i] = traj[i].a;
        states.omega[i] = traj[i].omega;
    }
    return states;
}

bool TrajectoryOptimizer::FormulateCorridorConstraints(const States &states, Constraints &constraints) {
    constraints.center_bound.resize(config_.nfe);

    for (size_t i = 1; i < config_.nfe; ++i) {
        double xc, yc;
        std::tie(xc, yc) = vehicle_.GetDiscPositions(states.x[i], states.y[i], states.theta[i]);

        AABox2d center;

        // 根据点的位置使用不同的走廊生成策略
        if (!GenerateAABoxWithCutoff(xc, yc, vehicle_.radius, center, i)) {
            return false;
        }

        constraints.center_bound[i] = {center.min_x(), center.max_x(), center.min_y(), center.max_y()};
    }

    log_nlp_.corridor_bound_ = constraints.center_bound;

    // 可视化走廊
    for (size_t i = 3; i < constraints.center_bound.size(); ++i) {
        Polygon2d box = Polygon2d(Box2d(AABox2d({constraints.center_bound[i][0], constraints.center_bound[i][2]},
                                                {constraints.center_bound[i][1], constraints.center_bound[i][3]})));
        VisualizationPlot::PlotPolygon(box, 0.001, common::util::Color::Blue, i + 100, "Corridor");
    }

    VisualizationPlot::Trigger();
    return true;
}

bool TrajectoryOptimizer::GenerateAABoxWithCutoff(double &x, double &y, double radius, AABox2d &box, size_t point_index) const {
    double ri = radius;
    AABox2d bound({x - ri, y - ri}, {x + ri, y + ri});

    // 根据一刀切策略选择步长
    double step = (point_index < config_.cutoff_index) ? config_.corridor_step_near : config_.corridor_step_far;

    // 第一步：寻找无碰撞的中心点
    if (env_->CheckCollision(Box2d(bound))) {
        int inc = 4;
        double real_x, real_y;
        do {
            int iter = inc / 4;
            uint8_t edge = inc % 4;

            real_x = x;
            real_y = y;
            if (edge == 0) {
                real_x = x - iter * step;
            } else if (edge == 1) {
                real_x = x + iter * step;
            } else if (edge == 2) {
                real_y = y - iter * step;
            } else if (edge == 3) {
                real_y = y + iter * step;
            }

            ++inc;
            bound = AABox2d({real_x - ri, real_y - ri}, {real_x + ri, real_y + ri});
        } while (env_->CheckCollision(Box2d(bound)) && inc < config_.corridor_max_iter);

        if (inc > config_.corridor_max_iter) {
            return false;
        }

        x = real_x;
        y = real_y;
    }

    // 第二步：扩展走廊边界
    int inc = 4;
    std::bitset<4> blocked;
    double incremental[4] = {0.0};

    do {
        int iter = inc / 4;
        uint8_t edge = inc % 4;
        ++inc;

        if (blocked[edge]) continue;

        incremental[edge] = iter * step;

        AABox2d test({x - ri - incremental[0], y - ri - incremental[2]}, {x + ri + incremental[1], y + ri + incremental[3]});

        if (env_->CheckCollision(Box2d(test)) || incremental[edge] >= config_.corridor_incremental_limit) {
            incremental[edge] -= step;
            blocked[edge] = true;
        }
    } while (!blocked.all() && inc < config_.corridor_max_iter);

    if (inc > config_.corridor_max_iter) {
        return false;
    }

    box = {{x - incremental[0], y - incremental[2]}, {x + incremental[1], y + incremental[3]}};
    return true;
}

void TrajectoryOptimizer::LogTime(const std::vector<double> time, std::string log_filename) {
    std::ofstream file;
    file.open(log_filename, std::ios::app);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open file for logging");
        return;
    }
    file << time.back() << "\n";
    file.close();
}

bool TrajectoryOptimizer::CheckTrajFeasible(const common::data::Trajectory &result) {
    for (int i = 0; i < result.size(); ++i) {
        auto g_v_region = env_->vehicle.max_velocity - result.at(i).v;
        if (g_v_region < -0.000001) {
            ROS_ERROR("v limit invalid error is %lf", -g_v_region);
            return false;
        }
    }
    return true;
}    // namespace trajectory_nlp

bool LogNlp::Read(const std::string &env_file) {
    std::ifstream is(env_file, std::ios::binary);
    if (is.is_open()) {
        json j = json::parse(is);
        *this = j;
        return true;
    } else {
        return false;
    }
}

void LogNlp::Save(const std::string &env_file) const {
    std::ofstream os(env_file, std::ios::binary);
    if (os.is_open()) {
        json j = *this;
        os << j;
    }
}

}    // namespace trajectory_nlp