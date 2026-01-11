#include "dp_sv_graph.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "common/math/trajectory1d.h"
#include "common/util/vector.h"

namespace dp_velocity_decider {

DpSVGraph::DpSVGraph(const DpSVGraphConfig &config, Env env) : config_(config), env_(std::move(env)) {}

std::pair<double, double> DpSVGraph::GetCost(const int parent_v_ind, const int s_ind, const int v_ind, const double v_guide,
                                             const bool is_near) {
    double parent_v = s_ind > 0 ? velocity_vec_.at(s_ind).at(parent_v_ind) : start_v_;
    double velocity = velocity_vec_.at(s_ind + 1).at(v_ind);
    double current_time = std::fabs(2.0 * s_resolution_ / (velocity + parent_v));

    double acc = (velocity * velocity - parent_v * parent_v) / (2.0 * s_resolution_);

    // 使用一刀切的权重策略
    double w_velocity = is_near ? config_.w_velocity_near : config_.w_velocity_far;
    double w_acceleration = is_near ? config_.w_acceleration_near : config_.w_acceleration_far;

    // 速度偏差代价
    double cost_v = w_velocity * std::fabs(0.5 * (velocity + parent_v) - v_guide * config_.v_ratio);

    // 简化的加速度代价 - 统一使用软约束，避免因 s 采样密集导致加速度计算过大而剪枝
    double cost_acc = 0.0;
    if ((acc - acc_max_) > kMathEpsilon) {
        // 超过最大加速度，使用线性惩罚
        cost_acc = w_acceleration * (acc - acc_max_) * 100.0;
    } else if ((acc - acc_min_) < -kMathEpsilon) {
        // 超过最大减速度，使用线性惩罚
        cost_acc = w_acceleration * (acc_min_ - acc) * 100.0;
    } else {
        // 在范围内，使用二次惩罚（近距离）或线性惩罚（远距离）
        cost_acc = is_near ? w_acceleration * (acc * acc) : w_acceleration * std::fabs(acc);
    }

    double total_cost = cost_v + cost_acc;

    return std::make_pair(total_cost, current_time);
}

void DpSVGraph::EstimateSpeedLimit(const common::data::DiscretizedTrajectory &reference_line, const common::data::Trajectory &path) {
    int NS = s_list_.size() - 1;
    v_ub_list_.clear();
    v_ub_list_.resize(NS + 1, env_->vehicle.max_velocity);

    // // 循环遍历DP图的每一个s节点，计算该处的速度上限
    // for (int i = 0; i <= NS; ++i) {
    //     double s_cur = s_list_[i];
    //     for (const auto &elem : env_->limit_region) {
    //         // 在高分辨率的原始路径点中，找到与当前s节点最近的点
    //         auto min_elem_it = std::min_element(s_list_from_path_.begin(), s_list_from_path_.end(),
    //                                             [s_cur](double a, double b) { return std::abs(a - s_cur) < std::abs(b - s_cur); });
    //         int ind = std::distance(s_list_from_path_.begin(), min_elem_it);

    //         // 在该最近点进行碰撞检测
    //         auto V_ego = common::math::Box2d(env_->vehicle.GenerateBox(common::math::Pose({path[ind].x, path[ind].y, path[ind].theta})));
    //         if (elem.HasOverlap(V_ego)) {
    //             v_ub_list_[i] = env_->vehicle.max_velocity * config_.v_reduction_rate;
    //         }
    //     }
    // }
}

bool DpSVGraph::Plan(const common::data::DiscretizedTrajectory &reference_line, const common::data::TrajectoryPoint &start,
                     common::data::Trajectory &traj) {
    if (traj.empty()) {
        ROS_ERROR("DpSVGraph::Plan: Input trajectory is empty!");
        return false;
    }

    double sv_start_time = common::util::GetHighCurrentTimestamp();

    acc_max_ = env_->vehicle.max_acceleration * config_.a_ratio;
    acc_min_ = env_->vehicle.max_deceleration * config_.a_ratio;
    start_v_ = start.v;

    // 1. 计算路径的S坐标
    s_list_from_path_.assign(traj.size(), 0.0);
    for (size_t i = 1; i < traj.size(); ++i) {
        auto ds = std::hypot(traj[i].x - traj[i - 1].x, traj[i].y - traj[i - 1].y);
        s_list_from_path_[i] = s_list_from_path_[i - 1] + ds;
    }
    double total_s_horizon = s_list_from_path_.back() - s_list_from_path_.front();

    // 2. 初始化配置参数
    config_.Initialize(total_s_horizon);

    // 3. 根据一刀切策略生成S采样点
    std::vector<double> near_s_vec, far_s_vec;

    // 近距离S采样（密集）
    if (config_.near_s_layers > 0) {
        near_s_vec = common::util::LinSpaced(s_list_from_path_.front(), s_list_from_path_.front() + config_.cutoff_s,
                                             config_.near_s_layers + 1);    // +1 包含端点
    }

    // 远距离S采样（稀疏）
    if (config_.far_s_layers > 0) {
        far_s_vec = common::util::LinSpaced(s_list_from_path_.front() + config_.cutoff_s, s_list_from_path_.back(),
                                            config_.far_s_layers + 1);    // +1 包含端点
    }

    // 合并S采样点
    s_list_ = near_s_vec;
    if (!far_s_vec.empty()) {
        // 跳过第一个元素以避免重复
        s_list_.insert(s_list_.end(), std::next(far_s_vec.begin()), far_s_vec.end());
    }

    int NS = s_list_.size() - 1;    // NS:区间数量

    // 4. 估计速度限制
    states_.clear();
    states_.resize(NS);
    EstimateSpeedLimit(reference_line, traj);

    // 5. 生成速度采样点（根据一刀切策略）
    velocity_vec_.resize(NS + 1);
    for (size_t i = 0; i <= NS; ++i) {
        int current_num_v;
        double current_s = s_list_[i] - s_list_from_path_.front();

        // 判断是否在近距离范围内
        if (current_s <= config_.cutoff_s) {
            current_num_v = config_.near_v_nodes;
        } else {
            current_num_v = config_.far_v_nodes;
        }

        // 根据速度上限生成速度采样点
        velocity_vec_.at(i) = common::util::LinSpaced(config_.v_min, v_ub_list_.at(i), current_num_v - 1);
        velocity_vec_.at(i).push_back(v_ub_list_.at(i) * config_.v_ratio);    // 添加参考速度点

        if (i > 0) {
            states_.at(i - 1).resize(velocity_vec_.at(i).size());
        }
    }

    // 6. 动态规划主循环
    // 初始化第一层
    s_resolution_ = s_list_[1] - s_list_[0];
    bool is_first_near = (s_list_[1] - s_list_from_path_.front()) <= config_.cutoff_s;

    for (int i = 0; i < states_.front().size(); ++i) {
        auto tmp = GetCost(-1, 0, i, v_ub_list_[1], is_first_near);
        states_[0][i].cost = tmp.first;
        states_[0][i].t = tmp.second;
        states_[0][i].s = s_list_[1];
        states_[0][i].v = velocity_vec_.at(1).at(i);
        states_[0][i].parent_v_ind = -1;
    }

    // DP主循环
    for (int i = 0; i < NS - 1; ++i) {
        // 更新当前步长
        s_resolution_ = s_list_[i + 2] - s_list_[i + 1];

        // 判断是否在近距离范围内
        double current_s = s_list_[i + 2] - s_list_from_path_.front();
        bool is_near = (current_s <= config_.cutoff_s);

        for (int j = 0; j < states_.at(i).size(); ++j) {
            if (states_[i][j].cost > config_.dp_cost_max / 2.0) continue;    // 剪枝

            for (int k = 0; k < states_.at(i + 1).size(); ++k) {
                auto tmp = GetCost(j, i + 1, k, v_ub_list_[i + 2], is_near);
                double cur_cost = states_[i][j].cost + tmp.first;

                if (cur_cost < states_[i + 1][k].cost) {
                    states_[i + 1][k].cost = cur_cost;
                    states_[i + 1][k].parent_v_ind = j;
                    states_[i + 1][k].t = states_[i][j].t + tmp.second;
                    states_[i + 1][k].v = velocity_vec_.at(i + 2).at(k);
                    states_[i + 1][k].s = s_list_[i + 2];
                }
            }
        }
    }

    // 7. 回溯最优路径
    int cur_best_v_ind = 0;
    for (int j = 0; j < states_.back().size(); ++j) {
        if (states_[NS - 1][j].cost < states_[NS - 1][cur_best_v_ind].cost && states_[NS - 1][j].t >= config_.tf) {
            cur_best_v_ind = j;
        }
    }

    double best_cost = states_[NS - 1][cur_best_v_ind].cost;

    if (best_cost >= config_.dp_cost_max) {
        return false;
    }
    // // 7. 回溯最优路径 (FIXED)
    // int cur_best_v_ind = -1;
    // double min_cost = std::numeric_limits<double>::max();

    // // 步骤 A: 严格寻找满足所有条件的最优解
    // for (int j = 0; j < states_.back().size(); ++j) {
    //     const auto &final_state = states_[NS - 1][j];
    //     // 检查状态是否可达 (cost < max) 且满足时间要求
    //     if (final_state.cost < config_.dp_cost_max && final_state.t >= config_.tf) {
    //         if (final_state.cost < min_cost) {
    //             min_cost = final_state.cost;
    //             cur_best_v_ind = j;
    //         }
    //     }
    // }

    // // 步骤 B: 如果没有找到满足时间要求的解，则进行降级，寻找成本最低的“最短”路径
    // // 这样做可以防止在无法规划全程时，因找不到解而使用一个完全无效的垃圾路径。
    // if (cur_best_v_ind == -1) {
    //     ROS_WARN("DpSVGraph: No speed profile reached the full time horizon tf. Falling back to the minimum cost path.");
    //     min_cost = std::numeric_limits<double>::max();
    //     for (int j = 0; j < states_.back().size(); ++j) {
    //         const auto &final_state = states_[NS - 1][j];
    //         if (final_state.cost < min_cost) {    // 只比较 cost，找到一个最“好”的路径，即使它很短
    //             min_cost = final_state.cost;
    //             cur_best_v_ind = j;
    //         }
    //     }
    // }

    // // 步骤 C: 最终检查，确保找到的解（无论是完整解还是降级解）是有效的
    // if (cur_best_v_ind == -1 || states_[NS - 1][cur_best_v_ind].cost >= config_.dp_cost_max) {
    //     ROS_ERROR("DpSVGraph: Failed to find any valid speed profile after checking all options.");
    //     return false;    // 确实没有找到任何可行的解
    // }

    std::vector<double> speed_s(NS + 1, 0.0), speed_v(NS + 1, 0.0), speed_t(NS + 1, 0.0);
    speed_s[0] = s_list_[0];
    speed_v[0] = start_v_;

    for (int i = NS; i > 0; --i) {
        speed_s[i] = states_[i - 1][cur_best_v_ind].s;
        speed_t[i] = states_[i - 1][cur_best_v_ind].t;
        speed_v[i] = states_[i - 1][cur_best_v_ind].v;
        cur_best_v_ind = states_[i - 1][cur_best_v_ind].parent_v_ind;
    }

    // 8. 生成最终轨迹
    std::vector<double> new_s, new_v, new_t;
    GenerateIG(speed_s, speed_v, speed_t, new_s, new_v, new_t, traj, start);

    double sv_end_time = common::util::GetHighCurrentTimestamp();
    sv_time_.push_back(sv_end_time - sv_start_time);

    return true;
}

int DpSVGraph::FindClosestIndex(const std::vector<double> &vec, double value) {
    auto it = std::min_element(vec.begin(), vec.end(), [value](double a, double b) { return std::abs(a - value) < std::abs(b - value); });
    return std::distance(vec.begin(), it);
}

void DpSVGraph::GenerateIG(const std::vector<double> &old_s, const std::vector<double> &old_v, const std::vector<double> &old_t,
                           std::vector<double> &new_s, std::vector<double> &new_v, std::vector<double> &new_t,
                           common::data::Trajectory &traj, const common::data::TrajectoryPoint &start) {
    std::vector<double> x_new(traj.size(), 0.0), y_new(traj.size(), 0.0);
    for (size_t i = 0; i < traj.size(); ++i) {
        x_new[i] = traj[i].x;
        y_new[i] = traj[i].y;
    }

    // 插值生成高分辨率的速度剖面
    for (size_t i = 1; i < old_t.size(); ++i) {
        double dt = old_t[i] - old_t[i - 1];
        int nfe_local = std::round(fabs(dt) / config_.dt_resolution);
        if (nfe_local < 2) nfe_local = 2;  // 确保至少有2个点

        auto tmp = common::util::LinSpaced(old_t[i - 1], old_t[i], nfe_local);
        new_t.insert(new_t.end(), tmp.begin(), tmp.end() - 1);

        tmp = common::util::LinSpaced(old_s[i - 1], old_s[i], nfe_local);
        new_s.insert(new_s.end(), tmp.begin(), tmp.end() - 1);

        // 对于第一段（从起始点到第一个DP节点），如果起始速度很低，使用物理加速模型而非线性插值
        if (i == 1 && old_v[0] < config_.v_min) {
            // 使用匀加速模型：v = v0 + a*t，其中 a = acc_max_
            double v0 = old_v[0];
            double v1 = old_v[1];
            double dt_segment = old_t[1] - old_t[0];

            for (int k = 0; k < nfe_local - 1; ++k) {
                double t_ratio = static_cast<double>(k) / (nfe_local - 1);
                double t_elapsed = t_ratio * dt_segment;
                // 匀加速：v = v0 + a*t，但不超过目标速度v1
                double v_accel = v0 + acc_max_ * t_elapsed;
                double v_interp = std::min(v_accel, v1);
                new_v.push_back(v_interp);
            }
        } else {
            tmp = common::util::LinSpaced(old_v[i - 1], old_v[i], nfe_local);
            new_v.insert(new_v.end(), tmp.begin(), tmp.end() - 1);
        }
    }

    new_t.push_back(old_t.back());
    new_s.push_back(old_s.back());
    new_v.push_back(old_v.back());

    // 截断到时间范围
    auto tf = config_.tf;
    auto it = std::find_if(new_t.rbegin(), new_t.rend(), [tf](double val) { return val <= tf; });

    size_t tf_threshold_ind;
    if (it == new_t.rend()) {
        // 所有时间都大于 tf，使用第一个点
        tf_threshold_ind = 0;
    } else {
        tf_threshold_ind = std::distance(new_t.begin(), it.base()) - 1;
        // 选择更接近 tf 的索引
        if (tf_threshold_ind > 0 && tf_threshold_ind + 1 < new_t.size()) {
            double diff_cur = std::fabs(new_t[tf_threshold_ind] - tf);
            double diff_next = std::fabs(new_t[tf_threshold_ind + 1] - tf);
            if (diff_next < diff_cur) {
                tf_threshold_ind++;
            }
        }
    }

    // 确保至少有2个点
    tf_threshold_ind = std::max(tf_threshold_ind, static_cast<size_t>(1));
    tf_threshold_ind = std::min(tf_threshold_ind, new_t.size() - 1);

    new_t.resize(tf_threshold_ind + 1);
    new_s.resize(tf_threshold_ind + 1);
    new_v.resize(tf_threshold_ind + 1);

    // 重采样到固定点数
    // 注意：LinSpaced 的第二个参数应该是 new_t.size() - 1，确保索引不越界
    auto index = common::util::LinSpaced(0.0, static_cast<double>(new_t.size() - 1), config_.nfe);
    std::vector<size_t> chosen_ind(config_.nfe);
    for (size_t i{0U}; i < config_.nfe; ++i) {
        size_t idx = static_cast<size_t>(std::round(index[i]));
        // 确保索引不越界
        chosen_ind[i] = std::min(idx, new_t.size() - 1);
    }

    std::vector<double> s_index(config_.nfe), t_index(config_.nfe), v_index(config_.nfe);

    for (int i = 0; i < config_.nfe; ++i) {
        s_index[i] = new_s[chosen_ind[i]];
        t_index[i] = new_t[chosen_ind[i]];
        v_index[i] = new_v[chosen_ind[i]];
    }

    auto init_a = start.a;
    auto init_omega = start.omega;
    auto init_phi = start.phi;

    traj.clear();
    traj.resize(config_.nfe);

    // 生成最终轨迹点
    // 确保 s_index 范围在 s_list_from_path_ 范围内
    double s_min = s_list_from_path_.front();
    double s_max = s_list_from_path_.back();

    for (int i = 0; i < config_.nfe; ++i) {
        double s_cur = s_index[i];
        // 将 s_cur 限制在有效范围内
        s_cur = std::max(s_min, std::min(s_max, s_cur));

        int ind = FindClosestIndex(s_list_from_path_, s_cur);
        // 确保索引不越界
        ind = std::max(0, std::min(ind, static_cast<int>(x_new.size()) - 1));

        traj[i].x = x_new[ind];
        traj[i].y = y_new[ind];
        traj[i].v = v_index[i];
    }

    // 计算航向角
    for (size_t i = 0; i < config_.nfe - 1; ++i) {
        traj[i].theta = i == 0 ? start.theta : atan2(traj[i + 1].y - traj[i].y, traj[i + 1].x - traj[i].x);

        while (i > 0 && traj[i].theta - traj[i - 1].theta > M_PI) {
            traj[i].theta = traj[i].theta - 2 * M_PI;
        }
        while (i > 0 && traj[i].theta - traj[i - 1].theta < -M_PI) {
            traj[i].theta = traj[i].theta + 2 * M_PI;
        }
    }

    traj.back().theta = traj[config_.nfe - 2].theta;

    // 计算加速度和转向角
    traj[0].a = init_a;
    traj[0].phi = init_phi;
    traj[0].omega = init_omega;
    double dt = config_.tf / (config_.nfe - 1);

    for (int i = 1; i < config_.nfe - 1; ++i) {
        double dv = traj[i + 1].v - traj[i].v;
        traj[i].a = std::min(std::max(dv / dt, acc_min_), acc_max_);
        traj[i].phi = std::atan((traj[i + 1].theta - traj[i].theta) * env_->vehicle.wheel_base / (dt * traj[i].v));
        traj[i].phi = std::min(std::max(traj[i].phi, env_->vehicle.min_phi), env_->vehicle.max_phi);
    }

    traj.back().a = 0.0;
    traj.back().phi = 0.0;

    for (int i = 1; i < config_.nfe - 1; i++) {
        double dt = t_index[i + 1] - t_index[i];
        double dphi = traj[i + 1].phi - traj[i].phi;
        traj[i].omega = std::min(std::max(dphi / dt, -env_->vehicle.max_omega), env_->vehicle.max_omega);
    }

    traj.back().omega = 0.0;
}

}    // namespace dp_velocity_decider