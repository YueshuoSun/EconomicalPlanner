#include "dp_path_decider.h"

#include <bitset>
#include <fstream>
#include <iostream>
#include <utility>

#include "common/math/polygon2d.h"
#include "common/math/quintic_polynomial_curve1d.h"
#include "common/util/vector.h"

namespace dp_path_decider {
using namespace common::util;
using common::math::Polygon2d;

constexpr double kMathEpsilon = 1e-9;

DpPathDecider::DpPathDecider(const DpDeciderConfig &config, Env env) : env_(std::move(env)), config_(config) {}

void DpPathDecider::Clear() {
    station_.clear();
    lateral_vec_.clear();
    common::data::DiscretizedTrajectory new_line;
    reference_line_ = new_line;
    state_ = StartState();
    state_space_.clear();
}

std::vector<double> DpPathDecider::GetSpecificCost(const std::vector<double> &local_obs_s_list,
                                                   const std::vector<double> &local_obs_suggested_l_list, const double parent_s,
                                                   const double parent_l, const double cur_s, const double cur_l, const bool isFirstSeg,
                                                   const std::vector<double> &last_traj_s, const std::vector<double> &last_traj_l) {
    std::vector<double> cost_res(4, 0.0);
    std::vector<double> s_vec, l_vec;
    auto path =
        GenerateQuinticPolynomialPath(parent_s, parent_l, cur_s, cur_l, s_vec, l_vec, config_.collision_check_resolution, isFirstSeg);

    env_->sample_paths.push_back(path);

    // 判断是否在一刀切距离内
    double path_distance = cur_s - state_.start_s;
    bool is_near = (path_distance <= config_.cutoff_distance);

    // 使用不同的障碍物权重
    double obstacle_weight = is_near ? config_.w_obstacle_near : config_.w_obstacle_far;

    double cost_obstacle = 0.0;

    for (auto &pt : path) {
        auto vec_box = env_->vehicle.GenerateBoxBuffer(common::math::Pose({pt.x, pt.y, pt.theta}));
        for (auto &obstacle : env_->obstacles) {
            if (obstacle.GetPolygon().HasOverlap(vec_box)) {
                cost_res[0] = obstacle_weight;
                return cost_res;
            }
        }
    }
    cost_res[0] = cost_obstacle;

    // 横向变化率代价
    for (size_t i = 1; i < s_vec.size(); ++i) {
        double dl_tmp = l_vec[i] - l_vec[i - 1];
        double ds_tmp = std::max(0.001, std::fabs(s_vec[i] - s_vec[i - 1]));
        cost_res[1] += std::fabs(dl_tmp / ds_tmp);
    }

    // 中心线跟踪代价
    for (size_t i = 0; i < l_vec.size(); ++i) {
        double s_cur = s_vec[i];
        double l_cur = l_vec[i];
        bool is_has_obs = false;
        for (size_t j = 0; j < local_obs_s_list.size(); ++j) {
            double obs_s = local_obs_s_list[j];
            if ((s_cur - obs_s < config_.center_line_obs_range_front && s_cur - obs_s > 0.0) ||
                (obs_s - s_cur < config_.center_line_obs_range_rear && obs_s - s_cur > 0.0)) {
                // double obs_suggested_l = local_obs_suggested_l_list[j];
                // cost_res[2] += (obs_suggested_l - l_cur) * (obs_suggested_l - l_cur);
                is_has_obs = true;
            }
        }
        if (!is_has_obs) {
            cost_res[2] += l_cur * l_cur;    // 二次惩罚
        }
    }

    return cost_res;
}

std::pair<double, double> DpPathDecider::GetCost(const std::vector<double> &local_obs_s_list,
                                                 const std::vector<double> &local_obs_suggested_l_list, const StateIndex parent_ind,
                                                 const StateIndex cur_ind, const bool isFirstSeg, const std::vector<double> &last_traj_s,
                                                 const std::vector<double> &last_traj_l) {
    double parent_s = state_.start_s, grandparent_s = state_.start_s;
    double parent_l = state_.start_l, grandparent_l = state_.start_l;

    if (parent_ind.s >= 0 && parent_ind.s < static_cast<int>(state_space_.size()) && parent_ind.l >= 0 &&
        parent_ind.l < static_cast<int>(state_space_[parent_ind.s].size())) {
        auto &cell = state_space_[parent_ind.s][parent_ind.l];
        int grandparent_s_ind = cell.parent_s_ind;
        int grandparent_l_ind = cell.parent_l_ind;
        parent_s = cell.current_s;

        // 添加边界检查
        if (parent_ind.s < static_cast<int>(lateral_vec_.size()) && parent_ind.l < static_cast<int>(lateral_vec_[parent_ind.s].size())) {
            parent_l = lateral_vec_.at(parent_ind.s).at(parent_ind.l);
        }

        if (parent_ind.s >= 1 && grandparent_s_ind >= 0 && grandparent_l_ind >= 0 &&
            grandparent_s_ind < static_cast<int>(state_space_.size()) &&
            grandparent_l_ind < static_cast<int>(state_space_[grandparent_s_ind].size()) &&
            grandparent_s_ind < static_cast<int>(lateral_vec_.size()) &&
            grandparent_l_ind < static_cast<int>(lateral_vec_[grandparent_s_ind].size())) {
            grandparent_s = state_space_[grandparent_s_ind][grandparent_l_ind].current_s;
            grandparent_l = lateral_vec_.at(grandparent_s_ind).at(grandparent_l_ind);
        }
    }

    double cur_s = state_.start_s + station_[cur_ind.s];
    double cur_l = 0.0;

    // 添加边界检查
    if (cur_ind.s >= 0 && cur_ind.s < static_cast<int>(lateral_vec_.size()) && cur_ind.l >= 0 &&
        cur_ind.l < static_cast<int>(lateral_vec_[cur_ind.s].size())) {
        cur_l = lateral_vec_.at(cur_ind.s).at(cur_ind.l);
    } else {
        // 如果索引越界，返回一个高代价以避免选择此路径
        return std::make_pair(cur_s, 1e9);
    }

    auto cost_tmp = GetSpecificCost(local_obs_s_list, local_obs_suggested_l_list, parent_s, parent_l, cur_s, cur_l, isFirstSeg, last_traj_s,
                                    last_traj_l);
    double cost_obstacle = cost_tmp[0];
    double cost_lateral_change = cost_tmp[1];
    double cost_near_guidance = cost_tmp[2];
    double cost_target = cost_tmp[3];

    double delta_cost = cost_obstacle + config_.w_lateral_change * cost_lateral_change + config_.w_lateral * cost_near_guidance +
                        config_.w_target * cost_target;

    return std::make_pair(cur_s, delta_cost);
}

bool DpPathDecider::Plan(const common::data::DiscretizedTrajectory &reference_line, const common::data::TrajectoryPoint &start_point,
                         const common::data::Trajectory &last_traj, common::data::Trajectory &path) {
    double dp_path_start_time = common::util::GetHighCurrentTimestamp();
    env_->sample_paths.clear();
    Clear();
    reference_line_ = reference_line;
    auto start_proj = reference_line_.GetProjection({start_point.x, start_point.y});

    // 处理障碍物引导线
    std::vector<double> local_obs_s_list;
    std::vector<double> local_obs_suggested_l_list;
    for (auto &obstacle : env_->obstacles) {
        Vec2d cur_obs_sl;
        double cur_obs_s;
        double cur_obs_l;

        cur_obs_sl = reference_line_.GetProjection(obstacle.GetPolygon().center());
        cur_obs_s = cur_obs_sl.x();
        cur_obs_l = cur_obs_sl.y();

        if (std::fabs(cur_obs_l) >= config_.min_s_distance) {
            continue;
        }
        if (cur_obs_l > 0) {
            local_obs_s_list.emplace_back(cur_obs_s);
            local_obs_suggested_l_list.emplace_back(0.5 * (cur_obs_l - config_.road_half_width));
        } else {
            local_obs_s_list.emplace_back(cur_obs_s);
            local_obs_suggested_l_list.emplace_back(0.5 * (cur_obs_l + config_.road_half_width));
        }
    }

    // 生成中心线
    std::vector<Vec2d> center_line_xy;
    for (size_t i = 0; i < reference_line_.data().size(); ++i) {
        bool is_has_obs = false;
        for (size_t j = 0; j < local_obs_s_list.size(); ++j) {
            if ((reference_line_.data().at(i).s - local_obs_s_list.at(j) < config_.center_line_obs_range_front + kMathEpsilon &&
                 reference_line_.data().at(i).s - local_obs_s_list.at(j) > -kMathEpsilon) ||
                (local_obs_s_list.at(j) - reference_line_.data().at(i).s < config_.center_line_obs_range_rear + kMathEpsilon &&
                 local_obs_s_list.at(j) - reference_line_.data().at(i).s > -kMathEpsilon)) {
                double center_line_point_l = local_obs_suggested_l_list.at(j);
                Vec2d center_line_point_xy = reference_line_.ToCartesian(reference_line_.data().at(i).s, center_line_point_l);
                center_line_xy.push_back(center_line_point_xy);
                is_has_obs = true;
                break;
            }
        }
        if (!is_has_obs) {
            Vec2d center_line_point_xy = reference_line_.ToCartesian(reference_line_.data().at(i).s, 0.0);
            center_line_xy.push_back(center_line_point_xy);
        }
    }
    env_->center_line = center_line_xy;

    // 计算规划范围
    horizon_ = env_->vehicle.max_velocity * config_.tf * config_.s_coefficient;

    // 初始化配置参数
    config_.Initialize(horizon_);

    // 根据一刀切策略生成采样点
    // 近距离部分：密集采样
    std::vector<double> near_s_vec;
    if (config_.near_layers > 0) {
        near_s_vec = common::util::LinSpaced(config_.cutoff_distance / config_.near_layers, config_.cutoff_distance, config_.near_layers);
    }

    // 远距离部分：稀疏采样
    std::vector<double> far_s_vec;
    if (config_.far_layers > 0) {
        far_s_vec = common::util::LinSpaced(config_.cutoff_distance + (horizon_ - config_.cutoff_distance) / config_.far_layers, horizon_,
                                            config_.far_layers);
    }

    // 生成横向采样点
    lateral_vec_.clear();

    // 近距离：更多横向采样点
    for (size_t i = 0; i < near_s_vec.size(); ++i) {
        lateral_vec_.push_back(common::util::LinSpaced(-config_.l_deviation, 0.05 + config_.l_deviation, config_.near_nodes_per_layer - 1));
        lateral_vec_.back().push_back(0.0);    // 添加中心线点
    }

    // 远距离：较少横向采样点
    for (size_t i = 0; i < far_s_vec.size(); ++i) {
        lateral_vec_.push_back(common::util::LinSpaced(-config_.l_deviation, 0.05 + config_.l_deviation, config_.far_nodes_per_layer - 1));
        if (config_.far_nodes_per_layer > 1) {
            lateral_vec_.back().push_back(0.0);    // 添加中心线点
        }
    }

    // 合并纵向采样点
    station_ = near_s_vec;
    station_.insert(station_.end(), far_s_vec.begin(), far_s_vec.end());
    NS_ = station_.size();

    // 检查是否有有效的采样层
    if (NS_ == 0 || lateral_vec_.empty()) {
        std::cout << "Error: No valid sampling layers generated!" << std::endl;
        return false;
    }

    // 初始化起始状态
    state_.start_s = start_proj.x();
    state_.start_l = start_proj.y();
    state_.start_theta = start_point.theta;

    auto x_next = start_point.x + config_.initial_next_length * cos(start_point.theta);
    auto y_next = start_point.y + config_.initial_next_length * sin(start_point.theta);
    auto sl = reference_line_.GetProjection(Vec2d(x_next, y_next));
    start_s_ = sl.x();
    init_dl = (sl.y() - state_.start_l) / (sl.x() - state_.start_s);

    // 处理上一帧轨迹
    std::vector<double> last_traj_s(last_traj.size(), Inf), last_traj_l(last_traj.size(), Inf);
    for (size_t i{0U}; i < last_traj.size(); i++) {
        auto sl_tmp = reference_line_.GetProjection({last_traj.at(i).x, last_traj.at(i).y});
        last_traj_s[i] = sl_tmp.x();
        last_traj_l[i] = sl_tmp.y();
    }

    // 初始化状态空间
    state_space_.clear();
    for (int i = 0; i < NS_; ++i) {
        int num_nodes = lateral_vec_[i].size();
        std::vector<StateCell> col_state(num_nodes);
        // 初始化所有状态单元的代价为无穷大
        for (int j = 0; j < num_nodes; ++j) {
            col_state[j].cost = Inf;
            col_state[j].parent_s_ind = -1;
            col_state[j].parent_l_ind = -1;
        }
        state_space_.push_back(col_state);
    }

    // 初始化第一层
    for (size_t i = 0; i < lateral_vec_[0].size(); i++) {
        auto tup = GetCost(local_obs_s_list, local_obs_suggested_l_list, StateIndex(-1, -1), StateIndex(0, i), 1, last_traj_s, last_traj_l);
        state_space_[0][i].current_s = tup.first;
        state_space_[0][i].current_l = lateral_vec_.at(0).at(i);
        state_space_[0][i].cost = tup.second;
        state_space_[0][i].parent_s_ind = -1;
        state_space_[0][i].parent_l_ind = -1;
    }

    // 动态规划主循环
    for (int i = 0; i < NS_ - 1; i++) {
        int cur_layer_nodes = lateral_vec_[i].size();
        int next_layer_nodes = lateral_vec_[i + 1].size();

        for (int j = 0; j < cur_layer_nodes; j++) {
            // 跳过代价为无穷大的节点
            if (state_space_[i][j].cost >= Inf - 1) {
                continue;
            }

            StateIndex parent_ind(i, j);
            for (int k = 0; k < next_layer_nodes; k++) {
                StateIndex current_ind(i + 1, k);
                auto tup = GetCost(local_obs_s_list, local_obs_suggested_l_list, parent_ind, current_ind, 0, last_traj_s, last_traj_l);
                double delta_cost = tup.second;

                // 如果当前段代价过高，跳过
                if (delta_cost >= 1e9 - 1) {
                    continue;
                }

                double cur_s = tup.first;
                double cur_l = lateral_vec_.at(i + 1).at(k);
                double cur_cost = state_space_[i][j].cost + delta_cost;

                if (cur_cost < state_space_[i + 1][k].cost) {
                    state_space_[i + 1][k] = StateCell(cur_cost, cur_s, cur_l, i, j);
                }
            }
        }
    }

    // 找到最优路径
    double min_cost = Inf;
    int min_l_ind = -1;
    int last_layer_nodes = lateral_vec_[NS_ - 1].size();
    for (int i = 0; i < last_layer_nodes; ++i) {
        double cost = state_space_[NS_ - 1][i].cost;
        if (cost < min_cost) {
            min_l_ind = i;
            min_cost = cost;
        }
    }

    // 检查是否找到有效路径
    if (min_l_ind == -1 || min_cost >= Inf - 1) {
        std::cout << "Error: No valid path found in DP planning!" << std::endl;
        return false;
    }

    // 回溯最优路径
    std::vector<std::pair<StateIndex, StateCell>> waypoints(NS_);
    int current_s_ind = NS_ - 1;
    int current_l_ind = min_l_ind;

    for (int i = NS_ - 1; i >= 0; i--) {
        if (current_s_ind < 0 || current_l_ind < 0 || current_s_ind >= static_cast<int>(state_space_.size()) ||
            current_l_ind >= static_cast<int>(state_space_[current_s_ind].size())) {
            std::cout << "Error: Invalid indices during backtracking!" << std::endl;
            return false;
        }

        auto &cell = state_space_[current_s_ind][current_l_ind];
        waypoints[i] = std::make_pair(StateIndex(current_s_ind, current_l_ind), cell);

        // 更新为父节点索引
        int next_s_ind = cell.parent_s_ind;
        int next_l_ind = cell.parent_l_ind;
        current_s_ind = next_s_ind;
        current_l_ind = next_l_ind;
    }

    // 构建路径点
    std::vector<double> s_vector, l_vector;
    s_vector.push_back(state_.start_s);
    l_vector.push_back(state_.start_l);

    for (int i = 0; i < NS_; ++i) {
        double cur_s = state_.start_s + station_[i];
        double cur_l = lateral_vec_.at(i).at(waypoints[i].first.l);
        s_vector.push_back(cur_s);
        l_vector.push_back(cur_l);
    }

    // 重采样生成最终路径
    Resample(start_point, s_vector, l_vector, path);

    double dp_path_end_time = common::util::GetHighCurrentTimestamp();
    sl_time_.push_back(dp_path_end_time - dp_path_start_time);

    return true;
}

void DpPathDecider::Resample(const common::data::TrajectoryPoint &start_point, const std::vector<double> &old_s,
                             const std::vector<double> &old_l, common::data::Trajectory &result) {
    std::vector<double> s_record, l_record;

    for (size_t i{0U}; i < old_s.size() - 1; ++i) {
        size_t num = static_cast<size_t>(std::ceil((old_s[i + 1] - old_s[i]) / config_.resample_resolution));
        if (num == 0) num = 1;    // 确保至少有一个点

        std::vector<double> s_full = common::util::LinSpaced(old_s[i], old_s[i + 1], num);
        std::vector<double> l_full(s_full.size(), Inf);
        auto curve = i == 0
                         ? common::math::QuinticPolynomialCurve1d(old_l[i], init_dl, 0.0, old_l[i + 1], 0.0, 0.0, old_s[i + 1] - old_s[i])
                         : common::math::QuinticPolynomialCurve1d(old_l[i], 0.0, 0.0, old_l[i + 1], 0.0, 0.0, old_s[i + 1] - old_s[i]);
        for (size_t j{0U}; j < l_full.size(); ++j) {
            l_full[j] = curve.Evaluate(0, s_full[j] - s_full[0]);
        }
        s_record.insert(s_record.end(), s_full.begin(), s_full.end() - 1);
        l_record.insert(l_record.end(), l_full.begin(), l_full.end() - 1);
    }
    s_record.push_back(old_s.back());
    l_record.push_back(old_l.back());

    result.clear();
    for (size_t i = 0; i < s_record.size(); ++i) {
        common::data::TrajectoryPoint tp;
        auto xy = reference_line_.ToCartesian(s_record.at(i), l_record.at(i));
        tp.x = xy.x();
        tp.y = xy.y();
        tp.s = s_record.at(i);
        result.push_back(tp);
    }
}

void DpPathDecider::GenerateThetaForXYInDp(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &theta,
                                           const bool isFirstSeg) {
    if (theta.size() == 0) return;

    for (size_t i = 1; i < theta.size(); ++i) {
        theta[i] = atan2(y[i] - y[i - 1], x[i] - x[i - 1]);
    }

    theta[0] = isFirstSeg ? state_.start_theta : theta[1];

    for (size_t i = 1; i < theta.size(); ++i) {
        while (theta[i] - theta[i - 1] > M_PI) {
            theta[i] = theta[i] - 2 * M_PI;
        }
        while (theta[i] - theta[i - 1] < -M_PI) {
            theta[i] = theta[i] + 2 * M_PI;
        }
    }

    for (size_t i = 1; i < theta.size(); ++i) {
        if (abs(theta[i] - theta[i - 1]) > M_PI_4) {
            theta[i] = theta[i - 1];
        }
    }
}

common::data::Trajectory DpPathDecider::GenerateQuinticPolynomialPath(const double parent_s, const double parent_l, const double cur_s,
                                                                      const double cur_l, std::vector<double> &s_full,
                                                                      std::vector<double> &l_full, const double step,
                                                                      const bool isFirstSeg) {
    size_t num = static_cast<size_t>(ceil((cur_s - parent_s) / step));
    if (num == 0) num = 1;    // 确保至少有一个点

    s_full = common::util::LinSpaced(parent_s, cur_s, num);
    l_full.resize(s_full.size(), Inf);
    auto curve = isFirstSeg ? common::math::QuinticPolynomialCurve1d(parent_l, init_dl, 0.0, cur_l, 0.0, 0.0, cur_s - parent_s)
                            : common::math::QuinticPolynomialCurve1d(parent_l, 0.0, 0.0, cur_l, 0.0, 0.0, cur_s - parent_s);
    for (size_t j{0U}; j < l_full.size(); ++j) {
        l_full[j] = curve.Evaluate(0, s_full[j] - s_full[0]);
    }

    std::vector<double> tmp_x(num, Inf), tmp_y(num, Inf);
    for (size_t i = 0; i < num; ++i) {
        auto xy = reference_line_.ToCartesian(s_full.at(i), l_full.at(i));
        tmp_x[i] = xy.x();
        tmp_y[i] = xy.y();
    }

    std::vector<double> theta(num, Inf);
    GenerateThetaForXYInDp(tmp_x, tmp_y, theta, isFirstSeg);

    common::data::Trajectory path;
    for (size_t i = 0; i < num; ++i) {
        path.push_back(common::data::TrajectoryPoint(common::math::Pose({tmp_x[i], tmp_y[i], theta[i]})));
    }
    return path;
}

double DpPathDecider::DotProduct(const Vec2d &p1, const Vec2d &p2, const Vec2d &p3) {
    return (p2.x() - p1.x()) * (p3.x() - p1.x()) + (p2.y() - p1.y()) * (p3.y() - p1.y());
}

bool DpPathDecider::IsInsideRectangle(const std::vector<Vec2d> &ABCD, const Vec2d &M) {
    double AB_AB = DotProduct(ABCD[0], ABCD[1], ABCD[1]);
    double AM_AB = DotProduct(ABCD[0], ABCD[1], M);

    double AD_AD = DotProduct(ABCD[0], ABCD[3], ABCD[3]);
    double AM_AD = DotProduct(ABCD[0], ABCD[3], M);

    return (AM_AB > 0 && AM_AB < AB_AB) && (AM_AD > 0 && AM_AD < AD_AD);
}

}    // namespace dp_path_decider