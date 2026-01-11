#include "interface.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <filesystem>
#include <fstream>

#include "common/math/vec2d.h"
#include "common/util/time.h"
#include "common/visualization_plot.h"

namespace fs = std::filesystem;

Planner::Planner() : nh_("~/planning"), rd_(), gen_(rd_()) {
    std::string home_dir = std::getenv("HOME");
    std::string dir_path = home_dir + "/logs";
    log_filename_ = dir_path + "/time.csv";
    log_time_other_filename_ = dir_path + "/other_time.csv";
    log_cost_filename_ = dir_path + "/cost.csv";
    log_timecost_filename_ = dir_path + "/time_cost.csv";

    if (!fs::exists(dir_path)) {
        fs::create_directories(dir_path);
    }
    if (fs::exists(log_filename_)) {
        fs::remove(log_filename_);
    }

    VisualizationPlot::Init(nh_, "world", "markers");
    env_ = std::make_shared<Environment>();
    ReadConfig();
    nh_.param("min_obs_length", min_obs_length_, 0.8);      // 最小长度，默认0.8米
    nh_.param("max_obs_length", max_obs_length_, 1.5);      // 最大长度，默认1.5米
    nh_.param("min_obs_width", min_obs_width_, 0.6);        // 最小宽度，默认0.6米
    nh_.param("max_obs_width", max_obs_width_, 1.0);        // 最大宽度，默认1.0米
    nh_.param("random_seed", (int&)random_seed_, 12345);    // 随机数种子，默认12345

    // 使用种子初始化随机数引擎
    random_generator_.seed(random_seed_);
    ROS_INFO("Planner's obstacle generator initialized with random seed: %u", random_seed_);

    // 初始化随机数分布范围
    length_dist_ = std::uniform_real_distribution<double>(min_obs_length_, max_obs_length_);
    width_dist_ = std::uniform_real_distribution<double>(min_obs_width_, max_obs_width_);

    double max_heading_deviation_deg;
    nh_.param("max_heading_deviation_deg", max_heading_deviation_deg, 15.0);    // 默认最大偏移±15度

    // 将角度转换为弧度，并初始化分布
    double max_heading_deviation_rad = max_heading_deviation_deg * M_PI / 180.0;
    heading_deviation_dist_ = std::uniform_real_distribution<double>(-max_heading_deviation_rad, max_heading_deviation_rad);

    ROS_INFO("Obstacle heading will be randomized with max deviation: +/- %.2f degrees", max_heading_deviation_deg);

    // 定义中间安全走廊和道路边界
    double lateral_dead_zone_min, lateral_safe_zone_max;
    nh_.param("lateral_dead_zone_min", lateral_dead_zone_min, 0.3);    // 中间安全走廊的半宽，即l_min
    nh_.param("lateral_safe_zone_max", lateral_safe_zone_max, 1.2);    // 道路可通行区域的半宽，即l_max

    // 校验参数合法性
    if (lateral_dead_zone_min < 0 || lateral_safe_zone_max < 0 || lateral_dead_zone_min >= lateral_safe_zone_max) {
        ROS_WARN("Lateral offset parameters are invalid. Reverting to defaults [0.3, 1.2].");
        lateral_dead_zone_min = 0.3;
        lateral_safe_zone_max = 1.2;
    }

    // 初始化三个分布
    side_chooser_dist_ = std::uniform_int_distribution<int>(0, 1);    // 0代表左侧, 1代表右侧
    left_lateral_dist_ = std::uniform_real_distribution<double>(-lateral_safe_zone_max, -lateral_dead_zone_min);
    right_lateral_dist_ = std::uniform_real_distribution<double>(lateral_dead_zone_min, lateral_safe_zone_max);

    ROS_INFO("Obstacle lateral offset 'l' will be randomized in ranges [%.2f, %.2f] U [%.2f, %.2f]", -lateral_safe_zone_max,
             -lateral_dead_zone_min, lateral_dead_zone_min, lateral_safe_zone_max);
    dp_path_decider_ = std::make_shared<dp_path_decider::DpPathDecider>(dp_path_config_, env_);
    dp_sv_graph_ = std::make_shared<dp_velocity_decider::DpSVGraph>(dp_speed_config_, env_);
    traj_opti_ = std::make_shared<trajectory_nlp::TrajectoryOptimizer>(traj_nlp_config_, env_);

    color_ = std::make_shared<common::util::Color>();
}

void Planner::ReadConfig() {
    // Vehicle Parameters
    nh_.param("rear_hang", env_->vehicle.rear_hang, -999.0);
    nh_.param("front_hang", env_->vehicle.front_hang, -999.0);
    nh_.param("wheel_base", env_->vehicle.wheel_base, -999.0);
    nh_.param("length", env_->vehicle.length, -999.0);
    nh_.param("width", env_->vehicle.width, -999.0);
    nh_.param("vehicle_buffer", env_->vehicle.vehicle_buffer, -999.0);
    nh_.param("max_acceleration", env_->vehicle.max_acceleration, -999.0);
    nh_.param("max_deceleration", env_->vehicle.max_deceleration, -999.0);
    nh_.param("max_velocity", env_->vehicle.max_velocity, -999.0);
    nh_.param("max_reverse_velocity", env_->vehicle.max_reverse_velocity, -999.0);
    nh_.param("max_phi", env_->vehicle.max_phi, -999.0);
    nh_.param("min_phi", env_->vehicle.min_phi, -999.0);
    nh_.param("max_omega", env_->vehicle.max_omega, -999.0);
    env_->vehicle.GenerateDiscs();

    // DP Path Parameters
    nh_.param("path_tf", dp_path_config_.tf, -999.0);
    nh_.param("s_coefficient", dp_path_config_.s_coefficient, -999.0);
    nh_.param("resample_resolution", dp_path_config_.resample_resolution, -999.0);
    nh_.param("w_lateral", dp_path_config_.w_lateral, -999.0);
    nh_.param("w_lateral_change", dp_path_config_.w_lateral_change, -999.0);
    // nh_.param("w_obstacle", dp_path_config_.w_obstacle, -999.0);
    // nh_.param("w_obstacle_near", dp_path_config_.w_obstacle, -999.0);
    // nh_.param("w_obstacle", dp_path_config_.w_obstacle, -999.0);
    nh_.param("w_target", dp_path_config_.w_target, -999.0);
    nh_.param("collision_check_resolution", dp_path_config_.collision_check_resolution, -999.0);
    // nh_.param("target_range", dp_path_config_.target_range, std::vector<double>{-999.0, -999.0});
    nh_.param("valid_angular", dp_path_config_.valid_angular, -999.0);
    // nh_.param("num_s", dp_path_config_.num_s, -999);
    // nh_.param("num_l", dp_path_config_.num_l, -999);
    nh_.param("l_deviation", dp_path_config_.l_deviation, -999.0);
    nh_.param("road_half_width", dp_path_config_.road_half_width, -999.0);
    nh_.param("center_line_obs_range_front", dp_path_config_.center_line_obs_range_front, -999.0);
    nh_.param("center_line_obs_range_rear", dp_path_config_.center_line_obs_range_rear, -999.0);
    nh_.param("min_s_distance", dp_path_config_.min_s_distance, -999.0);
    nh_.param("initial_next_length", dp_path_config_.initial_next_length, -999.0);

    // nh_.param("s_therehold", dp_path_config_.s_therehold, -999.0);
    // nh_.param("fine_num_s", dp_path_config_.fine_num_s, -999);
    // nh_.param("fine_num_l", dp_path_config_.fine_num_l, -999);
    // nh_.param("fine_check_resolution", dp_path_config_.fine_check_resolution, 0.5);
    // nh_.param("coarse_num_s", dp_path_config_.coarse_num_s, -999);
    // nh_.param("coarse_num_l", dp_path_config_.coarse_num_l, -999);
    // nh_.param("coarse_check_resolution", dp_path_config_.coarse_check_resolution, 0.5);

    // DP Speed Parameters
    nh_.param("tf", dp_speed_config_.tf, -999.0);
    nh_.param("nfe", dp_speed_config_.nfe, -999);
    // nh_.param("num_s_velocity", dp_speed_config_.num_s_velocity, -999);
    // nh_.param("num_v", dp_speed_config_.num_v, -999);
    // nh_.param("w_nominal_velocity", dp_speed_config_.w_nominal_velocity, -999.0);
    // nh_.param("w_acceleration", dp_speed_config_.w_acceleration, -999.0);
    nh_.param("v_min", dp_speed_config_.v_min, -999.0);
    nh_.param("dp_cost_max", dp_speed_config_.dp_cost_max, -999.0);
    nh_.param("dp_cost_acc_max", dp_speed_config_.dp_cost_acc_max, -999.0);
    nh_.param("min_lateral_acceleration", dp_speed_config_.min_lateral_acceleration, -999.0);
    nh_.param("v_ratio", dp_speed_config_.v_ratio, -999.0);
    nh_.param("a_ratio", dp_speed_config_.a_ratio, -999.0);
    nh_.param("v_reduction_rate", dp_speed_config_.v_reduction_rate, -999.0);
    nh_.param("dt_resolution", dp_speed_config_.dt_resolution, -999.0);

    // nh_.param("s_velocity_therehold", dp_speed_config_.s_velocity_therehold, -999.0);
    // nh_.param("fine_num_s_velocity", dp_speed_config_.fine_num_s_velocity, -999);
    // nh_.param("fine_num_v", dp_speed_config_.fine_num_v, -999);
    // nh_.param("coarse_num_s_velocity", dp_speed_config_.coarse_num_s_velocity, -999);
    // nh_.param("coarse_num_v", dp_speed_config_.coarse_num_v, -999);

    traj_delta_time_ = dp_speed_config_.tf / (dp_speed_config_.nfe - 1);

    // Trajectory NLP
    nh_.param("nfe", traj_nlp_config_.nfe, -999);
    // nh_.param("corridor_step", traj_nlp_config_.corridor_step, -999.0);
    // nh_.param("w_opti_omega", traj_nlp_config_.w_opti_omega, -999.0);
    // nh_.param("w_opti_a", traj_nlp_config_.w_opti_a, -999.0);
    nh_.param("w_end_ratio", traj_nlp_config_.end_ratio, -999.0);
    // nh_.param("w_opti_track", traj_nlp_config_.w_opti_track, -999.0);
    // nh_.param("w_opti_track_theta", traj_nlp_config_.w_opti_track_theta, -999.0);
    // nh_.param("w_opti_track_v", traj_nlp_config_.w_opti_track_v, -999.0);
    // traj_nlp_config_.w_opti_xy_end = traj_nlp_config_.end_ratio * traj_nlp_config_.w_opti_track;
    // traj_nlp_config_.w_opti_theta_end = traj_nlp_config_.end_ratio * traj_nlp_config_.w_opti_track_theta;
    // traj_nlp_config_.w_opti_v_end = traj_nlp_config_.end_ratio * traj_nlp_config_.w_opti_track_v;

    nh_.param("corridor_max_iter", traj_nlp_config_.corridor_max_iter, -999);
    nh_.param("corridor_incremental_limit", traj_nlp_config_.corridor_incremental_limit, -999.0);

    // Frame
    nh_.param("stitch_time", stitch_time_, -999.0);
    nh_.param("road_ratio", road_ratio_, -999.0);
    nh_.param("initial_obs_index", initial_obs_index_, -999);
    nh_.param("initial_vel_index", initial_vel_index_, -999);
    nh_.param("min_s_distance", min_s_distance_, -999.0);
    nh_.param("obs_min_l", obs_min_l_, -999.0);
    nh_.param("obs_max_l", obs_max_l_, -999.0);
    nh_.param("obs_length", obs_length_, -999.0);
    nh_.param("obs_num", obs_num_, -999);
    nh_.param("traj_v_max_ratio", traj_v_max_ratio_, -999.0);
    nh_.param("initial_target_index", initial_target_index_, -999);
    nh_.param("is_not_generate_target", is_not_generate_target_, -999.0);
    nh_.param("intercept_max_length", intercept_max_length_, -999.0);
    nh_.param("stop_extend_s", stop_extend_s_, -999.0);
    nh_.param("stop_acc_ratio", stop_acc_ratio_, -999.0);
    nh_.param("fault_add_stop_point_num", fault_add_stop_point_num_, -999);
    nh_.param("vel_move_dist", vel_move_dist_, -999.0);

    // Experiment
    nh_.param("virtual_obs_index", virtual_obs_index_, std::vector<int>{-999, -999, -999, -999});
    nh_.param("virtual_obs_l", virtual_obs_l_, std::vector<double>{-999.0, -999.0, -999.0, -999.0});
    nh_.param("virtual_obs_length", virtual_obs_length_, -999.0);

    env_file_ = nh_.param<std::string>("env_file", "env.json");
}

bool Planner::RollingTimePlan(int driving_type, const Trajectory& last_trajectory, Trajectory& new_trajectory) {
    MyEnvironment my_env;
    double start_time = common::util::GetCurrentTimestamp();

    ROS_INFO("RollingTimePlan called: last_trajectory.size()=%zu, driving_type=%d", last_trajectory.size(), driving_type);

    if (last_trajectory.empty()) {
        ROS_INFO("last trajectory is empty");
        if (PlanTrajectory(my_env, driving_type, TrajectoryPoint(current_state_), new_trajectory)) {
            start_time_ = common::util::GetCurrentTimestamp();
            new_trajectory.front().t = current_state_.t;

            for (size_t i = 1; i < new_trajectory.size(); ++i) {
                // 后续点依次递增
                new_trajectory.at(i).t = new_trajectory.at(i - 1).t + traj_delta_time_;
            }
            return true;
        }
        return false;
    }

    // 创建一个可修改的轨迹副本，后续所有操作都基于这个副本
    Trajectory safe_trajectory_to_stitch = last_trajectory;
    driving_type = 2;

    // 检查并截断轨迹副本
    TruncateTrajectoryToSafePoint(safe_trajectory_to_stitch);

    // 如果截断后轨迹太短或完全不可行，则无法进行拼接，应直接重规划
    if (safe_trajectory_to_stitch.size() < 2) {
        ROS_WARN("上一条轨迹在截断后过短或不可行，将从当前状态重新规划。");
        // 调用完整的重规划逻辑
        if (PlanTrajectory(my_env, driving_type, TrajectoryPoint(current_state_), new_trajectory)) {
            start_time_ = common::util::GetCurrentTimestamp();
            if (!new_trajectory.empty()) {
                new_trajectory.front().t = current_state_.t;
                for (size_t i = 1; i < new_trajectory.size(); ++i) {
                    new_trajectory.at(i).t = new_trajectory.at(i - 1).t + traj_delta_time_;
                }
            }
            return true;
        }
        return false;    // 如果重规划也失败了
    }

    int current_match_index;

    ROS_INFO("Before FindStitchIndex: safe_trajectory_to_stitch.size()=%zu", safe_trajectory_to_stitch.size());

    if (!FindStitchIndex(safe_trajectory_to_stitch, current_match_index)) {
        ROS_ERROR("FindStitchIndex failed");
        return false;
    }

    ROS_INFO("FindStitchIndex result: current_match_index=%d", current_match_index);

    // 确保 stitch_time_ 和 traj_delta_time_ 都是正数且合理
    if (stitch_time_ <= 0.0 || traj_delta_time_ <= 0.0) {
        ROS_ERROR("Invalid timing parameters: stitch_time_=%.6f, traj_delta_time_=%.6f", stitch_time_, traj_delta_time_);
        return false;
    }

    int increase_index = std::max(1, static_cast<int>(std::ceil(stitch_time_ / traj_delta_time_)));
    ROS_INFO("Calculated increase_index=%d, stitch_time_=%.3f, traj_delta_time_=%.6f", increase_index, stitch_time_, traj_delta_time_);

    int target_index = current_match_index + increase_index;
    ROS_INFO("Target index: current_match_index(%d) + increase_index(%d) = %d", current_match_index, increase_index, target_index);

    // 边界检查
    if (target_index < 0) {
        ROS_ERROR("Target index is negative: %d, using index 0", target_index);
        target_index = 0;
    }

    if (target_index >= static_cast<int>(safe_trajectory_to_stitch.size())) {
        ROS_WARN("Target index %d >= trajectory size %zu, clamping to size-1", target_index, safe_trajectory_to_stitch.size());
        target_index = safe_trajectory_to_stitch.size() - 1;
    }

    // 额外的安全检查
    if (target_index < 0 || target_index >= static_cast<int>(safe_trajectory_to_stitch.size())) {
        ROS_ERROR("CRITICAL: target_index %d still out of bounds for size %zu", target_index, safe_trajectory_to_stitch.size());
        return false;
    }

    ROS_INFO("Final target_index after bounds check: %d", target_index);

    planned_start_state_ = safe_trajectory_to_stitch.at(target_index);
    ROS_INFO("Successfully got planned_start_state_ from index %d", target_index);

    // 执行路径规划
    Trajectory planned_trajectory;
    if (!PlanTrajectory(my_env, driving_type, TrajectoryPoint(planned_start_state_), planned_trajectory)) {
        ROS_INFO("Plan Failed!");
        return false;
    }

    // 轨迹拼接
    common::data::Trajectory combined_trajectory;

    ROS_INFO("Copying trajectory segment: from 0 to %d (size: %zu)", target_index, safe_trajectory_to_stitch.size());

    // 拷贝前半段轨迹
    for (int i = 0; i < target_index && i < static_cast<int>(safe_trajectory_to_stitch.size()); ++i) {
        combined_trajectory.push_back(safe_trajectory_to_stitch.at(i));
    }

    ROS_INFO("Copied %zu points from original trajectory", combined_trajectory.size());

    // 检查新规划的轨迹是否为空
    if (planned_trajectory.empty()) {
        ROS_ERROR("Planned trajectory is empty, cannot combine");
        return false;
    }

    // 拼接新规划的轨迹
    combined_trajectory.insert(combined_trajectory.end(), planned_trajectory.begin(), planned_trajectory.end());

    ROS_INFO("Final combined trajectory size: %zu", combined_trajectory.size());

    // 时间戳更新
    if (!combined_trajectory.empty() && target_index < static_cast<int>(combined_trajectory.size())) {
        double base_time = target_index > 0 ? combined_trajectory.at(target_index - 1).t : 0.0;

        for (size_t i = target_index; i < combined_trajectory.size(); ++i) {
            if (i == target_index) {
                // 第一个新点的时间戳基于前一个点
                if (target_index > 0) {
                    combined_trajectory.at(i).t = combined_trajectory.at(target_index - 1).t + traj_delta_time_;
                } else {
                    combined_trajectory.at(i).t = 0.0;
                }
            } else {
                // 后续点依次递增
                combined_trajectory.at(i).t = combined_trajectory.at(i - 1).t + traj_delta_time_;
            }
        }
    }

    new_trajectory = combined_trajectory;

    VisualizationPlot::Trigger();
    start_time_ = common::util::GetCurrentTimestamp();

    ROS_INFO("RollingTimePlan completed successfully");
    return true;
}

bool Planner::PlanTrajectory(const MyEnvironment& my_env, int driving_type, TrajectoryPoint start, Trajectory& result) {
    ROS_INFO("=== PlanTrajectory START ===");
    ROS_INFO("PlanTrajectory called: driving_type=%d, start=(%.3f,%.3f,%.3f)", driving_type, start.x, start.y, start.theta);

    double run_time = 0.0;

    auto bounds = env_->points.bounds();
    env_->x_min = bounds[0];
    env_->x_max = bounds[1];
    env_->y_min = bounds[2];
    env_->y_max = bounds[3];

    double start_time = common::util::GetCurrentTimestamp();

    bool is_success = false;

    common::data::Trajectory dp_result;
    Pose start_pose = Pose(start);
    auto input_reference = common::data::DiscretizedTrajectory(traj_ref());

    ROS_INFO("Input reference size: %zu", input_reference.data().size());
    ROS_INFO("Environment obstacles size: %zu", env_->obstacles.size());

    auto start_dp_path_time = common::util::GetHighestCurrentTimestamp();

    my_dp_time_ = 0.0;
    my_nlp_time_ = 0.0;

    // DP Path Planning
    ROS_INFO("Starting DP path planning...");

    if (dp_path_decider_->Plan(input_reference, start, result, dp_result)) {
        double dp_path_time = common::util::GetHighestCurrentTimestamp() - start_dp_path_time;
        my_dp_time_ += dp_path_time;
        ROS_INFO("DP path planning succeeded, time: %lf s, result size: %zu", dp_path_time, dp_result.size());

        std::vector<double> xs, ys;
        for (auto& i : dp_result) {
            xs.push_back(i.x);
            ys.push_back(i.y);
        }
    } else {
        ROS_ERROR("DP path planning failed!");
        return false;
    }

    // DP Speed Planning
    ROS_INFO("Starting DP speed planning...");
    auto start_dp_speed_time = common::util::GetHighestCurrentTimestamp();

    if (dp_sv_graph_->Plan(input_reference, start, dp_result)) {
        double dp_velocity_time = common::util::GetHighestCurrentTimestamp() - start_dp_speed_time;
        my_dp_time_ += dp_velocity_time;
        ROS_INFO("DP velocity planning succeeded, time: %lf s", dp_velocity_time);
    } else {
        ROS_ERROR("DP velocity planning failed!");
        return false;
    }

    ROS_INFO("After DP planning, dp_result size: %zu", dp_result.size());

    std::vector<double> xs_sv, ys_sv;
    bool is_suc = true;
    for (auto& i : dp_result) {
        if (!xs_sv.empty() && hypot(i.x - xs_sv.back(), i.y - ys_sv.back()) > 0.1) {
            is_suc = false;
            return false;
            break;
        }
        xs_sv.push_back(i.x);
        ys_sv.push_back(i.y);
    }
    if (is_suc) {
        VisualizationPlot::Plot(xs_sv, ys_sv, 0.01, Color::Magenta, 7, "sv_result");
    }

    std::vector<double> x_center_line, y_center_line;
    for (size_t i = 0; i < env_->center_line.size(); ++i) {
        x_center_line.push_back(env_->center_line.at(i).x());
        y_center_line.push_back(env_->center_line.at(i).y());
    }
    VisualizationPlot::Plot(x_center_line, y_center_line, 0.005, Color::Yellow, 8, "center_line");

    double start_nlp_time = common::util::GetHighestCurrentTimestamp();
    common::data::Trajectory nlp_result;
    my_nlp_time_ = common::util::GetHighestCurrentTimestamp() - start_nlp_time;

    // Trajectory Optimization
    ROS_INFO("Starting trajectory optimization...");
    double start_liom_time = common::util::GetHighestCurrentTimestamp();
    common::data::Trajectory liom_result;
    std::vector<double> infeasibility_vector;
    env()->good_index = 0;
    if (traj_opti_->IterationOptimize(dp_result, liom_result, infeasibility_vector)) {
        my_liom_time_ = common::util::GetHighestCurrentTimestamp() - start_liom_time;
        ROS_INFO("Trajectory optimization succeeded, time: %lf s, result size: %zu", my_liom_time_, liom_result.size());
        double sum_infeasibility = 0.0;
        for (size_t i = 0; i < infeasibility_vector.size(); ++i) {
            sum_infeasibility += infeasibility_vector[i];
            if (sum_infeasibility > 1e-5) {
                break;
            }
            env()->good_index = i + 1;
        }
    } else {
        my_liom_time_ = common::util::GetHighestCurrentTimestamp() - start_liom_time;
        ROS_ERROR("Trajectory optimization failed!");
        return false;
    }

    // std::vector<double> xs_nlp, ys_nlp;
    // for (size_t i = 0; i <= 25; ++i) {
    //     xs_nlp.push_back(liom_result[i].x);
    //     ys_nlp.push_back(liom_result[i].y);
    // }
    // VisualizationPlot::Plot(xs_nlp, ys_nlp, 0.01, Color::Red, 10, "traj_result_before");
    // xs_nlp.clear();
    // ys_nlp.clear();
    // for (size_t i = 25; i < liom_result.size(); ++i) {
    //     xs_nlp.push_back(liom_result[i].x);
    //     ys_nlp.push_back(liom_result[i].y);
    // }
    // VisualizationPlot::Plot(xs_nlp, ys_nlp, 0.01, Color::Green, 11, "traj_result_after");
    // VisualizationPlot::PlotPoints({liom_result.front().x}, {liom_result.front().y}, 0.025, Color::Red, 12, "start_point");

    if (driving_type == 1) {
        std::vector<double> xs_nlp, ys_nlp;
        // for (size_t i = 0; i <= 25; ++i) {
        //     xs_nlp.push_back(liom_result[i].x);
        //     ys_nlp.push_back(liom_result[i].y);
        // }
        // VisualizationPlot::Plot(xs_nlp, ys_nlp, 0.01, Color::Red, 10, "traj_result_before");
        // xs_nlp.clear();
        // ys_nlp.clear();
        for (size_t i = 0; i < liom_result.size(); ++i) {
            xs_nlp.push_back(liom_result[i].x);
            ys_nlp.push_back(liom_result[i].y);
        }
        VisualizationPlot::Plot(xs_nlp, ys_nlp, 0.01, Color::Green, 11, "traj_result_after");
    }

    VisualizationPlot::Trigger();

    run_time = common::util::GetCurrentTimestamp() - start_time;

    ROS_INFO("PlanTrajectory completed successfully: run_time=%.6f", run_time);
    result = liom_result;

    ROS_INFO("=== PlanTrajectory END ===");
    return true;
}

int Planner::FindClosestIndex(const std::vector<double>& vec, double value) {
    auto it = std::min_element(vec.begin(), vec.end(), [value](double a, double b) { return std::abs(a - value) < std::abs(b - value); });
    return std::distance(vec.begin(), it);
}

void Planner::GetRefLine(const std::vector<map_generator::Road>& roads) {
    std::vector<double> ref_xs, ref_ys;
    std::vector<Pose> ref_line;
    for (int i = 0; i < roads.size(); i++) {
        for (int j = 0; j < roads[i].points.size(); j++) {
            ref_xs.push_back(roads[i].points[j].x);
            ref_ys.push_back(roads[i].points[j].y);
        }
    }
    double s_integral = 0;
    for (int i = 0; i < roads.size(); i++) {
        for (int j = 0; j < roads[i].points.size(); j++) {
            common::data::TrajectoryPoint pt;
            pt.x = (roads[i].points[j].x);
            pt.y = (roads[i].points[j].y);
            pt.theta = (roads[i].points[j].theta);
            pt.v = (nominal_v_);
            pt.s = s_integral;
            if (j < roads[i].points.size() - 1) {
                s_integral += std::hypot(roads[i].points[j + 1].y - roads[i].points[j].y, roads[i].points[j + 1].x - roads[i].points[j].x);
            }

            traj_ref_.emplace_back(pt);
        }
    }
    Color c = Color::White;
    c.set_a(0.8);
    VisualizationPlot::Plot(ref_xs, ref_ys, 0.01, c, 0, "road");
    VisualizationPlot::Trigger();
}

bool Planner::FindStitchIndex(common::data::Trajectory traj_pool, int& index) {
    ROS_INFO("FindStitchIndex called: traj_pool.size()=%zu", traj_pool.size());

    if (traj_pool.empty()) {
        ROS_ERROR("FindStitchIndex: trajectory pool is empty");
        index = 0;
        return false;
    }

    // 检查当前状态是否有效
    if (std::isnan(current_state_.x) || std::isnan(current_state_.y)) {
        ROS_ERROR("FindStitchIndex: current state contains NaN values");
        return false;
    }

    ROS_INFO("Current state: x=%.3f, y=%.3f", current_state_.x, current_state_.y);
    ROS_INFO("First trajectory point: x=%.3f, y=%.3f", traj_pool[0].x, traj_pool[0].y);

    double min_distance = std::hypot(current_state_.x - traj_pool[0].x, current_state_.y - traj_pool[0].y);
    int station_index = 0;

    ROS_INFO("Initial min_distance=%.3f", min_distance);

    // 寻找最近点
    for (size_t i = 1; i < traj_pool.size(); i++) {
        // 检查轨迹点是否有效
        if (std::isnan(traj_pool[i].x) || std::isnan(traj_pool[i].y)) {
            ROS_WARN("Skipping trajectory point %zu due to NaN values", i);
            continue;
        }

        double distance = std::hypot(current_state_.x - traj_pool[i].x, current_state_.y - traj_pool[i].y);
        if (distance < min_distance) {
            station_index = static_cast<int>(i);
            min_distance = distance;
        }
    }

    ROS_INFO("Found station_index=%d with min_distance=%.3f", station_index, min_distance);

    // 检查距离是否在合理范围内
    if (min_distance > min_s_distance_) {
        ROS_WARN("Min distance %.3f > threshold %.3f, but continuing", min_distance, min_s_distance_);
        // 不要直接返回false，而是继续使用找到的最近点
    }

    // 最终的边界检查
    if (station_index < 0 || station_index >= static_cast<int>(traj_pool.size())) {
        ROS_ERROR("Invalid station_index %d for trajectory size %zu", station_index, traj_pool.size());
        station_index = std::max(0, std::min(station_index, static_cast<int>(traj_pool.size()) - 1));
        ROS_WARN("Clamped station_index to %d", station_index);
    }

    index = station_index;
    ROS_INFO("FindStitchIndex returning: index=%d", index);
    return true;
}

bool Planner::GenerateRefLine(const common::data::DiscretizedTrajectory& road, const Pose current_pose,
                              common::data::Trajectory& ref_line) {
    auto sl = road.GetProjection(Vec2d(current_pose));
    double s_distance_min = std::numeric_limits<double>::max();
    int s_match_index = 0;
    auto road_size = road.data().size();
    int backward_num = 1.0 / road.data().back().s * road_size;
    for (int i = 0; i < road_size; ++i) {
        double s_distance = std::fabs(road.data().at(i).s - sl.x());
        if (s_distance < s_distance_min) {
            s_distance_min = s_distance;
            s_match_index = (i - backward_num) >= 0 ? (i - backward_num) : (road_size + (i - backward_num) - 1);
        } else {
            break;
        }
    }
    if (s_distance_min > min_s_distance_) {
        return false;
    }

    ref_line.clear();
    double total_s = 0.0;

    if (road_size - s_match_index > round(road_size * road_ratio_)) {
        for (int i = s_match_index; i < s_match_index + round(road_size * road_ratio_); ++i) {
            ref_line.push_back(road.data().at(i));
            if (i == s_match_index) {
                ref_line.front().s = 0.0;
            } else {
                auto last = *(ref_line.end() - 2);
                total_s += hypot(ref_line.back().x - last.x, ref_line.back().y - last.y);
                ref_line.back().s = total_s;
            }
        }
    } else {
        for (int i = s_match_index; i < road_size; ++i) {
            ref_line.push_back(road.data().at(i));
            if (i == s_match_index) {
                ref_line.front().s = 0.0;
            } else {
                auto last = *(ref_line.end() - 2);
                total_s += hypot(ref_line.back().x - last.x, ref_line.back().y - last.y);
                ref_line.back().s = total_s;
            }
        }
        for (int i = 0; i < round(road_size * road_ratio_) - (road_size - s_match_index); ++i) {
            ref_line.push_back(road.data().at(i));
            if (i == 0) {
                auto last = *(ref_line.end() - 2);
                total_s += hypot(ref_line.back().x - last.x, ref_line.back().y - last.y);
                ref_line.back().s = total_s;
            } else {
                auto last = *(ref_line.end() - 2);
                total_s += hypot(ref_line.back().x - last.x, ref_line.back().y - last.y);
                ref_line.back().s = total_s;
            }
        }
    }
    return true;
}

void Planner::CheckTrajFeasible(const Trajectory& result) {
    for (size_t i = 0; i < result.size(); ++i) {
        auto g_v_region = env_->vehicle.max_velocity - result.at(i).v;
        if (g_v_region < -0.000001) {
            VisualizationPlot::PlotCross(result.at(i).x, result.at(i).y, 0.010, Color::Red, i + 3000, "check_sign");
            ROS_ERROR("v limit invalid error is %lf", -g_v_region);
        } else {
            VisualizationPlot::PlotCross({}, {}, 0.00, Color::Grey, i + 3000, "check_sign");
        }
    }
}

void Planner::GenerateObstacles(const common::data::DiscretizedTrajectory& road, bool is_first, int index) {
    ROS_INFO("GenerateObstacles called: is_first=%d, index=%d, road.size()=%zu", is_first, index, road.data().size());

    std::uniform_real_distribution<double> dist1_obs(obs_min_l_, obs_max_l_);
    std::uniform_real_distribution<double> dist2_obs(-obs_max_l_, -obs_min_l_);
    std::uniform_int_distribution<int> dist_obs_choice(0, 1);

    const int num_obstacles = obs_num_;

    if (is_first) {
        obs_index_.clear();
        env_->obstacles.clear();
        env_->obstacles_struct.clear();

        if (obs_num_ >= 1 && !road.data().empty()) {
            int road_div = road.data().size() / num_obstacles;

            // 使用种子初始化随机数引擎
            random_generator_.seed(random_seed_);

            auto generate_random_l = [&]() -> double {
                if (side_chooser_dist_(random_generator_) == 0) {
                    return left_lateral_dist_(random_generator_);
                } else {
                    return right_lateral_dist_(random_generator_);
                }
            };

            for (int i = 0; i < num_obstacles; ++i) {
                // 位置: s值沿路前进 i/num_obstacles，横向位置l随机
                int obs_index = (initial_obs_index_ + i * road_div) % road.data().size();

                // 边界检查
                if (obs_index < 0 || obs_index >= static_cast<int>(road.data().size())) {
                    continue;
                }

                auto& road_point = road.data().at(obs_index);
                auto obs_center = road.ToCartesian(road_point.s, generate_random_l());

                // 尺寸: 随机
                double obs_length = length_dist_(random_generator_);
                double obs_width = width_dist_(random_generator_);

                // 朝向: 随机
                double road_heading = road_point.theta;
                double random_deviation = heading_deviation_dist_(random_generator_);
                double final_obs_heading = atan2(sin(road_heading + random_deviation), cos(road_heading + random_deviation));

                auto obs_box = common::math::Box2d(obs_center, final_obs_heading, obs_length, obs_width);

                // 添加到环境
                DynamicAABB obs_struct;
                obs_struct.center_xy = obs_center;
                obs_struct.change_time = common::util::GetCurrentTimestamp() - 100.0;
                obs_struct.dynamic_aabb = common::math::Polygon2d(obs_box);
                obs_struct.obs_box = obs_box;
                obs_struct.length = obs_length;
                obs_struct.width = obs_width;
                obs_struct.heading = final_obs_heading;
                env_->obstacles_struct.push_back(obs_struct);
                env_->obstacles.push_back(common::Obstacle(obs_struct.dynamic_aabb));

                obs_index_.push_back(obs_index);
            }
        }

        // Virtual Obstacles - 添加边界检查
        for (size_t i = 0; i < 4; ++i) {
            int obs_idx = virtual_obs_index_[i];

            // 边界检查
            if (obs_idx < 0 || obs_idx >= static_cast<int>(road.data().size())) {
                ROS_ERROR("Invalid virtual_obs_index[%zu] = %d for road size %zu, skipping", i, obs_idx, road.data().size());
                continue;
            }

            double obs_l = virtual_obs_l_[i];
            auto& road_point_virtual = road.data().at(obs_idx);
            auto obs_center_virtual = road.ToCartesian(road_point_virtual.s, obs_l);

            double road_heading_virtual = road_point_virtual.theta;
            double random_deviation_virtual = heading_deviation_dist_(random_generator_);
            double final_heading_virtual =
                atan2(sin(road_heading_virtual + random_deviation_virtual), cos(road_heading_virtual + random_deviation_virtual));

            auto obs_box_virtual = common::math::Box2d(obs_center_virtual, final_heading_virtual, virtual_obs_length_, virtual_obs_length_);

            DynamicAABB obs_struct_virtual;
            obs_struct_virtual.center_xy = obs_center_virtual;
            obs_struct_virtual.dynamic_aabb = common::math::Polygon2d(obs_box_virtual);
            obs_struct_virtual.obs_box = obs_box_virtual;
            obs_struct_virtual.length = virtual_obs_length_;
            obs_struct_virtual.width = virtual_obs_length_;
            obs_struct_virtual.heading = final_heading_virtual;
            env_->obstacles_struct.push_back(obs_struct_virtual);
            env_->obstacles.push_back(common::Obstacle(obs_struct_virtual.dynamic_aabb));
        }

        env_->GetObsXY(env_->obs_border_point_x, env_->obs_border_point_y);
        env_->GetObsPoints(env_->obs_points_);

        ROS_INFO("Created %zu obstacles total", env_->obstacles.size());
        return;
    }

    // 更新现有障碍物时的边界检查
    if (index != -1 && index < static_cast<int>(env_->obstacles_struct.size()) && index < static_cast<int>(obs_index_.size())) {
        bool obs_choice = dist_obs_choice(gen_);
        double obs_l = obs_choice ? dist1_obs(gen_) : dist2_obs(gen_);

        int road_idx = obs_index_.at(index);
        if (road_idx < 0 || road_idx >= static_cast<int>(road.data().size())) {
            ROS_ERROR("Invalid road index %d for obstacle %d, road size %zu", road_idx, index, road.data().size());
            return;
        }

        auto& road_point = road.data().at(road_idx);
        auto obs_new_center = road.ToCartesian(road_point.s, obs_l);

        double obs_length = env_->obstacles_struct.at(index).length;
        double obs_width = env_->obstacles_struct.at(index).width;

        double road_heading = road_point.theta;
        double random_deviation = heading_deviation_dist_(random_generator_);
        double final_obs_heading = atan2(sin(road_heading + random_deviation), cos(road_heading + random_deviation));

        auto obs_new_box = common::math::Box2d(obs_new_center, final_obs_heading, obs_length, obs_width);

        env_->obstacles_struct.at(index).change_time = common::util::GetCurrentTimestamp();
        env_->obstacles_struct.at(index).center_xy = obs_new_center;
        env_->obstacles_struct.at(index).obs_box = obs_new_box;
        env_->obstacles_struct.at(index).heading = final_obs_heading;
        env_->obstacles_struct.at(index).dynamic_aabb = common::math::Polygon2d(obs_new_box);
        env_->obstacles.at(index) = common::Obstacle(env_->obstacles_struct.at(index).dynamic_aabb);

        env_->GetObsXY(env_->obs_border_point_x, env_->obs_border_point_y);
        env_->GetObsPoints(env_->obs_points_);
    } else {
        ROS_ERROR("Invalid parameters for obstacle update: index=%d, obstacles_struct.size()=%zu, obs_index_.size()=%zu", index,
                  env_->obstacles_struct.size(), obs_index_.size());
    }
}

common::math::Polygon2d Planner::GenerateRandomConvexPolygon(const common::math::Vec2d& center, double avg_radius, int min_vertices,
                                                             int max_vertices, double irregularity, double spikeyness) {
    // 在设定的范围内随机决定顶点数量
    std::uniform_int_distribution<int> dist_num_vertices(min_vertices, max_vertices);
    int num_vertices = dist_num_vertices(gen_);

    std::vector<common::math::Vec2d> points;
    points.reserve(num_vertices);

    // 随机生成顶点
    double angle_step = 2.0 * M_PI / num_vertices;
    for (int i = 0; i < num_vertices; ++i) {
        double angle = i * angle_step;

        // 增加角度和半径的随机扰动
        std::uniform_real_distribution<double> dist_angle_offset(-angle_step * irregularity, angle_step * irregularity);
        double random_angle = angle + dist_angle_offset(gen_);

        std::uniform_real_distribution<double> dist_radius(avg_radius * (1.0 - spikeyness), avg_radius * (1.0 + spikeyness));
        double random_radius = dist_radius(gen_);

        points.emplace_back(center.x() + random_radius * cos(random_angle), center.y() + random_radius * sin(random_angle));
    }

    // --- 计算凸包 (Graham Scan 算法的简化实现) ---
    // 1. 找到Y坐标最小的点作为基准点 P0
    auto p0_it = std::min_element(points.begin(), points.end(),
                                  [](const auto& a, const auto& b) { return a.y() < b.y() || (a.y() == b.y() && a.x() < b.x()); });
    std::swap(*points.begin(), *p0_it);
    const auto& p0 = points[0];

    // 2. 将其他点按相对于P0的极角排序
    std::sort(std::next(points.begin()), points.end(), [&p0](const auto& a, const auto& b) {
        double angle_a = atan2(a.y() - p0.y(), a.x() - p0.x());
        double angle_b = atan2(b.y() - p0.y(), b.x() - p0.x());
        return angle_a < angle_b;
    });

    // 3. 构建凸包
    std::vector<common::math::Vec2d> hull;
    for (const auto& pt : points) {
        while (hull.size() >= 2) {
            // 检查新点是否在最后两个点的左侧 (叉积判断)
            double cross_product = (hull.back().x() - hull[hull.size() - 2].x()) * (pt.y() - hull.back().y()) -
                                   (hull.back().y() - hull[hull.size() - 2].y()) * (pt.x() - hull.back().x());
            if (cross_product >= 0) {    // 如果不是严格左转，则弹出栈顶
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(pt);
    }

    return common::math::Polygon2d(hull);
}

std::vector<common::math::Vec2d> Planner::PushPoints(const common::data::Trajectory& points, double distance) {
    std::vector<common::math::Vec2d> newPoints;
    for (int i = 0; i < points.size(); ++i) {
        double normal_x = std::sin(points.at(i).theta);
        double normal_y = -std::cos(points.at(i).theta);

        common::math::Vec2d newPoint;
        newPoint.set_x(points.at(i).x - distance * normal_x);
        newPoint.set_y(points.at(i).y - distance * normal_y);

        newPoints.push_back(newPoint);
    }
    return newPoints;
}

void Planner::PushPoints(const common::data::Trajectory& points, double distance, common::data::Trajectory& result) {
    result.clear();
    for (int i = 0; i < points.size(); ++i) {
        double normal_x = std::sin(points.at(i).theta);
        double normal_y = -std::cos(points.at(i).theta);

        common::data::TrajectoryPoint new_tp;
        new_tp = points.at(i);
        new_tp.x = points.at(i).x - distance * normal_x;
        new_tp.y = points.at(i).y - distance * normal_y;
        if (i == 0) {
            new_tp.s = 0.0;
        } else {
            new_tp.s = result.back().s + hypot(new_tp.x - result.back().x, new_tp.y - result.back().y);
        }

        result.emplace_back(new_tp);
    }
}

void Planner::LogCounter(const int& counter) {
    std::ofstream file;

    file.open(log_filename_, std::ios::app);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open file for logging");
        return;
    }
    file << counter << "\n";
    file.close();
}

void Planner::LogTime(double dp_solve_time, double nlp_solve_time, const std::string& log_filename) {
    std::ofstream file;

    // 以追加模式打开文件
    file.open(log_filename, std::ios::app);
    if (!file.is_open()) {
        ROS_ERROR("无法打开日志文件进行记录: %s", log_filename.c_str());
        return;
    }

    // 将两个耗时数据以逗号分隔的形式写入，并换行
    // 这会创建一个简单的CSV文件，便于后续处理
    file << dp_solve_time << "," << nlp_solve_time << "\n";

    file.close();
}

void Planner::LogCost(double liom_cost, double nlp_cost, const std::string& log_filename) {
    std::ofstream file;

    // 以追加模式打开文件
    file.open(log_filename, std::ios::app);
    if (!file.is_open()) {
        ROS_ERROR("无法打开日志文件进行记录: %s", log_filename.c_str());
        return;
    }

    // 写入代价值并换行
    file << liom_cost << "," << nlp_cost << "\n";

    file.close();
}

void Planner::LogTimeCost(double dp_solve_time, double nlp_solve_time, double liom_solve_time, double liom_cost, double nlp_cost,
                          const std::string& log_filename) {
    std::ofstream file;

    // 以追加模式打开文件
    file.open(log_filename, std::ios::app);
    if (!file.is_open()) {
        ROS_ERROR("无法打开日志文件进行记录: %s", log_filename.c_str());
        return;
    }

    // 将两个耗时数据以逗号分隔的形式写入，并换行
    // 这会创建一个简单的CSV文件，便于后续处理
    file << dp_solve_time << "," << nlp_solve_time << "," << liom_solve_time << "," << liom_cost << "," << nlp_cost << "\n";

    file.close();
}

void Planner::GenerateStopTraj(const TrajectoryPoint& plan_start, const common::data::DiscretizedTrajectory& ref_line, Trajectory& traj) {
    traj.clear();
    double v0 = plan_start.v;
    double max_deceleration_ = std::fabs(env()->vehicle.max_deceleration);    // 使用环境中的车辆参数
    double traj_time_resolution_ = 0.1;                                       // 轨迹时间分辨率，单位秒
    // --- 1. 处理已静止的情况 ---
    if (v0 < 0.01) {
        ROS_INFO("Vehicle is already stopped. Generating a stationary trajectory.");
        traj.reserve(20);
        for (int i = 0; i < 20; ++i) {    // 生成2秒的静止等待点
            common::data::TrajectoryPoint stop_point = plan_start;
            stop_point.v = 0.0;
            stop_point.a = 0.0;
            stop_point.t = plan_start.t + i * traj_time_resolution_;
            traj.push_back(stop_point);
        }
        return;
    }

    // --- 2. 根据物理公式计算刹车时间和距离 ---
    double time_to_stop = v0 / max_deceleration_;

    // --- 3. 随时间生成轨迹点 ---
    int num_steps = static_cast<int>(ceil(time_to_stop / traj_time_resolution_));
    traj.reserve(num_steps);    // Pre-allocate memory

    for (int i = 0; i <= num_steps; ++i) {
        double t = i * traj_time_resolution_;
        // 确保最后一个点的时间精确等于刹停时间
        if (t > time_to_stop) {
            t = time_to_stop;
        }

        common::data::TrajectoryPoint tp;

        // v = v0 + at
        double current_v = std::max(0.0, v0 - max_deceleration_ * t);
        // d = v0*t + 0.5*a*t^2
        double distance_traveled = v0 * t - 0.5 * max_deceleration_ * t * t;

        // 位姿: 沿着起始方向的直线上
        tp.x = plan_start.x + distance_traveled * std::cos(plan_start.theta);
        tp.y = plan_start.y + distance_traveled * std::sin(plan_start.theta);
        tp.theta = plan_start.theta;    // 航向角保持不变

        // 运动学状态
        tp.v = current_v;
        tp.a = (current_v > 1e-3) ? -max_deceleration_ : 0.0;    // 速度不为零时，加速度为最大减速度
        tp.t = plan_start.t + t;                                 // 使用绝对时间
        tp.phi = 0.0;                                            // 直线行驶，方向盘转角为0
        tp.omega = 0.0;                                          // 直线行驶，角速度为0

        traj.push_back(tp);

        if (t >= time_to_stop) {
            break;    // 确保在精确停止后不再添加点
        }
    }

    // --- 4. 在轨迹末尾添加额外的静止点，确保控制器有足够的时间反应 ---
    if (!traj.empty()) {
        common::data::TrajectoryPoint final_point = traj.back();
        final_point.v = 0.0;
        final_point.a = 0.0;
        for (int i = 1; i <= 10; ++i) {    // 额外增加1秒的静止点
            final_point.t += traj_time_resolution_;
            traj.push_back(final_point);
        }
    }

    // --- 5. 可视化最终生成的轨迹 ---
    std::vector<double> xs_traj, ys_traj;
    for (const auto& pt : traj) {
        xs_traj.push_back(pt.x);
        ys_traj.push_back(pt.y);
    }
    VisualizationPlot::Plot(xs_traj, ys_traj, 0.01, Color::Red, 0, "emergency_stop_traj");

    ROS_WARN("Generated physics-based stop trajectory. Time to stop: %.2fs, Final traj size: %zu", time_to_stop, traj.size());
}

void Planner::TruncateTrajectoryToSafePoint(common::data::Trajectory& traj) {
    if (traj.size() < 2) {
        return;    // 没有足够的点来进行检查
    }

    // 1. 计算整条轨迹的不可行性向量
    std::vector<double> infeasibility_vector = CalculateInfeasibilityVector(traj);

    // 2. 根据不可行性向量，找到轨迹的安全截断点
    int safe_end_index = FindSafeTrajectoryEndIndex(infeasibility_vector);

    // 3. 如果安全点是第一个点 (index 0) 或更早，说明整条轨迹都不可行
    if (safe_end_index < 1) {
        traj.clear();
        return;
    }

    // 4. 如果找到了不可行点，则将轨迹截断到安全的长度
    if (safe_end_index < infeasibility_vector.size()) {
        int original_size = traj.size();
        // safe_end_index 是最后一个有效点的索引, 所以新的大小是 index + 1
        traj.resize(safe_end_index + 1);
    }
}

std::vector<double> Planner::CalculateInfeasibilityVector(const common::data::Trajectory& traj) {
    if (traj.size() < 2) {
        return {};
    }

    std::vector<double> infeasibility_vector;
    infeasibility_vector.reserve(traj.size() - 1);

    for (size_t i = 0; i < traj.size() - 1; ++i) {
        const auto& prev = traj[i];
        const auto& next = traj[i + 1];

        double dt = next.t - prev.t;
        if (dt <= 1e-6) {    // 避免时间步过小导致计算问题
            infeasibility_vector.push_back(0.0);
            continue;
        }

        // 1. 根据前一个状态 prev，预测下一个状态 next_pred
        double x_pred = prev.x + dt * prev.v * std::cos(prev.theta);
        double y_pred = prev.y + dt * prev.v * std::sin(prev.theta);
        double theta_pred = prev.theta + dt * prev.v * std::tan(prev.phi) / env()->vehicle.wheel_base;
        double v_pred = prev.v + dt * prev.a;
        double phi_pred = prev.phi + dt * prev.omega;

        // 2. 计算预测值与真实值之间的误差
        double err_x = next.x - x_pred;
        double err_y = next.y - y_pred;
        double err_theta = next.theta - theta_pred;
        // 将角度误差归一化到 [-PI, PI] 区间
        err_theta = std::atan2(std::sin(err_theta), std::cos(err_theta));
        double err_v = next.v - v_pred;
        double err_phi = next.phi - phi_pred;

        // 3. 计算该步骤的不可行性（所有误差的平方和）
        double step_infeasibility = err_x * err_x + err_y * err_y + err_theta * err_theta + err_v * err_v + err_phi * err_phi;

        infeasibility_vector.push_back(step_infeasibility);
    }

    return infeasibility_vector;
}

int Planner::FindSafeTrajectoryEndIndex(const std::vector<double>& infeasibility_vector) {
    const double INFEASIBILITY_THRESHOLD = 1e-5;
    for (int i = 0; i < infeasibility_vector.size(); ++i) {
        if (infeasibility_vector[i] > INFEASIBILITY_THRESHOLD) {
            return i;
        }
    }
    return infeasibility_vector.size();
}
