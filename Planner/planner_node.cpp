#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <planner_node/Polygon2dArray.h>
#include <planner_node/Vec2d.h>
#include <ros/ros.h>
#include <sandbox_msgs/SteeringLimits.h>
#include <sandbox_msgs/Trajectory.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

// #include <boost/thread.hpp>

#include "agv_simulation_manager.h"
#include "common/backward.hpp"
#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/util/time.h"
#include "common/visualization_plot.h"
#include "interface.h"
// #include "kkswarm/multi_agv_trajectory_generator.h"

using namespace common::math;

// backward-cpp for crash stack traces (excluding SIGINT/SIGTERM for clean shutdown)
namespace backward {
backward::SignalHandling sh({SIGSEGV, SIGABRT, SIGFPE, SIGILL, SIGBUS});
}

class PlannerNode {
   public:
    PlannerNode() : nh_("~/planning") {
        pose_sub_ = nh_.subscribe("/initialpose", 10, &PlannerNode::GetStartPoseCallback, this, ros::TransportHints().tcpNoDelay());
        nokov_sub_ = nh_.subscribe("/nokov_info", 10, &PlannerNode::NokovCallback, this, ros::TransportHints().tcpNoDelay());
        // dynamic_obs_timer_ = nh_.createTimer(ros::Duration(0.002), &PlannerNode::DynamicObsVelTimer, this);

        std::vector<int> agv_ids;
        if (!nh_.getParam("real_agv_ids", agv_ids)) {
            agv_ids = {4, 5, 20};    // 默认ID
        }
        for (int id : agv_ids) {
            std::string topic = "/robot_" + std::to_string(id) + "/trajectory";
            // 使用 boost::bind 将 AGV 的 ID 绑定到回调函数中
            agv_traj_subscribers_[id] =
                nh_.subscribe<sandbox_msgs::Trajectory>(topic, 1, boost::bind(&PlannerNode::AgvTrajectoryCallback, this, _1, id));
            ROS_INFO("Subscribing to AGV trajectory topic: %s", topic.c_str());
        }

#ifdef EXPERIMENT
        real_motion_timer_ = nh_.createTimer(ros::Duration(0.002), &PlannerNode::RealMotionTimer, this);
#else
        simulated_motion_timer_ = nh_.createTimer(ros::Duration(0.002), &PlannerNode::SimulatedMotionTimer, this);
#endif

#ifdef RECORD_FOR_PLAYBACK
        enable_rosbag_recording_ = true;
        ego_pose_record_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/record/ego_pose", 10);
        agv_positions_record_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/record/agv_positions", 10);
        planned_traj_record_pub_ = nh_.advertise<sandbox_msgs::Trajectory>("/record/planned_trajectory", 10);
        timestamp_record_pub_ = nh_.advertise<std_msgs::String>("/record/timestamp", 10);

        ROS_INFO("ROS Bag recording mode enabled - publishing to /record/* topics");
#endif

        traj_pub_ = nh_.advertise<sandbox_msgs::Trajectory>("/traj", 1);
        plan_timer_ = nh_.createTimer(ros::Duration(0.002), &PlannerNode::PlanTimer, this);
        exp_obs_pub_ = nh_.advertise<planner_node::Polygon2dArray>("/obs", 10);

        env_file_ = nh_.param<std::string>("env_file", "env.json");
        // Frame
        nh_.param("min_s_distance", min_s_distance_, -999.0);
        nh_.param("suc_rolling_time", suc_rolling_time_, -999.0);
        nh_.param("fail_rolling_time", fail_rolling_time_, -999.0);
        nh_.param("replan_success_time", replan_success_time_, -999.0);
        nh_.param("send_traj_duration", send_traj_duration_, -999.0);
        nh_.param("replan_send_traj_duration", replan_send_traj_duration_, -999.0);
        nh_.param("obs_range", obs_range_, std::vector<double>{-999.0, -999.0});
        nh_.param("obs_num", obs_num_, -999);
        nh_.param("vel_region_range", vel_region_range_, std::vector<double>{-999.0, -999.0});
        nh_.param("obs_vel_time", obs_vel_time_, -999.0);
        nh_.param("num_failure_of_auto_replan", num_failure_of_auto_replan_, -999);
        nh_.param("intercept_max_length", intercept_max_length_, -999.0);
        nh_.param("traj_minimum_remaining_time", traj_minimum_remaining_time_, -999.0);
        nh_.param("traj_minimum_remaining_time_ratio", traj_minimum_remaining_time_ratio_, -999.0);
        traj_minimum_remaining_time_ *= traj_minimum_remaining_time_ratio_;
        nh_.param("rear_num", rear_num_, -999);
        nh_.param("inner_points_num", inner_points_num_, -999);

        // =====================> START: CRITICAL CRASH FIX <=====================
        // 在使用参数进行除法运算之前，立即进行合法性检查。
        // 这是导致概率性崩溃的根源：如果参数未从服务器加载，它将是-999，
        // 导致 (inner_points_num_ - 1) 变成负数，在转换为size_t后变成巨大正数，
        // 最终使采样步长 inner_step 变为0，从而引发无限循环和崩溃。
        if (inner_points_num_ <= 1) {
            ROS_FATAL(
                "Critical parameter error: 'inner_points_num' is %d, but must be > 1 to avoid division by zero. This is likely because "
                "the parameter was not loaded correctly from the launch/yaml file. Shutting down.",
                inner_points_num_);
            ros::shutdown();
            return;    // 立即退出构造函数，防止后续代码执行
        }
        // =====================> END: CRITICAL CRASH FIX <=====================
        // Map
        nh_.param("map_radius", map_param_.map_radius, -999.0);
        nh_.param("x_first_straight", map_param_.x_first_straight, -999.0);
        nh_.param("x_second_straight", map_param_.x_second_straight, -999.0);
        nh_.param("y_straight", map_param_.y_straight, -999.0);
        nh_.param("curve_length", map_param_.curve_length, -999.0);
        nh_.param("resolution", map_param_.resolution, -999.0);

        // =====================> START: FINAL CRASH FIX <=====================
        // 最终修复：验证所有地图参数是否已正确加载。
        // 根源在于，如果任何一个地图参数加载失败（取默认值-999），
        // 都会导致后续的地图点数计算错误，从而引发崩溃。
        // 此前的修复遗漏了对 x_second_straight 的检查。
        if (map_param_.map_radius <= 0.0 || map_param_.x_first_straight <= 0.0 || map_param_.x_second_straight < 0.0 ||
            map_param_.y_straight <= 0.0 || map_param_.curve_length <= 0.0 || map_param_.resolution <= 1e-6) {
            ROS_FATAL("Critical map parameter loading error! One or more map parameters are invalid (<= 0 or < 0).");
            ROS_FATAL("  map_radius: %.3f", map_param_.map_radius);
            ROS_FATAL("  x_first_straight: %.3f", map_param_.x_first_straight);
            ROS_FATAL("  x_second_straight: %.3f", map_param_.x_second_straight);    // Added check for this parameter
            ROS_FATAL("  y_straight: %.3f", map_param_.y_straight);
            ROS_FATAL("  curve_length: %.3f", map_param_.curve_length);
            ROS_FATAL("  resolution: %.3f", map_param_.resolution);
            ROS_FATAL("Please ensure all parameters are correctly set in your launch/yaml file. Shutting down.");
            ros::shutdown();
            return;    // 立即退出构造函数
        }
        // =====================> END: FINAL CRASH FIX <=====================
        map_generator_ = std::make_shared<map_generator::MapGenerator>(map_param_);
        map_generator_->GenerateMap();
        planner_.GetRefLine(map_generator_->roads());
        auto input_reference = common::data::DiscretizedTrajectory(planner_.traj_ref());
        my_env_.road = input_reference.data();
        Trajectory outer_traj;
        planner_.PushPoints(input_reference.data(), -0.35, outer_traj);
        auto outer_dis_traj = common::data::DiscretizedTrajectory(outer_traj);
        std::vector<common::math::Vec2d> outer_points;
        for (size_t i = 0; i < outer_dis_traj.data().size(); ++i) {
            common::math::Vec2d tmp_vec;
            tmp_vec.set_x(outer_dis_traj.data().at(i).x);
            tmp_vec.set_y(outer_dis_traj.data().at(i).y);
            outer_points.emplace_back(tmp_vec);
        }
        my_env_.points = outer_points;
        planner_.env()->outer_points.SetPoints(outer_points);

        Trajectory inner_traj;
        planner_.PushPoints(input_reference.data(), 0.40, inner_traj);
        auto inner_dis_traj = common::data::DiscretizedTrajectory(inner_traj);
        std::vector<common::math::Vec2d> inner_points;
        for (size_t i = 0; i < inner_dis_traj.data().size(); ++i) {
            common::math::Vec2d tmp_vec;
            tmp_vec.set_x(inner_dis_traj.data().at(i).x);
            tmp_vec.set_y(inner_dis_traj.data().at(i).y);
            inner_points.emplace_back(tmp_vec);
        }
        my_env_.points.insert(my_env_.points.end(), inner_points.begin(), inner_points.end());
        my_env_.Save(env_file_);

        size_t inner_step = inner_points.size() / (inner_points_num_ - 1);

        for (size_t i = 0; i < inner_points.size(); i += inner_step) {
            planner_.env()->inner_corners.push_back(inner_points.at(i));
        }
        planner_.env()->inner_polygon = Polygon2d(planner_.env()->inner_corners);

        double outer_s = 0.0;
        for (size_t i = 1; i < outer_points.size(); ++i) {
            outer_s += hypot(outer_points.at(i).x() - outer_points.at(i - 1).x(), outer_points.at(i).y() - outer_points.at(i - 1).y());
        }
        double inner_s = 0.0;
        for (size_t i = 1; i < inner_points.size(); ++i) {
            inner_s += hypot(inner_points.at(i).x() - inner_points.at(i - 1).x(), inner_points.at(i).y() - inner_points.at(i - 1).y());
        }

        // 添加对 inner_s 的检查，防止除以零
        if (std::abs(inner_s) < 1e-6) {
            ROS_FATAL(
                "Critical map generation error: The calculated length of the inner path ('inner_s') is zero. Cannot proceed. Shutting "
                "down.");
            ros::shutdown();
            return;
        }

        int outer_points_num = round(outer_s / inner_s * inner_points_num_);
        if (outer_points_num <= 1) {
            ROS_FATAL(
                "Critical calculation error: 'outer_points_num' was calculated as %d, but must be > 1. This might be due to a malformed "
                "map or parameters. Shutting down.",
                outer_points_num);
            ros::shutdown();
            return;
        }
        size_t outer_step = outer_points.size() / (outer_points_num - 1);
        for (size_t i = 0; i < outer_points.size(); i += outer_step) {
            planner_.env()->outer_corners.emplace_back(outer_points.at(i));
        }

        VisualizationPlot::PlotPolygon(planner_.env()->inner_polygon, 0.01, Color::Red, 1, "inner polygon");

        // Sudden events
        nh_.param("extend_s_length", extend_s_length_, -999.0);
        nh_.param("all_s_length", all_s_length_, -999.0);
        nh_.param("sample_accuracy", sample_accuracy_, -999.0);
        nh_.param("replan_traj_resolution", replan_traj_resolution_, -999.0);
        nh_.param("replan_stitch_time", replan_stitch_time_, -999.0);
        nh_.param("search_carrot_step", search_carrot_step_, -999);
        nh_.param("gene_obs_min_time", gene_obs_min_time_, -999.0);
        nh_.param("gene_obs_max_time", gene_obs_max_time_, -999.0);

        // Experiment
        nh_.param("robot_id", robot_id_, -999);
        robot_id_string_ = std::to_string(robot_id_);
        nh_.param("tf", traj_all_time_, -999.0);
        nh_.param("nfe", traj_nfe_, -999);
        delta_time_ = traj_all_time_ / (traj_nfe_ - 1);
        nh_.param("stitch_time", stitch_time_, -999.0);

        if (my_env_.Read(env_file_)) {
            planner_.env()->reference = common::data::DiscretizedTrajectory(my_env_.road);
        }
        my_env_.inner_points = inner_points;
        my_env_.outer_points = outer_points;
        std::vector<double> ref_xs, ref_ys;
        for (int i = 0; i < my_env_.road.size(); ++i) {
            ref_xs.push_back(my_env_.road.at(i).x);
            ref_ys.push_back(my_env_.road.at(i).y);
        }
        Color c = Color::White;
        c.set_a(0.8);
        VisualizationPlot::Plot(ref_xs, ref_ys, 0.01, c, 0, "road");
        planner_.env()->points.SetPoints(my_env_.points);

#ifndef PLAY_BAG
        planner_.GenerateObstacles(planner_.env()->reference, true, -1);

        my_env_.obstacles_struct = planner_.env()->obstacles_struct;

        all_obs_num_ = planner_.env()->obstacles.size();
        if (all_obs_num_ >= 1) {
            visual_default_polygon = planner_.env()->obstacles.front().GetPolygon();
        } else {
            common::math::Vec2d c{0.0, 0.0};
            auto box = common::math::AABox2d(c, 0.1, 0.1);
            visual_default_polygon = common::math::Polygon2d(common::math::Box2d(box));
        }

        my_env_.all_obstacles_num = planner_.env()->obstacles.size();
        my_env_.obstacles = planner_.env()->obstacles;
#endif

        Vec2d virtual_center = {0.0, 0.0};
        auto virtual_polygon = common::math::AABox2d(virtual_center, 0.1, 0.1);
        vis_virtual_polygon_ = common::math::Polygon2d(common::math::Box2d(virtual_polygon));
        my_env_.Visualize();

        max_steering_ = planner_.env()->vehicle.max_phi;
        min_steering_ = planner_.env()->vehicle.min_phi;
        pre_max_steering_ = max_steering_;
        pre_min_steering_ = min_steering_;

        time_gene_obs_ = std::uniform_real_distribution<double>(gene_obs_min_time_, gene_obs_max_time_);

        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        std::string home_dir = std::getenv("HOME");

        nh_.param("algorithm_id", algorithm_id_, 1);
        nh_.param("start_x", start_x_, 0.0);
        nh_.param("start_y", start_y_, 0.0);
        nh_.param("start_theta", start_theta_, 0.0);
        nh_.param("shutdown_mileage_threshold", shutdown_mileage_threshold_, -1.0);
        nh_.param("random_seed", (int&)node_random_seed_, 54321);

        node_random_generator_.seed(node_random_seed_);
        ROS_INFO("PlannerNode start pose generator initialized with random seed: %u", node_random_seed_);

        double perturb_range_x, perturb_range_y, perturb_range_theta_deg;
        nh_.param("start_pos_perturb_range_x", perturb_range_x, 0.03);
        nh_.param("start_pos_perturb_range_y", perturb_range_y, 0.03);
        nh_.param("start_pos_perturb_range_theta_deg", perturb_range_theta_deg, 5.0);

        start_x_dist_ = std::uniform_real_distribution<double>(-perturb_range_x, perturb_range_x);
        start_y_dist_ = std::uniform_real_distribution<double>(-perturb_range_y, perturb_range_y);
        double perturb_range_theta_rad = perturb_range_theta_deg * M_PI / 180.0;
        start_theta_dist_ = std::uniform_real_distribution<double>(-perturb_range_theta_rad, perturb_range_theta_rad);

        ROS_INFO("Start pose will be perturbed within: X[+/-%.2f], Y[+/-%.2f], Theta[+/-%.2f deg]", perturb_range_x, perturb_range_y,
                 perturb_range_theta_deg);

        start_x_ += start_x_dist_(node_random_generator_);
        start_y_ += start_y_dist_(node_random_generator_);
        start_theta_ += start_theta_dist_(node_random_generator_);

        nh_.param("real_agv_enabled", real_agv_enabled_, true);
        nh_.param("real_agv_enabled", use_real_agv_, true);

        original_obstacle_count_ = planner_.env()->obstacles.size();
        if (use_real_agv_) {
            ROS_INFO("Using REAL AGV mode - AGVs will be treated as dynamic obstacles from Nokov data");
            ROS_INFO("Emergency stop functionality ENABLED (real vehicle mode)");
            if (real_agv_enabled_ && map_generator_) {
                ROS_INFO("Real AGV trajectory generator initialized");
            }
        } else {
            ROS_INFO("Using SIMULATION mode - Emergency stop functionality DISABLED");
            bool agv_sim_enabled = false;
            nh_.param("agv_simulation_enabled", agv_sim_enabled, false);

            if (agv_sim_enabled && map_generator_) {
                ROS_INFO("Using SIMULATED AGV mode - treating slow AGVs as static obstacles");

                try {
                    agv_simulator_ = std::make_shared<AGVSimulator>(nh_, map_generator_);

                    if (agv_simulator_->IsEnabled()) {
                        ROS_INFO("AGV simulation started - treating slow AGVs as static obstacles");
                        ROS_INFO("AGV speed: 5mm/s, Main vehicle speed: ~200mm/s");
                    } else {
                        ROS_WARN("AGV simulator failed to initialize");
                        agv_simulator_.reset();
                    }
                } catch (const std::exception& e) {
                    ROS_ERROR("Failed to create AGV simulator: %s", e.what());
                    agv_simulator_.reset();
                }
            }
        }
        original_max_velocity_ = planner_.env()->vehicle.max_velocity;
    }

    // This is a new central function to handle vehicle state updates
    void UpdateVehicleState(const Pose& new_pose) {
        if (has_previous_pose_) {
            double distance_increment = std::hypot(new_pose.x() - previous_pose_.x(), new_pose.y() - previous_pose_.y());
            traveled_distance_ += distance_increment;
        }
        previous_pose_ = new_pose;
        has_previous_pose_ = true;

        // Record and draw the historical path
        historical_path_.push_back(new_pose);
    }

    // Centralized, thread-safe function to set the start pose
    void SetStartPose(const Pose& pose) {
        // Lock the mutex to protect shared data from race conditions
        boost::mutex::scoped_lock lock(pose_mutex_);

        current_pose_ = pose;

        planner_.set_start_pose(current_pose_);
        auto current_state = TrajectoryPoint(current_pose_);
        // 设置初始速度为一个小的正值，避免从0开始无法加速
        current_state.v = 0.05;  // 初始速度 5cm/s
        current_state.t = 0.0;   // 初始时间戳为0
        planner_.set_current_state(current_state);

        ROS_WARN("[DEBUG-INIT] Initial state set: pos=(%.3f,%.3f,%.3f), v=%.3f, t=%.3f",
            current_state.x, current_state.y, current_state.theta, current_state.v, current_state.t);

        common::data::Trajectory ref_line;
        planner_.GenerateRefLine(planner_.env()->reference, current_pose_, ref_line);
        planner_.set_traj_ref(ref_line);
        current_pose_received_ = true;    // This flag triggers the planner

        // Visualization
        Box2d foot_print = planner_.env()->vehicle.GenerateBox(Pose(planner_.current_state()));
        VisualizationPlot::PlotPolygon(Polygon2d(foot_print), 0.01, Color::Green, 1, "vehicle");
        VisualizationPlot::PlotVehicleHeading(foot_print, Color::Green, 0.01, 1, "vehicle heading");
        VisualizationPlot::PlotBox(foot_print, Color::Green_translucent, 1, "vehicle fill");
        VisualizationPlot::PlotPoints({current_pose_.x()}, {current_pose_.y()}, 0.02, Color::Green, 10, "current position");

        VisualizationPlot::Trigger();

        ROS_INFO("Start pose set to (X: %.2f, Y: %.2f, Theta: %.2f)", pose.x(), pose.y(), pose.theta());
    }

    void PlanTimer(const ros::TimerEvent& evt) {
        double current_time = common::util::GetCurrentTimestamp();
        if (planner_choose_ == NORMAL) {
            double time_diff = current_time - plan_next_time_;

            if ((current_pose_received_ && (time_diff >= kMathEpsilon || cycle_count_ == 0))) {
                ++cycle_count_;

                // 正常规划逻辑
                if (!use_real_agv_) {
                    UpdateAGVObstaclesAsStatic(evt);
                }

                for (size_t i = 0; i < 10; ++i) {
                    VisualizationPlot::Plot({}, {}, 0.0, Color::White, i + 1000, "carrot global path");
                    VisualizationPlot::Plot({}, {}, 0.0, Color::White, i + 1000, "smooth global path");
                }

                planner_.env()->vehicle.max_phi = max_steering_;
                planner_.env()->vehicle.min_phi = min_steering_;
                ++planner_.env()->counter;

                // 首先调用状态机更新函数，它会处理所有的状态转换
                // 注意：急停功能只在实车模式下启用，仿真模式下跳过
                if (use_real_agv_) {
                    UpdateObstacleAvoidanceState();
                }

                // 如果处于紧急等待状态，则不进行规划，仅维持停车/等待状态
                if (use_real_agv_ && avoidance_state_ == EMERGENCY_WAITING) {
                    ROS_INFO_THROTTLE(1.0, "In EMERGENCY_WAITING state. Holding position.");

                    // MODIFICATION: If the vehicle has stopped (v~0) AND the current trajectory is almost finished,
                    // generate a new stationary trajectory to hold the position.
                    bool trajectory_finished = last_traj_.empty() || (cur_index_ >= static_cast<int>(last_traj_.size()) - 2);
                    if (planner_.current_state().v < 0.01 && trajectory_finished) {
                        GenerateStationaryWaitTrajectory();
                    }

                    // 确保轨迹被标记为“已接收”，以便能够被发送出去
                    if (!last_traj_.empty()) {
                        is_traj_receive_ = true;
                    }

                    // 设置一个较短的下一次规划时间，以便频繁检查状态
                    plan_next_time_ = common::util::GetCurrentTimestamp() + 0.05;
                    return;    // 提前返回，跳过下面的正常规划逻辑
                }

                if (planner_.RollingTimePlan(1, last_traj_, planned_result_)) {
                    // 规划成功，重置失败计数
                    consecutive_planning_failures_ = 0;

                    // 调试日志：打印规划结果的时间戳范围和速度
                    if (!planned_result_.empty()) {
                        ROS_INFO("[DEBUG-PLAN] Planning SUCCESS! traj_size=%zu, t_range=[%.3f, %.3f], v_range=[%.3f, %.3f]",
                            planned_result_.size(),
                            planned_result_.front().t, planned_result_.back().t,
                            planned_result_.front().v, planned_result_.back().v);
                    }

                    if (cycle_count_ == 1) {
                        last_traj_ = planned_result_;
                        // 第一次规划成功，重置仿真时间基准
                        is_first_send_current_time_ = true;
                        ROS_WARN("[DEBUG] First planning done, reset simulation time base");
                    } else {
                        if (!FindTrajOnCurrentIndex(cur_index_) || cur_index_ < 0 ||
                            cur_index_ >= static_cast<int>(planned_result_.size())) {
                            ROS_WARN("Invalid trajectory index found, resetting to 0");
                            cur_index_ = 0;
                        }

                        int truncate_index = 0;
                        for (truncate_index = cur_index_; truncate_index > cur_index_ - rear_num_; --truncate_index) {
                            if (truncate_index <= 0) {
                                break;
                            }
                        }

                        if (truncate_index < 0) truncate_index = 0;
                        if (truncate_index >= static_cast<int>(planned_result_.size())) {
                            truncate_index = planned_result_.empty() ? 0 : planned_result_.size() - 1;
                        }
                        last_traj_.clear();
                        if (truncate_index < static_cast<int>(planned_result_.size())) {
                            auto copy_size = planned_result_.end() - (planned_result_.begin() + truncate_index);
                            last_traj_.resize(copy_size);
                            std::copy(planned_result_.begin() + truncate_index, planned_result_.end(), last_traj_.begin());
                            planner_.CheckTrajFeasible(last_traj_);
                        }
                    }

                    is_traj_receive_ = true;
                    plan_next_time_ = common::util::GetCurrentTimestamp() + suc_rolling_time_;
                    planner_.env()->num_of_continuous_failure = 0;

                    ROS_DEBUG("Planning successful, consecutive failures reset to 0");
                    // ros::shutdown();
                } else {
                    // 规划失败，增加失败计数
                    consecutive_planning_failures_++;

                    // ROS_WARN_THROTTLE(1.0, "Planning failed - consecutive failures: %d/%d", consecutive_planning_failures_, MAX_CONSECUTIVE_FAILURES);

                    if (!last_traj_.empty()) {
                        ++planner_.env()->num_of_continuous_failure;
                        plan_next_time_ = common::util::GetCurrentTimestamp() + fail_rolling_time_;
                        ++failed_count_;

                        ROS_INFO("failed_count = %u, cycle = %u, fail_rate = %lf", failed_count_, cycle_count_,
                                 static_cast<double>(failed_count_) / cycle_count_);
                    }

                    if (cycle_count_ == 1) {
                        ros::shutdown();
                    } else {
                        // 使用上一帧轨迹继续行驶
                        if (!FindTrajOnCurrentIndex(cur_index_) || cur_index_ < 0 || cur_index_ >= static_cast<int>(last_traj_.size())) {
                            ROS_WARN("Invalid trajectory index in failure handling, resetting to 0");
                            cur_index_ = 0;
                        }

                        int truncate_index;
                        for (truncate_index = cur_index_; truncate_index > cur_index_ - rear_num_; --truncate_index) {
                            if (truncate_index <= 0) {
                                break;
                            }
                        }

                        if (truncate_index < 0) truncate_index = 0;
                        if (truncate_index >= static_cast<int>(last_traj_.size())) {
                            truncate_index = last_traj_.empty() ? 0 : last_traj_.size() - 1;
                        }

                        Trajectory tmp_traj = last_traj_;
                        last_traj_.clear();

                        if (truncate_index < static_cast<int>(tmp_traj.size())) {
                            auto copy_size = tmp_traj.end() - (tmp_traj.begin() + truncate_index);
                            last_traj_.resize(copy_size);
                            std::copy(tmp_traj.begin() + truncate_index, tmp_traj.end(), last_traj_.begin());
                        }
                    }
                }

                // 确保可视化是最新的
                if (agv_simulator_ && agv_simulator_->IsEnabled()) {
                    my_env_.Visualize();
                }

                common::data::Trajectory ref_line;
                planner_.GenerateRefLine(planner_.env()->reference, Pose(planner_.current_state()), ref_line);
                planner_.set_traj_ref(ref_line);

                ROS_INFO("current max phi = %lf, current min phi = %lf", planner_.env()->vehicle.max_phi, planner_.env()->vehicle.min_phi);
            }

            if (is_traj_receive_) {
                // PublishRecordingData();    // Publish data for recording
            }
        }
    }

    void RealMotionTimer(const ros::TimerEvent& evt) {
        if (is_traj_receive_ && planner_choose_ == NORMAL) {
            double current_time = common::util::GetCurrentTimestamp();
            if (current_time > send_traj_next_time_) {
                send_traj_next_time_ = common::util::GetCurrentTimestamp() + send_traj_duration_;
                SendTrajectory(robot_id_, last_traj_);

                // 添加边界检查
                if (FindTrajOnCurrentIndex(cur_index_) && cur_index_ >= 0 && cur_index_ < static_cast<int>(last_traj_.size())) {
                    double lateral_error = hypot(planner_.current_state().x - last_traj_.at(cur_index_).x,
                                                 planner_.current_state().y - last_traj_.at(cur_index_).y);
                    if (last_traj_.back().t - last_traj_.at(cur_index_).t < traj_minimum_remaining_time_) {
                        GenerateStationaryWaitTrajectory();
                    }
                } else {
                    ROS_WARN("Invalid cur_index_ in RealMotionTimer: %d, last_traj_.size(): %zu", cur_index_, last_traj_.size());
                }
            }
            // Add recording data publishing
            // PublishRecordingData();
        }
    }

    void SimulatedMotionTimer(const ros::TimerEvent& evt) {
        if (is_traj_receive_ && planner_choose_ == NORMAL) {
            double current_time = common::util::GetCurrentTimestamp();
            if (is_first_send_current_time_) {
                first_current_time_ = current_time;
                is_first_send_current_time_ = false;
                ROS_WARN("[DEBUG] First send time initialized: first_current_time_ = %.3f", first_current_time_);
            }

            if (current_time > send_traj_next_time_) {
                send_traj_next_time_ = common::util::GetCurrentTimestamp() + send_traj_duration_;

                double elapsed_time = current_time - first_current_time_;
                ROS_INFO_THROTTLE(1.0, "[DEBUG-SIM] elapsed_time=%.3f, traj_size=%zu, traj_t_range=[%.3f, %.3f]",
                    elapsed_time, last_traj_.size(),
                    last_traj_.empty() ? -1.0 : last_traj_.front().t,
                    last_traj_.empty() ? -1.0 : last_traj_.back().t);

                bool found_index = false;
                for (int i = 0; i < last_traj_.size() - 1; ++i) {
                    if (last_traj_.at(i).t > elapsed_time) {
                        cur_index_ = i;
                        auto current_state = last_traj_.at(i);
                        planner_.set_current_state(last_traj_.at(i));
                        found_index = true;
                        ROS_INFO_THROTTLE(1.0, "[DEBUG-SIM] Found index=%d, traj_t=%.3f, pos=(%.3f,%.3f), v=%.3f",
                            i, last_traj_.at(i).t, current_state.x, current_state.y, current_state.v);

                        // Use the new central update function
                        UpdateVehicleState(Pose(current_state));

                        if (last_traj_.back().t - last_traj_.at(cur_index_).t < traj_minimum_remaining_time_) {
                            GenerateStationaryWaitTrajectory();
                        }

                        Box2d foot_print = planner_.env()->vehicle.GenerateBox(Pose(planner_.current_state()));

                        VisualizationPlot::PlotPolygon(Polygon2d(foot_print), 0.01, Color::Green, 1, "vehicle");
                        VisualizationPlot::PlotVehicleHeading(foot_print, Color::Green, 0.01, 1, "vehicle heading");
                        VisualizationPlot::PlotBox(foot_print, Color::Green_translucent, 1, "vehicle fill");

                        std::vector<double> xs_result, ys_result;
                        for (int j = i; j < last_traj_.size(); ++j) {
                            xs_result.push_back(last_traj_.at(j).x);
                            ys_result.push_back(last_traj_.at(j).y);
                        }
                        VisualizationPlot::Plot(xs_result, ys_result, 0.01, Color::Green, 9, "traj_result");
                        VisualizationPlot::Trigger();
                        break;
                    }
                }

                if (!found_index) {
                    // ROS_WARN_THROTTLE(1.0, "[DEBUG-SIM] NO INDEX FOUND! elapsed_time=%.3f exceeds all traj timestamps. Vehicle stuck!");
                }
            }
        } else {
            if (!is_first_info_) {
                ROS_INFO("Wait First Plan");
                is_first_info_ = true;
            }
        }
    }

    void NokovCallback(const std_msgs::StringConstPtr& msg) {
        json obj;
        try {
            obj = json::parse(msg->data);
        } catch (std::exception& ex) {
            ROS_ERROR("parsing json: %s", ex.what());
            return;
        }

        if (obj["vehicles"].contains(robot_id_string_)) {
            std::vector<double> pose;
            obj["vehicles"][robot_id_string_].get_to(pose);
            current_pose_ = Pose(pose[0] / 1000.0, pose[1] / 1000.0, pose[2]);
            planner_.set_start_pose(current_pose_);
            TrajectoryPoint current_state = TrajectoryPoint(current_pose_);
            planner_.set_current_state(current_state);

            Box2d foot_print = planner_.env()->vehicle.GenerateBox(Pose(planner_.current_state()));

            VisualizationPlot::PlotPolygon(Polygon2d(foot_print), 0.01, Color::Green, 1, "vehicle");
            VisualizationPlot::PlotVehicleHeading(foot_print, Color::Green, 0.01, 1, "vehicle heading");
            VisualizationPlot::PlotBox(foot_print, Color::Green_translucent, 1, "vehicle fill");

            if (!is_first_nokov_) {
                common::data::Trajectory ref_line;
                planner_.GenerateRefLine(planner_.env()->reference, current_pose_, ref_line);
                planner_.set_traj_ref(ref_line);
                is_first_nokov_ = true;
            }
            if (is_traj_receive_) {
                FindTrajOnCurrentIndex(cur_index_);

                std::vector<double> xs, ys;
                for (int i = cur_index_; i < last_traj_.size(); ++i) {
                    xs.emplace_back(last_traj_[i].x);
                    ys.emplace_back(last_traj_[i].y);
                }
                VisualizationPlot::Plot(xs, ys, 0.01, Color::Green, 9, "traj_result");
                VisualizationPlot::Trigger();
            }

#ifndef PLAY_BAG
            planner_node::Polygon2dArray obs_msg;
            for (const auto& poly : planner_.env()->obstacles_struct) {
                planner_node::Polygon2d polygon_msg;

                for (const auto& vertex : poly.dynamic_aabb.points()) {
                    geometry_msgs::Point p;
                    p.x = vertex.x();
                    p.y = vertex.y();
                    p.z = 0.0;
                    polygon_msg.vertices.push_back(p);
                }

                obs_msg.polygons.push_back(polygon_msg);
            }
            exp_obs_pub_.publish(obs_msg);
#endif

            current_pose_received_ = true;
        }
        // 如果使用实车AGV，更新AGV障碍物位置
        if (use_real_agv_ /*&& real_agv_generator_*/) {
            UpdateRealAGVObstacles(obj);
        }
        VisualizationPlot::Trigger();

        // Publish recording data when we have fresh Nokov data
        PublishRecordingData();
    }

    void UpdateRealAGVData(const json& nokov_data) {
        boost::mutex::scoped_lock lock(agv_data_mutex_);
        // 这个函数现在只负责为优化器提供障碍物的“当前”位置快照
        planner_.env()->obstacles.resize(original_obstacle_count_);
        planner_.env()->obstacles_struct.resize(original_obstacle_count_);

        if (nokov_data.contains("move_obs")) {
            std::vector<int> agv_ids;
            nh_.getParam("real_agv_ids", agv_ids);
            for (int agv_id : agv_ids) {
                std::string key = std::to_string(agv_id);
                if (nokov_data["move_obs"].contains(key)) {
                    std::vector<double> pose_vec;
                    nokov_data["move_obs"][key].get_to(pose_vec);
                    double x = pose_vec[0] / 1000.0;
                    double y = pose_vec[1] / 1000.0;
                    double theta = pose_vec[2];

                    // 1. 更新AGV的实时位姿状态
                    auto& state = agv_states_[agv_id];
                    state.last_known_pose.set_x(x);
                    state.last_known_pose.set_y(y);
                    state.last_known_pose.set_theta(theta);
                    state.last_update_time = ros::Time::now();
                    state.pose_initialized = true;

                    // 2. 为规划器创建当前的“静态”障碍物
                    planner_.env()->obstacles.push_back(common::Obstacle(CreateCircularPolygon(x, y, agv_collision_radius_, 12)));
                }
            }
        }
    }

    // 更新实车AGV作为障碍物
    void UpdateRealAGVObstacles(const json& nokov_data) {
        // 清理之前的动态障碍物，保留原始静态障碍物
        planner_.env()->obstacles.resize(original_obstacle_count_);
        planner_.env()->obstacles_struct.resize(original_obstacle_count_);

        // 从Nokov数据中获取AGV位置并添加为障碍物
        if (nokov_data.contains("move_obs")) {
            std::vector<int> agv_ids;
            if (!nh_.getParam("real_agv_ids", agv_ids)) {
                agv_ids = {4, 5, 20};    // 默认ID
            }

            for (int agv_id : agv_ids) {
                std::string key = std::to_string(agv_id);

                if (nokov_data["move_obs"].contains(key)) {
                    std::vector<double> pose;
                    nokov_data["move_obs"][key].get_to(pose);

                    // 转换为米
                    double x = pose[0] / 1000.0;
                    double y = pose[1] / 1000.0;
                    double theta = pose[2];

                    // 创建AGV障碍物（圆形）
                    Vec2d center(x, y);
                    double agv_radius = 0.065;        // AGV半径
                    double vis_agv_radius = 0.065;    // AGV半径

                    // 创建圆形多边形
                    std::vector<Vec2d> circle_points;
                    const int num_points = 12;
                    for (int i = 0; i < num_points; ++i) {
                        double angle = 2.0 * M_PI * i / num_points;
                        double px = x + agv_radius * cos(angle);
                        double py = y + agv_radius * sin(angle);
                        circle_points.emplace_back(px, py);
                    }
                    Polygon2d agv_poly(circle_points);
                    planner_.env()->obstacles.push_back(common::Obstacle(agv_poly));

                    circle_points.clear();
                    for (int i = 0; i < num_points; ++i) {
                        double angle = 2.0 * M_PI * i / num_points;
                        double px = x + vis_agv_radius * cos(angle);
                        double py = y + vis_agv_radius * sin(angle);
                        circle_points.emplace_back(px, py);
                    }
                    Polygon2d vis_agv_poly(circle_points);

                    // 创建障碍物结构体
                    DynamicAABB obs_struct;
                    obs_struct.center_xy = center;
                    obs_struct.dynamic_aabb = vis_agv_poly;
                    obs_struct.change_time = ros::Time::now().toSec();
                    obs_struct.is_dynamic = true;    // 标记为动态
                    obs_struct.radius = agv_radius;
                    obs_struct.length = agv_radius * 2.0;
                    obs_struct.width = agv_radius * 2.0;
                    obs_struct.heading = theta;

                    planner_.env()->obstacles_struct.push_back(obs_struct);
                }
            }

            // 更新环境的障碍物数据
            // if (!planner_.env()->obstacles.empty()) {
            planner_.env()->GetObsXY(planner_.env()->obs_border_point_x, planner_.env()->obs_border_point_y);
            planner_.env()->GetObsPoints(planner_.env()->obs_points_);

            // 同步到my_env
            my_env_.obstacles = planner_.env()->obstacles;
            my_env_.obstacles_struct = planner_.env()->obstacles_struct;

            // 触发可视化更新
            my_env_.Visualize();

            for (int agv_id : agv_ids) {
                std::string key = std::to_string(agv_id);
                if (nokov_data["move_obs"].contains(key)) {
                    std::vector<double> pose_vec;
                    nokov_data["move_obs"][key].get_to(pose_vec);
                    double x = pose_vec[0] / 1000.0;
                    double y = pose_vec[1] / 1000.0;
                    double theta = pose_vec[2];

                    // 更新AGV的实时位姿状态
                    auto& state = agv_states_[agv_id];
                    state.last_known_pose.set_x(x);
                    state.last_known_pose.set_y(y);
                    state.last_known_pose.set_theta(theta);
                    state.last_update_time = ros::Time::now();
                    state.pose_initialized = true;
                }
            }
            // }
        }

        ROS_DEBUG_THROTTLE(1.0, "Updated %zu real AGV obstacles from Nokov data",
                           planner_.env()->obstacles.size() - original_obstacle_count_);
    }

    void TrajCallback(const sandbox_msgs::TrajectoryConstPtr& msg) {
        last_traj_.clear();
        for (int i = 0; i < msg->points.size(); ++i) {
            TrajectoryPoint tp;
            tp.x = msg->points[i].x;
            tp.y = msg->points[i].y;
            tp.theta = msg->points[i].yaw;
            tp.v = msg->points[i].velocity;
            tp.a = msg->points[i].acceleration;
            tp.t = msg->points[i].time;
            last_traj_.push_back(tp);
        }
        is_traj_receive_ = true;
    }

    void DynamicObsVelTimer(const ros::TimerEvent& evt) {
        if (current_pose_received_) {
            double current_time = common::util::GetCurrentTimestamp();
            auto sl = planner_.env()->reference.GetProjection({planner_.current_state().x, planner_.current_state().y});

            for (int i = 0; i < obs_num_; ++i) {
                if (planner_.env()->obstacles.empty()) {
                    break;
                }
                auto obs_s = planner_.env()->reference.data().at(planner_.obs_index().at(i)).s;
                if (sl.x() - obs_s > obs_range_.front() && sl.x() - obs_s < obs_range_.back()) {
                    double time_diff = current_time - planner_.env()->obstacles_struct.at(i).change_time;
                    if (time_diff >= obs_vel_time_) {
                        // planner_.GenerateObstacles(planner_.env()->reference, false, i);
                        planner_.LogCounter(planner_.env()->counter);
                        planner_.env()->counter = 0;
                    }
                }
            }

            for (size_t i = all_obs_num_; i < planner_.env()->obstacles.size(); ++i) {
                auto cur_ref = common::data::DiscretizedTrajectory(planner_.traj_ref());
                double cur_s = cur_ref.GetProjection({planner_.current_state().x, planner_.current_state().y}).x();
                double obs_s = cur_ref.GetProjection(planner_.env()->obstacles_struct.at(i).center_xy).x();
                if (cur_s > obs_s) {
                    planner_.env()->obstacles_struct.pop_back();
                    planner_.env()->obstacles.pop_back();
                    planner_.env()->GetObsXY(planner_.env()->obs_border_point_x, planner_.env()->obs_border_point_y);
                    planner_.env()->GetObsPoints(planner_.env()->obs_points_);
                }
            }
            my_env_.obstacles = planner_.env()->obstacles;
            my_env_.obstacles_struct = planner_.env()->obstacles_struct;
            my_env_.Visualize();
        }
    }

    void ObsCallback(const planner_node::Polygon2dArray::ConstPtr& msg) {
        planner_.env()->obstacles.clear();
        for (size_t i = 0; i < msg->polygons.size(); ++i) {
            const planner_node::Polygon2d& polygon = msg->polygons[i];
            if (polygon.vertices.empty()) {
                continue;
            }

            Polygon2d obs;
            std::vector<Vec2d> poly_points;
            for (size_t j = 0; j < polygon.vertices.size(); ++j) {
                const geometry_msgs::Point& p = polygon.vertices[j];
                Vec2d poly_point = {p.x, p.y};
                poly_points.push_back(poly_point);
            }

            obs = Polygon2d(poly_points);
            planner_.env()->obstacles.push_back(common::Obstacle(obs));
        }

        for (int i = 0; i < planner_.env()->obstacles.size(); ++i) {
            VisualizationPlot::PlotPolygon(planner_.env()->obstacles[i].GetPolygon(), 0.01, Color::Grey, i, "Obstacles");
        }
    }

    void AgvTrajectoryCallback(const sandbox_msgs::TrajectoryConstPtr& msg, int agv_id) {
        boost::mutex::scoped_lock lock(agv_data_mutex_);
        common::data::Trajectory traj;

        for (const auto& pt_msg : msg->points) {
            TrajectoryPoint tp;
            tp.x = pt_msg.x;
            tp.y = pt_msg.y;
            tp.theta = pt_msg.yaw;
            tp.v = pt_msg.velocity;
            tp.t = pt_msg.time;
            traj.push_back(tp);
        }
        int agv_sync_index = 0;
        FindClosestPointOnTrajectory(traj, agv_states_[agv_id].last_known_pose, agv_sync_index);
        for (size_t i = 0; i < traj.size(); ++i) {
            traj[i].t = traj[i].t - traj[agv_sync_index].t + planner_.current_state().t;
        }
        agv_states_[agv_id].predicted_trajectory = traj;
    }

    enum PlannerChoose { NORMAL };

    // 更新AGV障碍物 - 每次规划时获取当前快照
    void UpdateAGVObstaclesAsStatic(const ros::TimerEvent&) {
        if (use_real_agv_) {
            // 实车模式下，障碍物更新已经在NokovCallback中处理
            return;
        }
        if (!agv_simulator_ || !agv_simulator_->IsEnabled()) {
            return;
        }

        try {
            // 获取AGV当前位置的快照
            auto agv_snapshot = agv_simulator_->GetAGVObstaclesSnapshot();

            // 清理之前的动态障碍物，保留原始静态障碍物
            planner_.env()->obstacles.resize(original_obstacle_count_);
            planner_.env()->obstacles_struct.resize(original_obstacle_count_);

            // 将AGV快照添加为"静态"障碍物
            for (const auto& agv_poly : agv_snapshot) {
                // 添加到障碍物列表
                planner_.env()->obstacles.push_back(common::Obstacle(agv_poly));

                // 创建障碍物结构体
                DynamicAABB obs_struct;
                obs_struct.center_xy = agv_poly.center();
                obs_struct.dynamic_aabb = agv_poly;
                obs_struct.change_time = ros::Time::now().toSec();
                obs_struct.is_dynamic = true;
                obs_struct.radius = 0.065;
                obs_struct.length = obs_struct.radius * 2.0;
                obs_struct.width = obs_struct.radius * 2.0;
                obs_struct.heading = 0.0;

                // // 创建Box2d用于可视化
                // obs_struct.obs_box = common::math::Box2d(obs_struct.center_xy, obs_struct.heading, obs_struct.length, obs_struct.width);

                planner_.env()->obstacles_struct.push_back(obs_struct);
            }
            // 更新环境的障碍物数据
            if (!agv_snapshot.empty()) {
                planner_.env()->GetObsXY(planner_.env()->obs_border_point_x, planner_.env()->obs_border_point_y);
                planner_.env()->GetObsPoints(planner_.env()->obs_points_);

                // 同步到my_env - 这是关键！
                my_env_.obstacles = planner_.env()->obstacles;
                my_env_.obstacles_struct = planner_.env()->obstacles_struct;

                // 立即触发可视化更新
                my_env_.Visualize();
            }

            ROS_DEBUG_THROTTLE(1.0, "Updated %zu AGV obstacles as static snapshots", agv_snapshot.size());

        } catch (const std::exception& e) {
            // ROS_WARN_THROTTLE(10.0, "AGV obstacle update failed: %s", e.what());
        }
    }

    void StopCommand() {
        if (is_traj_receive_) {
            auto input_reference = common::data::DiscretizedTrajectory(planner_.traj_ref());
            Trajectory stop_traj;
            FindTrajOnCurrentIndex(cur_index_);
            planner_.GenerateStopTraj(planner_.current_state(), input_reference, stop_traj);

            last_traj_ = stop_traj;

            for (int i = 0; i < 30; ++i) {
                last_traj_.push_back(last_traj_.back());
                last_traj_.back().t += 0.1;
            }
            sandbox_msgs::Trajectory traj_msg;
            std::vector<double> xs, ys;
            traj_msg.target = robot_id_;
            traj_msg.header.frame_id = "world";
            traj_msg.header.stamp = ros::Time::now();
            for (int i = 0; i < stop_traj.size(); ++i) {
                sandbox_msgs::TrajectoryPoint tp;
                xs.emplace_back(stop_traj[i].x);
                ys.emplace_back(stop_traj[i].y);
                tp.x = stop_traj[i].x;
                tp.y = stop_traj[i].y;
                tp.yaw = stop_traj[i].theta;
                tp.velocity = stop_traj[i].v;
                tp.acceleration = stop_traj[i].a;
                tp.time = stop_traj[i].t;
                traj_msg.points.push_back(tp);
            }
            pre_traj_msg_ = traj_msg;
            traj_pub_.publish(traj_msg);
            VisualizationPlot::Plot(xs, ys, 0.01, Color::Green, 9, "traj_result");
            ROS_INFO("Stop trajectory published");
        }
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_, goal_sub_, nokov_sub_, traj_sub_, exp_obs_sub_;
    ros::Publisher traj_pub_, exp_obs_pub_;
    ros::Timer plan_timer_, real_motion_timer_, simulated_motion_timer_, dynamic_obs_timer_;
    Planner planner_;

    std::string env_file_;
    MyEnvironment my_env_;
    bool is_traj_receive_ = false;
    bool current_pose_received_ = false;
    bool is_first_info_ = false;
    Pose current_pose_;
    Trajectory planned_result_;
    std::shared_ptr<map_generator::MapGenerator> map_generator_;
    MapParam map_param_;
    int failed_count_ = 0;
    int cycle_count_ = 0;
    common::data::Trajectory last_traj_;
    double plan_next_time_ = 0.0;
    double send_traj_next_time_ = 0.0;
    double first_current_time_ = 0.0;
    bool is_first_send_current_time_ = true;

    double target_last_time_ = 0.0;
    double suc_rolling_time_ = -999.0;
    double fail_rolling_time_ = -999.0;
    double replan_success_time_ = -999.0;
    double send_traj_duration_ = -999.0;
    double replan_send_traj_duration_ = -999.0;
    std::vector<double> obs_range_ = {-999.0, -999.0};
    std::vector<double> vel_region_range_ = {-999.0, -999.0};
    int obs_num_ = -999;
    int all_obs_num_ = -999;
    double obs_vel_time_ = -999.0;
    double min_s_distance_ = -999.0;

    int num_failure_of_auto_replan_ = -999;
    double intercept_max_length_ = -999.0;
    double traj_minimum_remaining_time_ = -999.0;
    double traj_minimum_remaining_time_ratio_ = -999.0;
    int rear_num_ = -999;

    // Sudden events
    PlannerChoose planner_choose_ = NORMAL;
    Polygon2d visual_default_polygon;
    double extend_s_length_ = -999.0;
    double all_s_length_ = -999.0;
    double sample_accuracy_ = -999.0;
    common::data::Trajectory carrot_global_path_;
    double replan_traj_resolution_ = -999.0;
    double replan_stitch_time_ = -999.0;
    int search_carrot_step_ = -999;
    int end_carrot_index_;
    int end_app_index_;
    double max_steering_ = -999.0;
    double min_steering_ = -999.0;
    double pre_max_steering_ = -999.0;
    double pre_min_steering_ = -999.0;

    int inner_points_num_ = -999;

    bool is_auto_replan_ = false;
    double gene_obs_min_time_ = 15.0;
    double gene_obs_max_time_ = 30.0;
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::uniform_real_distribution<double> time_gene_obs_;
    double next_replan_time_ = 999.0;

    // experiment param
    bool is_first_nokov_ = false;
    sandbox_msgs::Trajectory pre_traj_msg_;
    double start_time_ = 0.0;
    int robot_id_ = -999;
    std::string robot_id_string_;
    double traj_all_time_ = -999.0;
    int traj_nfe_ = -999;
    double delta_time_ = -999.0;
    double stitch_time_ = -999.0;
    int cur_index_ = 0;
    Polygon2d vis_virtual_polygon_;
    bool is_replan_suc_ = true;

    // new
    boost::mutex pose_mutex_;
    double start_x_;
    double start_y_;
    double start_theta_;
    std::vector<common::math::Pose> historical_path_;

    // std::vector<common::math::Pose> historical_path_;
    double traveled_distance_ = 0.0;
    Pose previous_pose_;
    bool has_previous_pose_ = false;
    double shutdown_mileage_threshold_ = -1.0;
    int algorithm_id_;    // **NEW**: ID for this specific run

    // 用于起点随机化的成员
    unsigned int node_random_seed_;         // 节点自身的随机数种子
    std::mt19937 node_random_generator_;    // 节点的随机数引擎

    // 用于生成x, y, theta扰动的随机数分布
    std::uniform_real_distribution<double> start_x_dist_;
    std::uniform_real_distribution<double> start_y_dist_;
    std::uniform_real_distribution<double> start_theta_dist_;

    // ==========AGV仿真相关===============
    std::shared_ptr<AGVSimulator> agv_simulator_;
    ros::Timer agv_obstacle_timer_;
    size_t original_obstacle_count_ = 0;

    double approach_detection_distance_ = 1.5;
    double centerline_tolerance_ = 0.15;
    double wait_time_ = 2.0;
    int consecutive_failures_ = 0;
    int max_consecutive_failures_ = 5;
    double original_max_velocity_;

    // Publishers for rosbag recording
    ros::Publisher ego_pose_record_pub_;         // 自车位置和姿态
    ros::Publisher agv_positions_record_pub_;    // KK AGV小车位置
    ros::Publisher planned_traj_record_pub_;     // 缝合后的规划轨迹
    ros::Publisher timestamp_record_pub_;        // 时间戳记录

    // Recording control
    bool enable_rosbag_recording_ = false;    // Will be controlled by RECORD_FOR_PLAYBACK macro

    struct AGVState {
        common::data::Trajectory predicted_trajectory;
        common::math::Pose last_known_pose;
        ros::Time last_update_time;
        bool pose_initialized = false;
    };
    std::map<int, ros::Subscriber> agv_traj_subscribers_;
    std::map<int, AGVState> agv_states_;
    boost::mutex agv_data_mutex_;
    double agv_collision_radius_;

    void ResetTaskState() {
        historical_path_.clear();
        traveled_distance_ = 0.0;
        has_previous_pose_ = false;
        ROS_INFO("New task started. Traveled distance reset.");
    }

    void GetStartPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        // Clear the history when a new start pose is set
        historical_path_.clear();
        ResetTaskState();

        double current_theta = tf::getYaw(msg->pose.pose.orientation);

        current_pose_ = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, current_theta);
        ROS_WARN("Received start pose: (X: %.2f, Y: %.2f, Theta: %.2f)", current_pose_.x(), current_pose_.y(), current_pose_.theta());
        planner_.set_start_pose(current_pose_);
        auto current_state = TrajectoryPoint(current_pose_);
        planner_.set_current_state(current_state);
        common::data::Trajectory ref_line;
        planner_.GenerateRefLine(planner_.env()->reference, current_pose_, ref_line);
        planner_.set_traj_ref(ref_line);
        current_pose_received_ = true;
        Box2d foot_print = planner_.env()->vehicle.GenerateBox(Pose(planner_.current_state()));
        VisualizationPlot::PlotPolygon(Polygon2d(foot_print), 0.01, Color::Green, 1, "vehicle");
        VisualizationPlot::PlotVehicleHeading(foot_print, Color::Green, 0.01, 1, "vehicle heading");
        VisualizationPlot::PlotBox(foot_print, Color::Green_translucent, 1, "vehicle fill");
        // VisualizationPlot::PlotPoints({current_pose_.x()}, {current_pose_.y()}, 0.02, Color::Green, 10, "current position");

        VisualizationPlot::Trigger();
    }

    //     void SendTrajectory(int ID, common::data::Trajectory& traj) {
    //         sandbox_msgs::Trajectory traj_msg;
    //         std::vector<double> xs, ys;
    //         traj_msg.target = ID;
    //         traj_msg.header.frame_id = "world";
    //         traj_msg.header.stamp = ros::Time::now();
    //         for (int i = 0; i < traj.size(); ++i) {
    //             sandbox_msgs::TrajectoryPoint tp;
    //             if (i >= cur_index_) {
    //                 xs.emplace_back(traj[i].x);
    //                 ys.emplace_back(traj[i].y);
    //             }
    //             tp.x = traj[i].x;
    //             tp.y = traj[i].y;
    //             tp.yaw = traj[i].theta;
    //             tp.velocity = traj[i].v;
    //             tp.acceleration = traj[i].a;
    //             tp.time = traj[i].t;
    //             traj_msg.points.push_back(tp);
    //         }
    //         pre_traj_msg_ = traj_msg;
    // #ifndef PLAY_BAG
    //         traj_pub_.publish(traj_msg);
    // #endif

    //         VisualizationPlot::Plot(xs, ys, 0.01, Color::Green, 9, "traj_result");
    //         VisualizationPlot::Trigger();
    //     }

    void SendTrajectory(int ID, common::data::Trajectory traj) {
        // 调试日志
        if (!traj.empty()) {
            ROS_INFO_THROTTLE(2.0, "[DEBUG-SEND] SendTrajectory called: traj_size=%zu, t_range=[%.3f,%.3f], v_range=[%.3f,%.3f]",
                traj.size(), traj.front().t, traj.back().t, traj.front().v, traj.back().v);
        }

        // 1. 计算整条轨迹的不可行性向量
        std::vector<double> infeasibility_vector = planner_.CalculateInfeasibilityVector(traj);
        // 2. 根据不可行性向量，找到轨迹的安全截断点
        int safe_end_index = planner_.FindSafeTrajectoryEndIndex(infeasibility_vector);
        // 3. 确保至少发送两个点，以便控制器可以处理
        if (safe_end_index < 1) {
            ROS_WARN("[DEBUG-SEND] safe_end_index < 1, NOT sending trajectory!");
            return;
        }
        // 4. 将轨迹截断到安全的长度
        // safe_end_index 是最后一个有效点的索引，所以新的大小是 index + 1
        // if (safe_end_index < infeasibility_vector.size()) {
        //     traj.resize(safe_end_index + 1);
        // }
        // 5. 发送截断后的安全轨迹
        sandbox_msgs::Trajectory traj_msg;
        std::vector<double> xs, ys;
        traj_msg.target = ID;
        traj_msg.header.frame_id = "world";
        traj_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < safe_end_index + 1; ++i) {
            sandbox_msgs::TrajectoryPoint tp;
            if (i >= cur_index_) {
                xs.emplace_back(traj[i].x);
                ys.emplace_back(traj[i].y);
            }
            tp.x = traj[i].x;
            tp.y = traj[i].y;
            tp.yaw = traj[i].theta;
            tp.velocity = traj[i].v;
            tp.acceleration = traj[i].a;
            tp.time = traj[i].t;
            traj_msg.points.push_back(tp);
        }

        for (int i = safe_end_index + 1; i < traj.size(); ++i) {
            if (i >= cur_index_) {
                xs.emplace_back(traj[i].x);
                ys.emplace_back(traj[i].y);
            }
        }

        pre_traj_msg_ = traj_msg;
#ifndef PLAY_BAG
        traj_pub_.publish(traj_msg);
#endif

        VisualizationPlot::Plot(xs, ys, 0.01, Color::Green, 9, "traj_result");
        VisualizationPlot::Trigger();
    }

    bool FindTrajOnCurrentIndex(int& cur_index) {
        if (last_traj_.empty()) {
            cur_index = 0;
            ROS_WARN("FindTrajOnCurrentIndex: last_traj_ is empty");
            return false;
        }

        // 以第一个点初始化
        double min_distance = hypot(planner_.current_state().x - last_traj_[0].x, planner_.current_state().y - last_traj_[0].y);
        int station_index = 0;

        for (int i = 1; i < static_cast<int>(last_traj_.size()); ++i) {
            double distance = hypot(planner_.current_state().x - last_traj_[i].x, planner_.current_state().y - last_traj_[i].y);
            if (distance < min_distance) {
                min_distance = distance;
                station_index = i;
            }
        }

        // 确保不会出现负数或越界
        station_index = std::max(0, std::min(station_index, static_cast<int>(last_traj_.size()) - 1));
        cur_index = station_index;

        // 再次验证结果
        if (cur_index < 0 || cur_index >= static_cast<int>(last_traj_.size())) {
            ROS_ERROR("FindTrajOnCurrentIndex: Invalid result cur_index = %d, size = %zu", cur_index, last_traj_.size());
            cur_index = 0;
            return false;
        }

        return true;
    }

    void GenerateStitchSegment(const common::data::TrajectoryPoint& start_tp, double duration, common::data::Trajectory* result) {
        result->clear();
        Vec2d goal_vec;
        goal_vec.set_x(start_tp.x + start_tp.v * duration * std::cos(start_tp.theta));
        goal_vec.set_y(start_tp.y + start_tp.v * duration * std::sin(start_tp.theta));
        std::vector<Vec2d> path = common::math::LineSegment2d(Pose(start_tp), goal_vec).SamplePoints(replan_traj_resolution_);

        for (size_t i = 0; i < path.size(); ++i) {
            common::data::TrajectoryPoint tp;
            tp.x = path.at(i).x();
            tp.y = path.at(i).y();
            tp.theta = start_tp.theta;
            tp.v = (start_tp.v > 0.01 ? start_tp.v : 0.01);
            tp.phi = start_tp.phi;
            tp.a = 0.0;
            tp.omega = 0.0;

            if (i == 0) {
                // 检查 cur_index_ 和 last_traj_ 的有效性
                if (!last_traj_.empty() && cur_index_ >= 0 && cur_index_ < static_cast<int>(last_traj_.size())) {
                    tp.t = last_traj_.at(cur_index_).t;
                } else {
                    tp.t = 0.0;    // 使用默认时间
                    ROS_WARN("Invalid cur_index_ (%d) or empty last_traj_ (size: %zu) in GenerateStitchSegment", cur_index_,
                             last_traj_.size());
                }
            } else {
                tp.t = result->back().t + hypot(tp.x - result->back().x, tp.y - result->back().y) / tp.v;
            }

            result->push_back(tp);
        }
    }

   private:
    // 避障状态机相关变量
    enum ObstacleAvoidanceState {
        NORMAL_PLANNING,     // 正常规划状态
        EMERGENCY_WAITING    // 紧急等待状态
    };

    ObstacleAvoidanceState avoidance_state_ = NORMAL_PLANNING;
    double state_transition_time_ = 0.0;
    int consecutive_planning_failures_ = 0;
    static constexpr int MAX_CONSECUTIVE_FAILURES = 2;               // 连续失败阈值
    static constexpr double FRONT_DETECTION_DISTANCE = 0.7;          // 前方检测距离
    static constexpr double CENTERLINE_BLOCKING_TOLERANCE = 0.07;    // 中心线阻挡容忍度
    static constexpr double EMERGENCY_WAIT_TIME = 2.0;               // 紧急等待时间

    // 平滑停车相关
    bool is_emergency_stopping_ = false;
    double emergency_stop_start_time_ = 0.0;

    bool use_real_agv_ = true;         // 是否使用实车AGV
    bool real_agv_enabled_ = false;    // 实车AGV是否启用
    // std::shared_ptr<MultiAGVTrajectoryGenerator> real_agv_generator_;    // 实车AGV轨迹生成器

    std::map<int, DynamicAABB> previous_agv_states_;

    // 检测前方是否有障碍物阻挡
    bool DetectFrontObstacle(double& min_distance) {
        if (!current_pose_received_) return false;

        // 获取当前车辆在参考线上的投影
        auto current_sl = planner_.env()->reference.GetProjection({planner_.current_state().x, planner_.current_state().y});
        min_distance = std::numeric_limits<double>::max();

        // 检查所有障碍物
        for (size_t i = original_obstacle_count_; i < planner_.env()->obstacles.size(); ++i) {
            auto obstacle = planner_.env()->obstacles[i];
            auto obs_sl = planner_.env()->reference.GetProjection(obstacle.GetPolygon().center());
            double obs_distance = obs_sl.x() - current_sl.x();

            // 只考虑前方障碍物
            if (obs_distance > 0 && obs_distance < FRONT_DETECTION_DISTANCE) {
                min_distance = std::min(min_distance, obs_distance);
            }
        }

        return min_distance < FRONT_DETECTION_DISTANCE;
    }

    // 检查AGV是否在中心线附近阻挡
    bool IsAGVBlockingCenterline() {
        if (!agv_simulator_ || !agv_simulator_->IsEnabled()) return false;

        return agv_simulator_->HasAGVNearCenterline(planner_.env()->reference.data(), CENTERLINE_BLOCKING_TOLERANCE);
    }

    // 更新避障状态机
    void UpdateObstacleAvoidanceState() {
        double current_time = common::util::GetCurrentTimestamp();
        double front_obstacle_distance;
        bool has_front_obstacle = DetectFrontObstacle(front_obstacle_distance);

        // 1. 根据当前车速，动态计算安全停车所需的预判时间
        const double current_velocity = planner_.current_state().v;
        // 注意: max_deceleration 在物理上为正值，代表减速度大小
        const double max_deceleration = std::abs(planner_.env()->vehicle.max_deceleration);
        const double system_reaction_buffer = 1.0;    // 为系统延迟、执行器延迟等设置的额外缓冲时间

        double braking_time = 0.0;
        // 防止除以零
        if (max_deceleration > 1e-3) {
            braking_time = current_velocity / max_deceleration;
        }

        // 最终用于碰撞检测的时间窗口 = 物理刹车时间 + 系统反应缓冲
        const double dynamic_check_time = braking_time + system_reaction_buffer;

        // 2. 主动检查当前轨迹在动态计算出的时间窗内是否存在碰撞风险
        bool is_imminent_collision = CheckTrajectoryCollision(last_traj_, dynamic_check_time);

        switch (avoidance_state_) {
            case NORMAL_PLANNING:
                // 1. (主动安全) 预测到在安全刹车距离内即将发生碰撞
                // 或
                // 2. (被动安全) 在障碍物前持续规划失败
                if (is_imminent_collision || (consecutive_planning_failures_ >= MAX_CONSECUTIVE_FAILURES && has_front_obstacle)) {
                    avoidance_state_ = EMERGENCY_WAITING;
                    state_transition_time_ = current_time;
                    is_emergency_stopping_ = true;

                    if (is_imminent_collision) {
                        ROS_WARN(
                            "State transition: NORMAL -> EMERGENCY_WAITING. Reason: Imminent collision detected within dynamic safety time "
                            "of %.2fs!",
                            dynamic_check_time);
                    } else {
                        ROS_WARN("State transition: NORMAL -> EMERGENCY_WAITING. Reason: Max planning failures with a front obstacle.");
                    }

                    StopCommand();
                    VisualizationPlot::Plot({}, {}, 0.0, Color::Magenta, 7, "sv_result");
                    for (size_t i = 0; i < 101; ++i) {
                        VisualizationPlot::PlotPolygon(vis_virtual_polygon_, 0.0, common::util::Color::Blue, i + 100, "Corridor");
                    }
                }
                break;

            case EMERGENCY_WAITING:
                // 退出紧急状态的条件: 等待时间结束，并且前方的直接威胁已解除
                if (current_time - state_transition_time_ > EMERGENCY_WAIT_TIME) {
                    if (!has_front_obstacle) {
                        ROS_INFO("Conditions improved. Transitioning back to NORMAL_PLANNING.");
                        avoidance_state_ = NORMAL_PLANNING;
                        consecutive_planning_failures_ = 0;    // 重置失败计数
                        is_emergency_stopping_ = false;        // 清除紧急停车标记
                        last_traj_.clear();                    // 清空旧轨迹，强制进行一次全新的规划
                    } else {
                        // --- MODIFICATION START ---
                        // 如果条件未改善，不再重置计时器，仅打印日志。
                        // 这将允许系统在每个后续周期都检查恢复条件。
                        // state_transition_time_ = current_time;  // <--- REMOVED THIS LINE
                        ROS_INFO_THROTTLE(
                            2.0, "Still in EMERGENCY_WAITING. Front obstacle is still present.");    // Use THROTTLE to avoid log spam
                        // --- MODIFICATION END ---
                    }
                }
                break;
        }
    }

    // bool CheckTrajectoryCollision(const common::data::Trajectory& traj, double check_time) {
    //     if (traj.empty() || !current_pose_received_) return false;

    //     double current_time = planner_.current_state().t;
    //     double end_time = current_time + check_time;

    //     for (const auto& tp : traj) {
    //         if (tp.t < current_time) continue;
    //         if (tp.t > end_time) break;

    //         Box2d vehicle_box = planner_.env()->vehicle.GenerateBox(Pose(tp));
    //         if (planner_.env()->CheckCollision(vehicle_box)) {
    //             ROS_WARN("Collision detected at time %.2f with obstacle", tp.t);
    //             return true;
    //         }
    //     }

    //     return false;
    // }

    bool CheckTrajectoryCollision(const common::data::Trajectory& ego_traj, double check_time) {
        if (ego_traj.empty() || !current_pose_received_) return false;

        boost::mutex::scoped_lock lock(agv_data_mutex_);
        if (agv_states_.empty()) return false;

        for (size_t i = cur_index_; i < ego_traj.size(); ++i) {
            auto ego_tp = ego_traj[i];
            if (ego_tp.t - planner_.current_state().t > check_time) break;

            Box2d ego_vehicle_box = planner_.env()->vehicle.GenerateBox(Pose(ego_tp.x, ego_tp.y, ego_tp.theta));

            // 遍历所有已知的AGV
            for (auto& [agv_id, agv_state] : agv_states_) {
                if (!agv_state.pose_initialized || agv_state.predicted_trajectory.empty()) continue;

                double agv_target_time = ego_tp.t;

                TrajectoryPoint predicted_agv_tp;
                if (InterpolateTrajectoryPoint(agv_state.predicted_trajectory, agv_target_time, predicted_agv_tp)) {
                    Polygon2d agv_polygon = CreateCircularPolygon(predicted_agv_tp.x, predicted_agv_tp.y, 0.07, 12);
                    if (agv_polygon.HasOverlap(ego_vehicle_box)) {
                        ROS_WARN("PREDICTED COLLISION with AGV %d in %.2fs!", agv_id, ego_tp.t);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    Polygon2d CreateCircularPolygon(double cx, double cy, double radius, int num_vertices) {
        std::vector<Vec2d> points;
        points.reserve(num_vertices);
        double angle_step = 2.0 * M_PI / num_vertices;
        for (int i = 0; i < num_vertices; ++i) {
            double angle = i * angle_step;
            points.emplace_back(cx + radius * cos(angle), cy + radius * sin(angle));
        }
        return Polygon2d(points);
    }

    bool FindClosestPointOnTrajectory(const common::data::Trajectory& traj, const Pose& pose, int& closest_index) {
        if (traj.empty()) {
            closest_index = -1;
            return false;
        }
        double min_dist_sq = std::numeric_limits<double>::max();
        closest_index = 0;
        for (int i = 0; i < traj.size(); ++i) {
            double dist_sq = pose.distanceSquareTo(Pose(traj[i].x, traj[i].y, traj[i].theta));
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_index = i;
            }
        }
        return true;
    }

    bool InterpolateTrajectoryPoint(const common::data::Trajectory& traj, double time, TrajectoryPoint& output) {
        if (traj.size() < 2) return false;
        auto it = std::lower_bound(traj.begin(), traj.end(), time, [](const TrajectoryPoint& p, double t) { return p.t < t; });
        if (it == traj.begin()) {
            output = traj.front();
            return true;
        }
        if (it == traj.end()) {
            output = traj.back();
            return true;
        }
        const TrajectoryPoint& p1 = *(it - 1);
        const TrajectoryPoint& p2 = *it;
        double dt = p2.t - p1.t;
        if (dt < 1e-6) {
            output = p1;
            return true;
        }
        double ratio = (time - p1.t) / dt;
        output.t = time;
        output.x = p1.x + ratio * (p2.x - p1.x);
        output.y = p1.y + ratio * (p2.y - p1.y);
        double d_theta = p2.theta - p1.theta;
        while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
        while (d_theta < -M_PI) d_theta += 2.0 * M_PI;
        output.theta = p1.theta + ratio * d_theta;
        output.v = p1.v + ratio * (p2.v - p1.v);
        return true;
    }

    // 生成就地等待轨迹（车辆已停下时使用）
    void GenerateStationaryWaitTrajectory() {
        last_traj_.clear();

        // 获取当前状态
        double current_x = planner_.current_state().x;
        double current_y = planner_.current_state().y;
        double current_theta = planner_.current_state().theta;
        double current_time = planner_.current_state().t;

        // 生成就地等待的轨迹点（保持位置不变）
        int wait_points = 20;    // 2秒的等待轨迹
        double dt = 0.1;

        for (int i = 0; i < wait_points; ++i) {
            TrajectoryPoint tp;
            tp.x = current_x;
            tp.y = current_y;
            tp.theta = current_theta;
            tp.v = 0.0;
            tp.a = 0.0;
            tp.phi = 0.0;
            tp.omega = 0.0;
            tp.t = current_time + i * dt;

            last_traj_.push_back(tp);
        }

        ROS_INFO("Generated stationary wait trajectory with %d points at position (%.3f, %.3f)", wait_points, current_x, current_y);
    }

    void PublishRecordingData() {
#ifdef RECORD_FOR_PLAYBACK
        if (!enable_rosbag_recording_) return;

        ros::Time current_ros_time = ros::Time::now();

        // 1. Publish ego vehicle pose (自车位置和姿态)
        if (current_pose_received_) {
            geometry_msgs::PoseStamped ego_pose_msg;
            ego_pose_msg.header.stamp = current_ros_time;
            ego_pose_msg.header.frame_id = "world";
            ego_pose_msg.pose.position.x = planner_.current_state().x;
            ego_pose_msg.pose.position.y = planner_.current_state().y;
            ego_pose_msg.pose.position.z = 0.0;

            tf::Quaternion q = tf::createQuaternionFromYaw(planner_.current_state().theta);
            ego_pose_msg.pose.orientation.x = q.x();
            ego_pose_msg.pose.orientation.y = q.y();
            ego_pose_msg.pose.orientation.z = q.z();
            ego_pose_msg.pose.orientation.w = q.w();

            ego_pose_record_pub_.publish(ego_pose_msg);
        }

        // 2. Publish AGV positions (物理世界里的KK小车位置)
        geometry_msgs::PoseArray agv_poses_msg;
        agv_poses_msg.header.stamp = current_ros_time;
        agv_poses_msg.header.frame_id = "world";

        if (use_real_agv_) {
            // Real AGV mode - positions from Nokov
            // These will be updated in NokovCallback and stored
            for (const auto& obs : planner_.env()->obstacles) {
                if (obs.GetPolygon().points().size() > 0) {
                    geometry_msgs::Pose agv_pose;
                    auto center = obs.GetPolygon().center();
                    agv_pose.position.x = center.x();
                    agv_pose.position.y = center.y();
                    agv_pose.position.z = 0.0;

                    // AGVs are circular, orientation doesn't matter
                    agv_pose.orientation.w = 1.0;
                    agv_poses_msg.poses.push_back(agv_pose);
                }
            }
        } else if (agv_simulator_ && agv_simulator_->IsEnabled()) {
            // Simulated AGV mode
            auto agv_snapshot = agv_simulator_->GetAGVObstaclesSnapshot();
            for (const auto& agv_poly : agv_snapshot) {
                geometry_msgs::Pose agv_pose;
                auto center = agv_poly.center();
                agv_pose.position.x = center.x();
                agv_pose.position.y = center.y();
                agv_pose.position.z = 0.0;
                agv_pose.orientation.w = 1.0;
                agv_poses_msg.poses.push_back(agv_pose);
            }
        }

        if (!agv_poses_msg.poses.empty()) {
            agv_positions_record_pub_.publish(agv_poses_msg);
        }

        // 3. Publish stitched trajectory (缝合后的轨迹，车辆实际跟踪的轨迹)
        if (is_traj_receive_ && !last_traj_.empty()) {
            // 使用您自定义的 sandbox_msgs::Trajectory 消息类型
            sandbox_msgs::Trajectory traj_msg;
            traj_msg.header.stamp = current_ros_time;
            traj_msg.header.frame_id = "world";
            traj_msg.target = robot_id_;    // 设置轨迹的目标ID

            // 从当前车辆位置开始记录轨迹
            int start_idx = 0;
            if (FindTrajOnCurrentIndex(start_idx)) {
                for (int i = start_idx; i < static_cast<int>(last_traj_.size()); ++i) {
                    // 创建并填充包含完整信息的消息点
                    sandbox_msgs::TrajectoryPoint tp;
                    const auto& src_tp = last_traj_[i];
                    tp.x = src_tp.x;
                    tp.y = src_tp.y;
                    tp.yaw = src_tp.theta;
                    tp.velocity = src_tp.v;
                    tp.acceleration = src_tp.a;
                    tp.omega = src_tp.omega;    // 记录角速度
                    tp.time = src_tp.t;         // 记录时间戳

                    traj_msg.points.push_back(tp);
                }
            }

            if (!traj_msg.points.empty()) {
                planned_traj_record_pub_.publish(traj_msg);
            }
        }

        // 4. Publish timestamp for synchronization
        std_msgs::String timestamp_msg;
        timestamp_msg.data = std::to_string(common::util::GetCurrentTimestamp());
        timestamp_record_pub_.publish(timestamp_msg);
#endif
    }
};

namespace frame {
PlannerNode* g_node = nullptr;

void globalSigintHandler(int sig) {
    if (g_node) {
        ROS_INFO("Shutting down planner...");
        g_node->StopCommand();
    }
    ros::shutdown();
}
}    // namespace frame

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");

    PlannerNode global_planner;
    frame::g_node = &global_planner;
    signal(SIGINT, frame::globalSigintHandler);
    ROS_INFO("Planning started");
    ros::spin();
    return 0;
}
