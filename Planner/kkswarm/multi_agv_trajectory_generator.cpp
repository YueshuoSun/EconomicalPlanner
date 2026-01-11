#include "multi_agv_trajectory_generator.h"

#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <limits>

MultiAGVTrajectoryGenerator::MultiAGVTrajectoryGenerator(ros::NodeHandle& nh, const std::shared_ptr<map_generator::MapGenerator>& map_gen)
    : nh_(nh), map_generator_(map_gen) {
    // 读取参数
    nh_.param("real_agv_enabled", enabled_, false);
    nh_.param("num_real_agvs", num_agvs_, 3);

    // ** MODIFICATION: Increased the base velocity to a more realistic value. **
    // The original value of 0.01 m/s was too slow, providing a poor reference for the controller's feedforward term.
    // A value of 0.15 m/s is more dynamic and better suited for the controller's limits (max 0.2 m/s).
    // nh_.param("real_agv_velocity", base_agv_velocity_, 0.005);
    nh_.param("real_agv_velocity", base_agv_velocity_, 0.05);

    nh_.param("real_agv_radius", agv_radius_, 0.065);

    nh_.param("max_trajectory_velocity", max_v_, 0.10);
    nh_.param("max_trajectory_acceleration", max_a_, 0.05);
    nh_.param("max_trajectory_omega", max_omega_, 0.75);

    // 读取每个AGV的固定横向偏移参数
    std::vector<double> default_offsets = {0.0, -0.08, 0.02};
    nh_.param("real_agv_lateral_offsets", agv_lateral_offsets_, default_offsets);

    // 确保偏移数组大小与AGV数量匹配
    if (agv_lateral_offsets_.size() < num_agvs_) {
        ROS_WARN("Not enough lateral offsets specified. Filling with zeros.");
        while (agv_lateral_offsets_.size() < num_agvs_) {
            agv_lateral_offsets_.push_back(0.0);
        }
    }

    // 读取AGV ID列表
    if (!nh_.getParam("real_agv_ids", agv_ids_)) {
        agv_ids_ = {1, 5, 7};    // 默认ID
    }

    // 确保AGV数量匹配
    if (agv_ids_.size() != num_agvs_) {
        ROS_WARN("Real AGV ID list size doesn't match num_agvs, adjusting...");
        num_agvs_ = agv_ids_.size();
    }

    if (!enabled_) {
        ROS_INFO("Real AGV trajectory generation disabled");
        return;
    }

    // 修改轨迹参数 - 使用长轨迹，事件触发模式
    nh_.param("real_trajectory_coverage", trajectory_coverage_, 0.75);    // 发送3/4圈轨迹
    nh_.param("real_trajectory_overlap", trajectory_overlap_, 0.25);      // 重叠1/4圈
    nh_.param("real_trajectory_dt", trajectory_dt_, 0.01);                // 时间步长10ms
    nh_.param("real_publish_frequency", publish_frequency_, 10.0);        // 检查频率1Hz（用于判断是否需要发新轨迹）

    // 使用传入的map_generator初始化参考线
    if (!map_generator_) {
        ROS_ERROR("Map generator is null for real AGV trajectory generation");
        enabled_ = false;
        return;
    }

    if (!InitializeReferenceLine()) {
        ROS_ERROR("Failed to initialize reference line for real AGV trajectory generation");
        enabled_ = false;
        return;
    }

    // 初始化AGV状态（但不设置初始位置，等待Nokov数据）
    InitializeAGVs();

    // 初始化发布器 - 为每个AGV创建独立的轨迹发布器
    for (int i = 0; i < num_agvs_; ++i) {
        int agv_id = agv_ids_[i];
        std::string topic = "/robot_" + std::to_string(agv_id) + "/trajectory";
        trajectory_pubs_[agv_id] = nh_.advertise<sandbox_msgs::Trajectory>(topic, 1);
        ROS_INFO("Created trajectory publisher for real robot_%d on topic: %s", agv_id, topic.c_str());
    }

    // 初始化可视化发布器
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/real_agv_trajectories_markers", 1);
    debug_pub_ = nh_.advertise<std_msgs::String>("/real_agv_debug", 1);

    // 订阅Nokov定位信息
    nokov_sub_ = nh_.subscribe("/nokov_info", 10, &MultiAGVTrajectoryGenerator::nokovCallback, this);

    // 定时器 - 用于检查是否需要发送新轨迹
    publish_timer_ =
        nh_.createTimer(ros::Duration(1.0 / publish_frequency_), &MultiAGVTrajectoryGenerator::checkAndPublishTrajectories, this);

    // 调试定时器
    debug_timer_ = nh_.createTimer(ros::Duration(0.5), &MultiAGVTrajectoryGenerator::publishDebugInfo, this);

    ROS_INFO("Real Multi-AGV Trajectory Generator started");
    ROS_INFO("Base AGV velocity set to %.3f m/s", base_agv_velocity_);
    ROS_INFO("Waiting for Nokov data to initialize AGV positions...");
}

bool MultiAGVTrajectoryGenerator::InitializeReferenceLine() {
    if (!map_generator_ || map_generator_->roads().empty()) {
        ROS_ERROR("Map generator or roads not available");
        return false;
    }

    reference_points_ = map_generator_->roads()[0].points;
    if (reference_points_.size() < 2) {
        ROS_ERROR("Reference line has too few points");
        return false;
    }

    // 检查并闭合参考线
    auto& first_point = reference_points_.front();
    auto& last_point = reference_points_.back();
    double gap = std::hypot(first_point.x - last_point.x, first_point.y - last_point.y);

    if (gap > 0.05) {
        ROS_INFO("Closing reference line gap of %.3f m", gap);
        int num_closure_points = static_cast<int>(ceil(gap / 0.02));
        for (int i = 1; i < num_closure_points; ++i) {
            double ratio = static_cast<double>(i) / num_closure_points;
            common::data::TrajectoryPoint closure_point;
            closure_point.x = last_point.x + ratio * (first_point.x - last_point.x);
            closure_point.y = last_point.y + ratio * (first_point.y - last_point.y);
            closure_point.theta = atan2(first_point.y - last_point.y, first_point.x - last_point.x);
            reference_points_.push_back(closure_point);
        }
    }

    // 计算总长度
    total_length_ = 0.0;
    for (size_t i = 1; i < reference_points_.size(); ++i) {
        double dx = reference_points_[i].x - reference_points_[i - 1].x;
        double dy = reference_points_[i].y - reference_points_[i - 1].y;
        total_length_ += std::hypot(dx, dy);
    }

    // 添加闭合段长度
    if (reference_points_.size() >= 2) {
        auto& final_point = reference_points_.back();
        auto& start_point = reference_points_.front();
        double closure_length = std::hypot(start_point.x - final_point.x, start_point.y - final_point.y);
        total_length_ += closure_length;
    }

    ROS_INFO("Reference line initialized: %zu points, %.2f meters total", reference_points_.size(), total_length_);

    return true;
}

void MultiAGVTrajectoryGenerator::InitializeAGVs() {
    agvs_.resize(num_agvs_);

    for (int i = 0; i < num_agvs_; ++i) {
        auto& agv = agvs_[i];
        agv.id = agv_ids_[i];
        agv.s_position = -1.0;    // 标记为未初始化
        agv.base_velocity = base_agv_velocity_;
        agv.s_velocity = base_agv_velocity_;
        agv.is_active = false;    // 初始为不活动，等待Nokov数据
        agv.lateral_offset = agv_lateral_offsets_[i];
        agv.last_trajectory_end_s = -1.0;                // 初始化轨迹结束位置
        agv.last_trajectory_sent_time = ros::Time(0);    // 初始化发送时间

        // 设置轨迹类型和参数（保持原有设置）
        agv.trajectory_type = i % 3;
        switch (agv.trajectory_type) {
            case 0:    // 中心线带缓慢蛇形
                agv.amplitude = 0.2;
                agv.frequency = 3.0;
                agv.phase_offset = 0.0;
                break;

            case 1:    // 小蛇形
                agv.amplitude = 0.15;
                agv.frequency = 8.0;
                agv.phase_offset = (2.0 * M_PI / 3.0);
                break;

            case 2:    // 大蛇形
                agv.amplitude = 0.22;
                agv.frequency = 8.0;
                agv.phase_offset = (2.0 * M_PI / 2.0);
                break;
        }

        poses_initialized_[agv.id] = false;
        ROS_INFO("AGV %d configured, waiting for Nokov position data...", agv.id);
    }
}

void MultiAGVTrajectoryGenerator::nokovCallback(const std_msgs::StringConstPtr& msg) {
    json obj;
    try {
        obj = json::parse(msg->data);
    } catch (std::exception& ex) {
        ROS_ERROR_THROTTLE(1.0, "Parsing json failed: %s", ex.what());
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 更新每个AGV的实时位置
    if (obj.contains("move_obs")) {
        for (int i = 0; i < num_agvs_; ++i) {
            int agv_id = agv_ids_[i];
            std::string key = std::to_string(agv_id);

            if (obj["move_obs"].contains(key)) {
                std::vector<double> pose;
                obj["move_obs"][key].get_to(pose);

                // 更新当前位姿 (Nokov单位是毫米，需要转换为米)
                double x = pose[0] / 1000.0;
                double y = pose[1] / 1000.0;
                double theta = pose[2];

                current_poses_[agv_id] = common::math::Pose(x, y, theta);

                // 第一次收到位置数据，初始化AGV的s位置
                if (!poses_initialized_[agv_id]) {
                    double s = ProjectToReferenceLine(x, y);
                    agvs_[i].s_position = s;
                    agvs_[i].is_active = true;
                    agvs_[i].current_pose = current_poses_[agv_id];
                    poses_initialized_[agv_id] = true;

                    ROS_INFO("AGV %d initialized at position (%.3f, %.3f), s=%.3f", agv_id, x, y, s);

                    // 立即为这个AGV发送第一条轨迹
                    sendLongTrajectory(i);
                } else {
                    // 更新当前位姿和s位置
                    agvs_[i].current_pose = current_poses_[agv_id];
                    agvs_[i].s_position = ProjectToReferenceLine(x, y);
                }
            }
        }
    }
}

void MultiAGVTrajectoryGenerator::checkAndPublishTrajectories(const ros::TimerEvent&) {
    if (!enabled_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // 检查每个AGV是否需要发送新轨迹
    for (int i = 0; i < num_agvs_; ++i) {
        int agv_id = agv_ids_[i];

        if (!poses_initialized_[agv_id] || !agvs_[i].is_active) {
            continue;
        }

        // 判断是否需要发送新轨迹
        if (needsNewTrajectory(i)) {
            sendLongTrajectory(i);
        }
    }

    // 可视化当前状态
    visualizeAllTrajectories();
}

bool MultiAGVTrajectoryGenerator::needsNewTrajectory(int agv_index) {
    auto& agv = agvs_[agv_index];

    // 如果从未发送过轨迹，需要发送
    if (agv.last_trajectory_end_s < 0) {
        return true;
    }

    // 计算当前位置到轨迹终点的剩余距离
    double remaining_distance = agv.last_trajectory_end_s - agv.s_position;

    // 处理环绕情况
    if (remaining_distance < 0) {
        remaining_distance += total_length_;
    }

    // 如果剩余距离小于重叠区域，发送新轨迹
    double overlap_distance = trajectory_overlap_ * total_length_;

    if (remaining_distance < overlap_distance) {
        ROS_INFO("AGV %d needs new trajectory. Remaining: %.3f m, Overlap threshold: %.3f m", agv.id, remaining_distance, overlap_distance);
        return true;
    }

    return false;
}

void MultiAGVTrajectoryGenerator::sendLongTrajectory(int agv_index) {
    auto& agv = agvs_[agv_index];

    // 生成长轨迹
    auto trajectory = generateLongTrajectory(agv_index);

    if (trajectory.empty()) {
        ROS_WARN("Failed to generate trajectory for AGV %d", agv.id);
        return;
    }

    // 转换为ROS消息
    sandbox_msgs::Trajectory traj_msg;
    traj_msg.target = agv.id;
    traj_msg.header.frame_id = "world";
    traj_msg.header.stamp = ros::Time::now();

    // 使用绝对时间戳
    double base_time = ros::Time::now().toSec();

    for (size_t i = 0; i < trajectory.size(); ++i) {
        sandbox_msgs::TrajectoryPoint tp;
        tp.time = base_time + i * trajectory_dt_;    // 使用绝对时间
        tp.x = trajectory[i].x;
        tp.y = trajectory[i].y;
        tp.yaw = trajectory[i].theta;
        tp.velocity = trajectory[i].v;
        tp.acceleration = trajectory[i].a;
        tp.omega = trajectory[i].omega;
        traj_msg.points.push_back(tp);
    }

    // 发布轨迹
    trajectory_pubs_[agv.id].publish(traj_msg);

    // 更新状态
    agv.last_trajectory_sent_time = ros::Time::now();
    agv.current_trajectory = trajectory;

    // 计算并记录轨迹终点的s位置
    double trajectory_length = trajectory_coverage_ * total_length_;
    agv.last_trajectory_end_s = fmod(agv.s_position + trajectory_length, total_length_);

    ROS_INFO("Sent long trajectory for AGV %d: %zu points, covering %.2f m (%.1f%% of track)", agv.id, trajectory.size(), trajectory_length,
             trajectory_coverage_ * 100);
    ROS_INFO("  Start s: %.3f, End s: %.3f", agv.s_position, agv.last_trajectory_end_s);
}

common::data::Trajectory MultiAGVTrajectoryGenerator::generateLongTrajectory(int agv_index) {
    common::data::Trajectory trajectory;
    auto& agv = agvs_[agv_index];

    // 计算轨迹覆盖的距离和预估的点数 (这部分与您之前的逻辑相同)
    double trajectory_distance = trajectory_coverage_ * total_length_;
    double trajectory_time_estimate = trajectory_distance / agv.s_velocity;
    int num_points = static_cast<int>(ceil(trajectory_time_estimate / trajectory_dt_)) + 1;

    if (num_points <= 1) {
        ROS_WARN("点数不足，无法为 AGV %d 生成有效轨迹。", agv.id);
        return trajectory;
    }

    // --- 步骤 1: 生成所有路径点的 (x, y) 坐标 ---
    // 我们先只生成路径形状，暂时不关心朝向角和速度等信息。
    // 这里会完整地调用您之前的逻辑，保留蛇形、偏移等效果。
    std::vector<common::math::Pose> raw_poses(num_points);
    for (int i = 0; i < num_points; ++i) {
        double pt_t = i * trajectory_dt_;
        double delta_s = agv.s_velocity * pt_t;
        double future_s = fmod(agv.s_position + delta_s, total_length_);
        if (future_s < 0.0) future_s += total_length_;

        // 调用您原有的函数来获取包含蛇形/偏移效果的位姿
        // 此时，我们只关心其 x 和 y 坐标
        GetContinuousPoseFromS(future_s, agv.amplitude, agv.frequency, agv.phase_offset, agv.lateral_offset, raw_poses[i]);
    }

    // --- 步骤 2: 后处理，精确计算航向角(theta)和速度(v) ---
    // 此时我们有了完整的路径点，可以根据相邻点的位置精确计算出切线方向和瞬时速度。
    trajectory.resize(num_points);
    for (int i = 0; i < num_points; ++i) {
        auto& pt = trajectory[i];
        pt.x = raw_poses[i].x();
        pt.y = raw_poses[i].y();
        pt.t = i * trajectory_dt_;

        // 基于路径的实际切线方向来计算航向角
        if (i < num_points - 1) {
            // 通过后一个点和当前点的坐标差，计算精确的切线方向角
            pt.theta = atan2(raw_poses[i + 1].y() - pt.y, raw_poses[i + 1].x() - pt.x);
        } else if (i > 0) {
            pt.theta = trajectory[i - 1].theta;    // 最后一个点沿用前一个点的朝向
        } else {
            pt.theta = 0.0;    // 孤立点
        }

        // 基于实际位移计算瞬时速度
        if (i > 0) {
            double dist = std::hypot(pt.x - trajectory[i - 1].x, pt.y - trajectory[i - 1].y);
            pt.v = dist / trajectory_dt_;
        } else {
            pt.v = agv.s_velocity;    // 轨迹起始速度使用基准速度
        }
    }

    // --- 步骤 3: 再次遍历，计算加速度(a)和角速度(omega)，并应用限制 ---
    for (int i = 0; i < num_points; ++i) {
        auto& pt = trajectory[i];

        // 计算线加速度 (a)
        if (i > 0) {
            pt.a = (pt.v - trajectory[i - 1].v) / trajectory_dt_;
        } else {
            pt.a = 0.0;
        }

        // 计算角速度 (omega)
        if (i > 0) {
            double dtheta = pt.theta - trajectory[i - 1].theta;
            // 必须处理角度的跳变 (-PI 到 PI)
            while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
            while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
            pt.omega = dtheta / trajectory_dt_;
        } else {
            pt.omega = 0.0;
        }

        // **关键新增：应用你在构造函数中设置的运动学上限**
        pt.v = std::max(-max_v_, std::min(pt.v, max_v_));
        pt.a = std::max(-max_a_, std::min(pt.a, max_a_));
        pt.omega = std::max(-max_omega_, std::min(pt.omega, max_omega_));
    }

    // 平滑第一个点的速度值，避免控制突变
    if (num_points > 1) {
        trajectory[0].v = trajectory[1].v;
        trajectory[0].a = trajectory[1].a;
        trajectory[0].omega = trajectory[1].omega;
    }

    return trajectory;
}

void MultiAGVTrajectoryGenerator::publishDebugInfo(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);

    json debug_info;

    for (int i = 0; i < num_agvs_; ++i) {
        auto& agv = agvs_[i];
        if (!agv.is_active) continue;

        json agv_info;
        agv_info["id"] = agv.id;
        agv_info["active"] = agv.is_active;
        agv_info["current_s"] = agv.s_position;
        agv_info["current_x"] = agv.current_pose.x();
        agv_info["current_y"] = agv.current_pose.y();
        agv_info["current_theta"] = agv.current_pose.theta();
        agv_info["last_traj_end_s"] = agv.last_trajectory_end_s;
        agv_info["traj_points"] = agv.current_trajectory.size();

        if (agv.last_trajectory_end_s >= 0) {
            double remaining = agv.last_trajectory_end_s - agv.s_position;
            if (remaining < 0) remaining += total_length_;
            agv_info["remaining_distance"] = remaining;
            agv_info["overlap_threshold"] = trajectory_overlap_ * total_length_;
        }

        debug_info["agvs"].push_back(agv_info);
    }

    debug_info["total_track_length"] = total_length_;
    debug_info["trajectory_coverage"] = trajectory_coverage_;
    debug_info["trajectory_overlap"] = trajectory_overlap_;

    std_msgs::String debug_msg;
    debug_msg.data = debug_info.dump(2);
    debug_pub_.publish(debug_msg);
}

void MultiAGVTrajectoryGenerator::visualizeAllTrajectories() {
    visualization_msgs::MarkerArray marker_array;

    // 清除旧的markers
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    int marker_id = 0;

    // 1. 可视化参考线（灰色）
    visualization_msgs::Marker ref_line;
    ref_line.header.frame_id = "world";
    ref_line.header.stamp = ros::Time::now();
    ref_line.ns = "reference_line";
    ref_line.id = marker_id++;
    ref_line.type = visualization_msgs::Marker::LINE_STRIP;
    ref_line.action = visualization_msgs::Marker::ADD;
    ref_line.pose.orientation.w = 1.0;
    ref_line.scale.x = 0.015;    // 加粗一点
    ref_line.color.r = 0.5;
    ref_line.color.g = 0.5;
    ref_line.color.b = 0.5;
    ref_line.color.a = 0.5;
    // ref_line.lifetime = ros::Duration(0.5);    // 设置生命周期

    for (const auto& pt : reference_points_) {
        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.0;
        ref_line.points.push_back(p);
    }

    // 闭合参考线
    if (!reference_points_.empty()) {
        geometry_msgs::Point p;
        p.x = reference_points_.front().x;
        p.y = reference_points_.front().y;
        p.z = 0.0;
        ref_line.points.push_back(p);
    }

    marker_array.markers.push_back(ref_line);

    // 颜色配置
    std::vector<std::vector<float>> colors = {
        {1.0, 0.0, 0.0},    // Red
        {0.0, 1.0, 0.0},    // Green
        {0.0, 0.0, 1.0},    // Blue
        {1.0, 1.0, 0.0},    // Yellow
        {0.0, 1.0, 1.0},    // Cyan
        {1.0, 0.0, 1.0}     // Magenta
    };

    // 2. 为每个AGV可视化
    for (int i = 0; i < num_agvs_; ++i) {
        auto& agv = agvs_[i];
        if (!agv.is_active) continue;

        auto& color = colors[i % colors.size()];

        // 2.1 当前位置球体
        visualization_msgs::Marker current_sphere;
        current_sphere.header.frame_id = "world";
        current_sphere.header.stamp = ros::Time::now();
        current_sphere.ns = "agv_position_sphere";
        current_sphere.id = marker_id++;
        current_sphere.type = visualization_msgs::Marker::SPHERE;
        current_sphere.action = visualization_msgs::Marker::ADD;
        current_sphere.pose.position.x = agv.current_pose.x();
        current_sphere.pose.position.y = agv.current_pose.y();
        current_sphere.pose.position.z = 0.05;
        current_sphere.pose.orientation.w = 1.0;
        current_sphere.scale.x = 0.08;
        current_sphere.scale.y = 0.08;
        current_sphere.scale.z = 0.08;
        current_sphere.color.r = color[0];
        current_sphere.color.g = color[1];
        current_sphere.color.b = color[2];
        current_sphere.color.a = 1.0;
        // current_sphere.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(current_sphere);

        // 2.2 当前姿态箭头
        visualization_msgs::Marker pose_arrow;
        pose_arrow.header.frame_id = "world";
        pose_arrow.header.stamp = ros::Time::now();
        pose_arrow.ns = "agv_pose_arrow";
        pose_arrow.id = marker_id++;
        pose_arrow.type = visualization_msgs::Marker::ARROW;
        pose_arrow.action = visualization_msgs::Marker::ADD;

        // 设置箭头起点和终点
        geometry_msgs::Point arrow_start, arrow_end;
        arrow_start.x = agv.current_pose.x();
        arrow_start.y = agv.current_pose.y();
        arrow_start.z = 0.05;

        double arrow_length = 0.15;
        arrow_end.x = arrow_start.x + arrow_length * cos(agv.current_pose.theta());
        arrow_end.y = arrow_start.y + arrow_length * sin(agv.current_pose.theta());
        arrow_end.z = 0.05;

        pose_arrow.points.push_back(arrow_start);
        pose_arrow.points.push_back(arrow_end);

        pose_arrow.scale.x = 0.02;    // 箭头杆直径
        pose_arrow.scale.y = 0.04;    // 箭头头部直径
        pose_arrow.scale.z = 0.0;
        pose_arrow.color.r = color[0];
        pose_arrow.color.g = color[1];
        pose_arrow.color.b = color[2];
        pose_arrow.color.a = 1.0;
        // pose_arrow.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(pose_arrow);

        // 2.3 AGV ID文本
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "world";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "agv_id_text";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = agv.current_pose.x();
        text_marker.pose.position.y = agv.current_pose.y();
        text_marker.pose.position.z = 0.15;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.08;
        text_marker.color.r = color[0];
        text_marker.color.g = color[1];
        text_marker.color.b = color[2];
        text_marker.color.a = 1.0;
        text_marker.text = "AGV " + std::to_string(agv.id);
        // text_marker.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(text_marker);

        // 2.4 完整预期路径（整圈，半透明细线）
        visualization_msgs::Marker full_path;
        full_path.header.frame_id = "world";
        full_path.header.stamp = ros::Time::now();
        full_path.ns = "agv_full_path";
        full_path.id = marker_id++;
        full_path.type = visualization_msgs::Marker::LINE_STRIP;
        full_path.action = visualization_msgs::Marker::ADD;
        full_path.pose.orientation.w = 1.0;
        full_path.scale.x = 0.008;    // 细线
        full_path.color.r = color[0];
        full_path.color.g = color[1];
        full_path.color.b = color[2];
        full_path.color.a = 0.3;    // 半透明
        // full_path.lifetime = ros::Duration(0.5);

        // 生成完整路径（高密度采样）
        int num_samples = 500;    // 增加采样密度
        for (int j = 0; j <= num_samples; ++j) {
            double s = (j * total_length_) / num_samples;
            Pose pose;
            GetContinuousPoseFromS(s, agv.amplitude, agv.frequency, agv.phase_offset, agv.lateral_offset, pose);
            geometry_msgs::Point p;
            p.x = pose.x();
            p.y = pose.y();
            p.z = 0.01;
            full_path.points.push_back(p);
        }

        marker_array.markers.push_back(full_path);

        // 2.5 当前发送的轨迹段（粗线，不透明）
        if (!agv.current_trajectory.empty()) {
            visualization_msgs::Marker current_traj;
            current_traj.header.frame_id = "world";
            current_traj.header.stamp = ros::Time::now();
            current_traj.ns = "agv_current_trajectory";
            current_traj.id = marker_id++;
            current_traj.type = visualization_msgs::Marker::LINE_STRIP;
            current_traj.action = visualization_msgs::Marker::ADD;
            current_traj.pose.orientation.w = 1.0;
            current_traj.scale.x = 0.025;    // 粗线
            current_traj.color.r = color[0];
            current_traj.color.g = color[1];
            current_traj.color.b = color[2];
            current_traj.color.a = 0.9;    // 几乎不透明
            // current_traj.lifetime = ros::Duration(0.5);

            for (const auto& pt : agv.current_trajectory) {
                geometry_msgs::Point p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = 0.02;
                current_traj.points.push_back(p);
            }

            marker_array.markers.push_back(current_traj);

            // 2.6 轨迹起点和终点标记
            // 起点（绿色小球）
            visualization_msgs::Marker traj_start;
            traj_start.header.frame_id = "world";
            traj_start.header.stamp = ros::Time::now();
            traj_start.ns = "trajectory_start";
            traj_start.id = marker_id++;
            traj_start.type = visualization_msgs::Marker::SPHERE;
            traj_start.action = visualization_msgs::Marker::ADD;
            traj_start.pose.position.x = agv.current_trajectory.front().x;
            traj_start.pose.position.y = agv.current_trajectory.front().y;
            traj_start.pose.position.z = 0.03;
            traj_start.pose.orientation.w = 1.0;
            traj_start.scale.x = 0.05;
            traj_start.scale.y = 0.05;
            traj_start.scale.z = 0.05;
            traj_start.color.r = 0.0;
            traj_start.color.g = 1.0;
            traj_start.color.b = 0.0;
            traj_start.color.a = 1.0;
            // traj_start.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(traj_start);

            // 终点（红色小球）
            visualization_msgs::Marker traj_end;
            traj_end.header.frame_id = "world";
            traj_end.header.stamp = ros::Time::now();
            traj_end.ns = "trajectory_end";
            traj_end.id = marker_id++;
            traj_end.type = visualization_msgs::Marker::SPHERE;
            traj_end.action = visualization_msgs::Marker::ADD;
            traj_end.pose.position.x = agv.current_trajectory.back().x;
            traj_end.pose.position.y = agv.current_trajectory.back().y;
            traj_end.pose.position.z = 0.03;
            traj_end.pose.orientation.w = 1.0;
            traj_end.scale.x = 0.05;
            traj_end.scale.y = 0.05;
            traj_end.scale.z = 0.05;
            traj_end.color.r = 1.0;
            traj_end.color.g = 0.0;
            traj_end.color.b = 0.0;
            traj_end.color.a = 1.0;
            // traj_end.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(traj_end);
        }

        // 2.7 显示s位置信息
        visualization_msgs::Marker s_info;
        s_info.header.frame_id = "world";
        s_info.header.stamp = ros::Time::now();
        s_info.ns = "agv_s_info";
        s_info.id = marker_id++;
        s_info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        s_info.action = visualization_msgs::Marker::ADD;
        s_info.pose.position.x = agv.current_pose.x();
        s_info.pose.position.y = agv.current_pose.y() - 0.1;
        s_info.pose.position.z = 0.08;
        s_info.pose.orientation.w = 1.0;
        s_info.scale.z = 0.05;
        s_info.color.r = 1.0;
        s_info.color.g = 1.0;
        s_info.color.b = 1.0;
        s_info.color.a = 1.0;

        char info_text[256];
        snprintf(info_text, sizeof(info_text), "s=%.2f/%.2f", agv.s_position, total_length_);
        s_info.text = info_text;
        // s_info.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(s_info);
    }

    // 发布所有markers
    marker_pub_.publish(marker_array);

    ROS_DEBUG_THROTTLE(1.0, "Published %zu visualization markers", marker_array.markers.size());
}

// 保持原有的辅助函数不变
double MultiAGVTrajectoryGenerator::ProjectToReferenceLine(double x, double y) {
    // [保持原代码不变]
    double min_dist = std::numeric_limits<double>::max();
    double best_s = 0.0;
    double accumulated_s = 0.0;

    // 遍历参考线找到最近点
    for (size_t i = 1; i < reference_points_.size(); ++i) {
        const auto& p1 = reference_points_[i - 1];
        const auto& p2 = reference_points_[i];

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double segment_len = std::hypot(dx, dy);

        // 计算点到线段的投影
        double t = std::max(0.0, std::min(1.0, ((x - p1.x) * dx + (y - p1.y) * dy) / (dx * dx + dy * dy)));

        double proj_x = p1.x + t * dx;
        double proj_y = p1.y + t * dy;
        double dist = std::hypot(x - proj_x, y - proj_y);

        if (dist < min_dist) {
            min_dist = dist;
            best_s = accumulated_s + t * segment_len;
        }

        accumulated_s += segment_len;
    }

    // 处理闭合段
    if (reference_points_.size() >= 2) {
        const auto& last = reference_points_.back();
        const auto& first = reference_points_.front();

        double dx = first.x - last.x;
        double dy = first.y - last.y;
        double segment_len = std::hypot(dx, dy);

        double t = std::max(0.0, std::min(1.0, ((x - last.x) * dx + (y - last.y) * dy) / (dx * dx + dy * dy)));

        double proj_x = last.x + t * dx;
        double proj_y = last.y + t * dy;
        double dist = std::hypot(x - proj_x, y - proj_y);

        if (dist < min_dist) {
            min_dist = dist;
            best_s = accumulated_s + t * segment_len;
        }
    }

    // 归一化s到[0, total_length_)
    best_s = fmod(best_s, total_length_);
    if (best_s < 0) best_s += total_length_;

    return best_s;
}

void MultiAGVTrajectoryGenerator::GetContinuousPoseFromS(double s, double amplitude, double frequency, double phase_offset,
                                                         double lateral_offset, Pose& pose) {
    // [保持原代码不变]
    s = fmod(s, total_length_);
    if (s < 0.0) s += total_length_;

    double accumulated_s = 0.0;
    size_t idx = 0;
    bool found_segment = false;

    for (size_t i = 1; i < reference_points_.size(); ++i) {
        double dx = reference_points_[i].x - reference_points_[i - 1].x;
        double dy = reference_points_[i].y - reference_points_[i - 1].y;
        double segment_len = std::hypot(dx, dy);

        if (accumulated_s + segment_len >= s) {
            idx = i - 1;
            found_segment = true;
            break;
        }
        accumulated_s += segment_len;
    }

    if (!found_segment) {
        // 处理闭合段
        idx = reference_points_.size() - 1;
        auto& last_point = reference_points_.back();
        auto& first_point = reference_points_.front();

        double closure_dx = first_point.x - last_point.x;
        double closure_dy = first_point.y - last_point.y;
        double closure_len = std::hypot(closure_dx, closure_dy);

        if (accumulated_s + closure_len >= s) {
            double local_s = s - accumulated_s;
            double ratio = (closure_len > 1e-9) ? (local_s / closure_len) : 0.0;
            ratio = std::max(0.0, std::min(1.0, ratio));

            double center_x = last_point.x + ratio * closure_dx;
            double center_y = last_point.y + ratio * closure_dy;
            double theta = atan2(closure_dy, closure_dx);

            ApplySnakeOffsetWithLateral(s, amplitude, frequency, phase_offset, lateral_offset, center_x, center_y, theta, pose);
            return;
        }
    }

    if (idx >= reference_points_.size() - 1) {
        idx = reference_points_.size() - 2;
    }

    const auto& p1 = reference_points_[idx];
    const auto& p2 = reference_points_[idx + 1];

    double local_s = s - accumulated_s;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double segment_len = std::hypot(dx, dy);

    double ratio = (segment_len > 1e-9) ? (local_s / segment_len) : 0.0;
    ratio = std::max(0.0, std::min(1.0, ratio));

    double center_x = p1.x + ratio * dx;
    double center_y = p1.y + ratio * dy;
    double theta = atan2(dy, dx);

    ApplySnakeOffsetWithLateral(s, amplitude, frequency, phase_offset, lateral_offset, center_x, center_y, theta, pose);
}

void MultiAGVTrajectoryGenerator::ApplySnakeOffsetWithLateral(double s, double amplitude, double frequency, double phase_offset,
                                                              double lateral_offset, double center_x, double center_y, double theta,
                                                              Pose& pose) {
    // [保持原代码不变]
    double normal_x = -sin(theta);
    double normal_y = cos(theta);

    // 首先应用固定的横向偏移
    double offset_x = center_x + lateral_offset * normal_x;
    double offset_y = center_y + lateral_offset * normal_y;

    if (amplitude > 1e-9 && frequency > 1e-9) {
        double normalized_s = s / total_length_;
        if (normalized_s >= 1.0 - 1e-9) normalized_s = 0.0;

        double phase = 2.0 * M_PI * frequency * normalized_s + phase_offset;
        double sin_val = sin(phase);
        double smooth_factor = 1.0 - 0.25 * sin_val * sin_val;
        double snake_offset = amplitude * sin_val * smooth_factor;

        // 在固定偏移的基础上添加蛇形偏移
        pose.set_x(offset_x + snake_offset * normal_x);
        pose.set_y(offset_y + snake_offset * normal_y);

        double cos_val = cos(phase);
        double d_smooth = cos_val * smooth_factor - 0.5 * sin_val * sin_val * cos_val;
        double d_phase = 2.0 * M_PI * frequency / total_length_;
        double d_lateral = amplitude * d_smooth * d_phase;
        d_lateral = std::max(-0.2, std::min(0.2, d_lateral));
        pose.set_theta(theta + atan(d_lateral));
    } else {
        pose.set_x(offset_x);
        pose.set_y(offset_y);
        pose.set_theta(theta);
    }
}

// 其他原有函数保持不变
std::vector<Polygon2d> MultiAGVTrajectoryGenerator::GetAGVObstaclesSnapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Polygon2d> obstacles;

    for (const auto& agv : agvs_) {
        if (!agv.is_active) continue;

        Vec2d center(agv.current_pose.x(), agv.current_pose.y());
        auto box = AABox2d(center, agv_radius_ * 2.0, agv_radius_ * 2.0);
        obstacles.push_back(Polygon2d(Box2d(box)));
    }

    return obstacles;
}

bool MultiAGVTrajectoryGenerator::HasAGVNearCenterline(const std::vector<common::data::TrajectoryPoint>& ref_line, double tolerance) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& agv : agvs_) {
        if (!agv.is_active) continue;

        if (std::abs(agv.lateral_offset) < tolerance) {
            return true;
        }
    }

    return false;
}

// 以下函数为空实现，保持接口兼容
void MultiAGVTrajectoryGenerator::UpdateSimulation(const ros::TimerEvent&) {
    // 实车模式不需要仿真更新
}

void MultiAGVTrajectoryGenerator::GenerateAndVisualizeFullPaths() {
    // 可选实现
}

void MultiAGVTrajectoryGenerator::VisualizeAllPaths() {
    // 已由visualizeAllTrajectories替代
}

void MultiAGVTrajectoryGenerator::visualizeTrajectory(int agv_id, const common::data::Trajectory& trajectory) {
    // 已由visualizeAllTrajectories替代
}

// 主函数 - 使节点可以独立运行
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "multi_agv_trajectory_generator");
    ros::NodeHandle nh("~");

    // 读取地图参数并生成地图
    MapParam map_param;
    nh.param("map_radius", map_param.map_radius, 0.65);
    nh.param("x_first_straight", map_param.x_first_straight, 0.65);
    nh.param("x_second_straight", map_param.x_second_straight, 0.05);
    nh.param("y_straight", map_param.y_straight, 0.5);
    nh.param("curve_length", map_param.curve_length, 1.105);
    nh.param("resolution", map_param.resolution, 0.02);

    auto map_generator = std::make_shared<map_generator::MapGenerator>(map_param);
    map_generator->GenerateMap();

    // 创建轨迹生成器
    MultiAGVTrajectoryGenerator generator(nh, map_generator);

    ROS_INFO("Multi-AGV Trajectory Generator Node Started");
    ROS_INFO("Publishing trajectories to /robot_X/trajectory topics");

    ros::spin();

    return 0;
}
