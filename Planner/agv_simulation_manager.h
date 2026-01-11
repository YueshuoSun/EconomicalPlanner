#pragma once

#include <ros/ros.h>

#include <mutex>
#include <vector>

#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/visualization_plot.h"
#include "map/map_generator.h"

using namespace common::math;
using common::util::Color;

class AGVSimulator {
   public:
    struct AGVState {
        int id;
        double s_position;       // 在参考线上的位置
        double base_velocity;    // 基础速度
        double s_velocity;       // 在参考线上的速度（所有AGV相同）
        Pose current_pose;       // 当前位姿
        Color color;             // 显示颜色
        bool is_active;
        int trajectory_type;                    // 轨迹类型：0-中心线，1-小蛇形，2-大蛇形
        double amplitude;                       // 蛇形振幅
        double frequency;                       // 蛇形频率
        double phase_offset;                    // 相位偏移
        double lateral_offset;                  // 横向偏移量（新增）
        double path_length_ratio;               // 路径长度与中心线长度的比值
        std::vector<Vec2d> full_path_points;    // 完整路径点
    };

    AGVSimulator(ros::NodeHandle& nh, const std::shared_ptr<map_generator::MapGenerator>& map_gen) : nh_(nh), map_generator_(map_gen) {
        // 读取参数
        nh_.param("agv_simulation_enabled", enabled_, true);
        nh_.param("num_simulated_agvs", num_agvs_, 3);
        nh_.param("agv_velocity", base_agv_velocity_, 0.07);
        nh_.param("agv_radius", agv_radius_, 0.065);

        // 读取每个AGV的固定横向偏移参数
        std::vector<double> default_offsets = {0.0, -0.08, 0.05};
        nh_.param("agv_lateral_offsets", agv_lateral_offsets_, default_offsets);

        // 确保偏移数组大小与AGV数量匹配
        if (agv_lateral_offsets_.size() < num_agvs_) {
            ROS_WARN("Not enough lateral offsets specified. Filling with zeros.");
            while (agv_lateral_offsets_.size() < num_agvs_) {
                agv_lateral_offsets_.push_back(0.0);
            }
        }

        if (!enabled_) {
            ROS_INFO("AGV simulation disabled");
            return;
        }

        if (!InitializeReferenceLine()) {
            ROS_ERROR("Failed to initialize reference line for AGV simulation");
            enabled_ = false;
            return;
        }

        InitializeAGVs();
        GenerateAndVisualizeFullPaths();

        simulation_timer_ = nh_.createTimer(ros::Duration(0.1), &AGVSimulator::UpdateSimulation, this);

        ROS_INFO("AGV simulation started with %d AGVs (%.1f mm/s centerline projection speed)", num_agvs_, base_agv_velocity_ * 1000);

        // 打印每个AGV的横向偏移
        for (int i = 0; i < num_agvs_; ++i) {
            ROS_INFO("AGV %d lateral offset: %.3f m", i, agv_lateral_offsets_[i]);
        }
    }

    // 获取AGV的当前位置快照作为静态障碍物
    std::vector<Polygon2d> GetAGVObstaclesSnapshot() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Polygon2d> obstacles;
        if (!enabled_) return obstacles;

        const double detection_radius = agv_radius_;
        for (const auto& agv : agvs_) {
            if (!agv.is_active) continue;
            Vec2d center(agv.current_pose.x(), agv.current_pose.y());
            obstacles.push_back(CreateCirclePolygon(center, detection_radius, 12));
        }
        return obstacles;
    }

    // 检查是否有AGV在中心线附近（用于避障决策）
    bool HasAGVNearCenterline(const std::vector<common::data::TrajectoryPoint>& reference_points, double tolerance = 0.2) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_ || agvs_.empty() || reference_points.empty()) return false;

        for (const auto& agv : agvs_) {
            if (!agv.is_active) continue;

            // 找到AGV在参考线上的投影点
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& ref_point : reference_points) {
                double distance = std::hypot(agv.current_pose.x() - ref_point.x, agv.current_pose.y() - ref_point.y);
                min_distance = std::min(min_distance, distance);
            }

            // 如果AGV距离中心线很近，认为它在阻挡中心线
            if (min_distance < tolerance) {
                return true;
            }
        }
        return false;
    }

    // 获取AGV作为圆形障碍物
    std::vector<Polygon2d> GetAGVObstacles() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Polygon2d> obstacles;
        if (!enabled_) return obstacles;

        for (const auto& agv : agvs_) {
            if (!agv.is_active) continue;
            std::vector<Vec2d> circle_points;
            const int num_points = 12;
            for (int i = 0; i < num_points; ++i) {
                double angle = 2.0 * M_PI * i / num_points;
                double x = agv.current_pose.x() + agv_radius_ * cos(angle);
                double y = agv.current_pose.y() + agv_radius_ * sin(angle);
                circle_points.emplace_back(x, y);
            }
            obstacles.push_back(Polygon2d(circle_points));
        }
        return obstacles;
    }

    common::math::Polygon2d CreateCirclePolygon(const Vec2d& center, double radius, int num_points = 12) {
        std::vector<Vec2d> circle_points;
        circle_points.reserve(num_points);
        for (int i = 0; i < num_points; ++i) {
            double angle = 2.0 * M_PI * i / num_points;
            double x = center.x() + radius * cos(angle);
            double y = center.y() + radius * sin(angle);
            circle_points.emplace_back(x, y);
        }
        return common::math::Polygon2d(circle_points);
    }

    std::vector<Pose> GetAGVPoses() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Pose> poses;
        for (const auto& agv : agvs_) {
            if (agv.is_active) {
                poses.push_back(agv.current_pose);
            }
        }
        return poses;
    }

    bool IsEnabled() const { return enabled_; }

   private:
    ros::NodeHandle& nh_;
    ros::Timer simulation_timer_;
    std::shared_ptr<map_generator::MapGenerator> map_generator_;

    std::vector<AGVState> agvs_;
    std::vector<common::data::TrajectoryPoint> reference_points_;
    double total_length_;

    std::mutex mutex_;
    bool enabled_;
    int num_agvs_;
    double base_agv_velocity_;
    double agv_radius_;

    // 每个AGV的固定横向偏移量（新增）
    std::vector<double> agv_lateral_offsets_;

    bool InitializeReferenceLine() {
        if (!map_generator_ || map_generator_->roads().empty()) {
            ROS_ERROR("Map generator or roads not available");
            return false;
        }

        reference_points_ = map_generator_->roads()[0].points;
        if (reference_points_.size() < 2) {
            ROS_ERROR("Reference line has too few points");
            return false;
        }

        auto& first_point = reference_points_.front();
        auto& last_point = reference_points_.back();
        double gap = std::hypot(first_point.x - last_point.x, first_point.y - last_point.y);

        ROS_INFO("Original reference line gap: %.6f meters", gap);

        if (gap > 0.05) {
            ROS_WARN("Reference line not closed (gap: %.6f m), adding closure segment", gap);
            int num_closure_points = static_cast<int>(ceil(gap / 0.02));
            for (int i = 1; i < num_closure_points; ++i) {
                double ratio = static_cast<double>(i) / num_closure_points;
                common::data::TrajectoryPoint closure_point;
                closure_point.x = last_point.x + ratio * (first_point.x - last_point.x);
                closure_point.y = last_point.y + ratio * (first_point.y - last_point.y);
                closure_point.theta = atan2(first_point.y - last_point.y, first_point.x - last_point.x);
                closure_point.v = first_point.v;
                reference_points_.push_back(closure_point);
            }
        }

        total_length_ = 0.0;
        for (size_t i = 1; i < reference_points_.size(); ++i) {
            double dx = reference_points_[i].x - reference_points_[i - 1].x;
            double dy = reference_points_[i].y - reference_points_[i - 1].y;
            total_length_ += std::hypot(dx, dy);
        }

        if (reference_points_.size() >= 2) {
            auto& final_point = reference_points_.back();
            auto& start_point = reference_points_.front();
            double closure_length = std::hypot(start_point.x - final_point.x, start_point.y - final_point.y);
            total_length_ += closure_length;
        }

        ROS_INFO("AGV reference line: %zu points, %.2f meters total (closed loop)", reference_points_.size(), total_length_);

        auto& final_first = reference_points_.front();
        auto& final_last = reference_points_.back();
        double final_gap = std::hypot(final_first.x - final_last.x, final_first.y - final_last.y);
        ROS_INFO("Final reference line gap: %.6f meters", final_gap);

        return true;
    }

    void InitializeAGVs() {
        agvs_.resize(num_agvs_);

        std::vector<Color> colors = {Color::Red, Color::Cyan, Color::Purple, Color::Yellow, Color::Green, Color::Magenta};

        int center_count = 0, small_wave_count = 1, large_wave_count = 1;

        for (int i = 0; i < num_agvs_; ++i) {
            auto& agv = agvs_[i];
            agv.id = 100 + i;
            agv.s_position = (total_length_ / num_agvs_) * i;
            agv.base_velocity = base_agv_velocity_;

            // 关键修改：所有AGV的s_velocity都相同，确保纵向距离不变
            agv.s_velocity = base_agv_velocity_;

            agv.color = colors[i % colors.size()];
            agv.is_active = true;

            // 使用固定的横向偏移量（从参数数组中获取）
            agv.lateral_offset = agv_lateral_offsets_[i];

            // 设置轨迹类型和参数
            agv.trajectory_type = i % 3;
            switch (agv.trajectory_type) {
                case 0:    // 中心线带缓慢蛇形
                    agv.amplitude = 0.2;
                    agv.frequency = 3.0;
                    agv.phase_offset = 0.0;
                    agv.path_length_ratio = CalculatePathLengthRatio(agv.amplitude, agv.frequency);
                    center_count++;
                    ROS_INFO("AGV %d: Center line with slow wave, lateral offset: %.3f m, s_velocity: %.3f m/s", agv.id, agv.lateral_offset,
                             agv.s_velocity);
                    break;

                case 1:    // 小蛇形
                    agv.amplitude = 0.15;
                    agv.frequency = 8.0;
                    agv.phase_offset = small_wave_count * (2.0 * M_PI / 3.0);
                    agv.path_length_ratio = CalculatePathLengthRatio(agv.amplitude, agv.frequency);
                    small_wave_count++;
                    ROS_INFO("AGV %d: Small wave (amp=%.1fcm, cycles=%.0f, phase=%.1f°, offset=%.3f m, s_vel=%.3f m/s)", agv.id,
                             agv.amplitude * 100, agv.frequency, agv.phase_offset * 180.0 / M_PI, agv.lateral_offset, agv.s_velocity);
                    break;

                case 2:    // 大蛇形
                    agv.amplitude = 0.22;
                    agv.frequency = 8.0;
                    agv.phase_offset = large_wave_count * (2.0 * M_PI / 2.5);
                    agv.path_length_ratio = CalculatePathLengthRatio(agv.amplitude, agv.frequency);
                    large_wave_count++;
                    ROS_INFO("AGV %d: Large wave (amp=%.1fcm, cycles=%.0f, phase=%.1f°, offset=%.3f m, s_vel=%.3f m/s)", agv.id,
                             agv.amplitude * 100, agv.frequency, agv.phase_offset * 180.0 / M_PI, agv.lateral_offset, agv.s_velocity);
                    break;
            }

            UpdateAGVPose(agv);
        }

        ROS_INFO("Initialized %d center, %d small wave, %d large wave AGVs with FIXED longitudinal distances", center_count,
                 small_wave_count, large_wave_count);
        ROS_INFO("All AGVs have identical centerline projection velocity: %.3f m/s", base_agv_velocity_);
    }

    double CalculatePathLengthRatio(double amplitude, double frequency) {
        if (amplitude < 1e-6) return 1.0;

        double k = amplitude * 2.0 * M_PI * frequency / total_length_;
        const int samples = 1000;
        double ds = total_length_ / samples;
        double arc_length = 0.0;

        for (int i = 0; i < samples; ++i) {
            double s = i * ds;
            double derivative = k * cos(2.0 * M_PI * frequency * s / total_length_);
            arc_length += sqrt(1.0 + derivative * derivative) * ds;
        }
        return arc_length / total_length_;
    }

    void GenerateAndVisualizeFullPaths() {
        const double path_resolution = 0.01;

        for (auto& agv : agvs_) {
            agv.full_path_points.clear();
            int num_points = static_cast<int>(ceil(total_length_ / path_resolution));

            for (int i = 0; i <= num_points; ++i) {
                double s = (i * total_length_) / num_points;
                if (i == num_points) s = 0.0;

                Pose temp_pose;
                GetContinuousPoseFromS(s, agv.amplitude, agv.frequency, agv.phase_offset, agv.lateral_offset, temp_pose);
                agv.full_path_points.emplace_back(temp_pose.x(), temp_pose.y());
            }

            ROS_INFO("Generated %zu continuous path points for AGV %d with lateral offset %.3f m", agv.full_path_points.size(), agv.id,
                     agv.lateral_offset);

            if (agv.full_path_points.size() >= 2) {
                auto& first = agv.full_path_points.front();
                auto& last = agv.full_path_points.back();
                double gap = hypot(first.x() - last.x(), first.y() - last.y());
                ROS_INFO("AGV %d path closure gap: %.6f meters", agv.id, gap);
            }
        }

        VisualizeAllPaths();
    }

    void VisualizeAllPaths() {
        for (const auto& agv : agvs_) {
            if (agv.full_path_points.empty()) continue;

            std::vector<double> path_x, path_y;
            for (const auto& point : agv.full_path_points) {
                path_x.push_back(point.x());
                path_y.push_back(point.y());
            }

            Color path_color = agv.color;
            path_color.set_a(0.8);

            std::string path_name = "agv_phase_path_" + std::to_string(agv.id);
            VisualizationPlot::Plot(path_x, path_y, 0.015, path_color, 100 + agv.id, path_name);

            ROS_INFO("Visualized path for AGV %d (type=%d, phase=%.1f°, lateral_offset=%.3f m)", agv.id, agv.trajectory_type,
                     agv.phase_offset * 180.0 / M_PI, agv.lateral_offset);
        }

        VisualizationPlot::Trigger();
        ROS_INFO("All AGV paths visualized with FIXED lateral offsets and CONSTANT longitudinal spacing");
    }

    // 修改后的位置计算函数，包含横向偏移
    void GetContinuousPoseFromS(double s, double amplitude, double frequency, double phase_offset, double lateral_offset, Pose& pose) {
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

    // 应用蛇形偏移和横向偏移
    void ApplySnakeOffsetWithLateral(double s, double amplitude, double frequency, double phase_offset, double lateral_offset,
                                     double center_x, double center_y, double theta, Pose& pose) {
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

    void UpdateAGVPose(AGVState& agv) {
        GetContinuousPoseFromS(agv.s_position, agv.amplitude, agv.frequency, agv.phase_offset, agv.lateral_offset, agv.current_pose);
    }

    void UpdateSimulation(const ros::TimerEvent&) {
        if (!enabled_) return;

        std::lock_guard<std::mutex> lock(mutex_);
        const double dt = 0.05;

        // 关键修改：所有AGV的s_position都以相同的速度更新
        for (auto& agv : agvs_) {
            if (!agv.is_active) continue;

            // 使用统一的s_velocity更新s_position，确保纵向距离不变
            agv.s_position += agv.s_velocity * dt;
            agv.s_position = fmod(agv.s_position, total_length_);
            if (agv.s_position < 0.0) agv.s_position += total_length_;

            UpdateAGVPose(agv);
        }

        VisualizeCurrentAGVs();
    }

    void VisualizeCurrentAGVs() {
        for (const auto& agv : agvs_) {
            if (!agv.is_active) continue;

            std::vector<Vec2d> agv_circle;
            const int num_points = 16;
            for (int i = 0; i < num_points; ++i) {
                double angle = 2.0 * M_PI * i / num_points;
                double x = agv.current_pose.x() + agv_radius_ * cos(angle);
                double y = agv.current_pose.y() + agv_radius_ * sin(angle);
                agv_circle.emplace_back(x, y);
            }

            Polygon2d agv_poly(agv_circle);
            VisualizationPlot::PlotPolygon(agv_poly, 0.02, agv.color, 200 + agv.id, "agv_current_" + std::to_string(agv.id));

            double arrow_len = agv_radius_ * 1.5;
            double end_x = agv.current_pose.x() + arrow_len * cos(agv.current_pose.theta());
            double end_y = agv.current_pose.y() + arrow_len * sin(agv.current_pose.theta());

            std::vector<double> arrow_x = {agv.current_pose.x(), end_x};
            std::vector<double> arrow_y = {agv.current_pose.y(), end_y};

            Color arrow_color = Color::Orange;
            VisualizationPlot::Plot(arrow_x, arrow_y, 0.012, arrow_color, 210 + agv.id, "agv_heading_" + std::to_string(agv.id));
        }

        VisualizationPlot::Trigger();
    }
};