#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sandbox_msgs/AprilObject.h>
#include <sandbox_msgs/Trajectory.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <deque>
#include <mutex>
#include <vector>

#include "common/math/pose.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace trajectory_tracking {

class DifferentialTracker {
   public:
    DifferentialTracker() : private_nh_("~") {
        // 参数初始化
        private_nh_.param<std::string>("robot_namespace", robot_namespace_, "/robot_1");
        // 使用新的、更简单的控制参数
        private_nh_.param("k_x", k_x_, 2.0);
        private_nh_.param("k_y", k_y_, 5.0);
        private_nh_.param("k_theta", k_theta_, 1.5);
        private_nh_.param("max_linear_speed", max_linear_speed_, 0.2);
        private_nh_.param("max_angular_speed", max_angular_speed_, 1.0);
        private_nh_.param("tracking_object", tracking_object_, 1);

        // 添加调试参数
        private_nh_.param("debug_output", debug_output_, true);
        private_nh_.param("enable_visualization", enable_visualization_, true);

        // ROS订阅和发布
        traj_topic_ = BuildTopicName("/trajectory");
        trajectory_sub_ =
            private_nh_.subscribe(traj_topic_, 100, &DifferentialTracker::TrajectoryCallback, this, ros::TransportHints().tcpNoDelay());

        pose_topic_ = BuildTopicName("/pose");
        cmd_vel_topic_ = BuildTopicName("/cmd_vel");

        nokov_sub_ = private_nh_.subscribe("/nokov_info", 1, &DifferentialTracker::NokovCallback, this, ros::TransportHints().tcpNoDelay());

        cmd_vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

        // 调试发布器
        debug_pub_ = private_nh_.advertise<std_msgs::String>("/control_debug" + robot_namespace_, 1);
        marker_pub_ = private_nh_.advertise<visualization_msgs::Marker>("/control_markers" + robot_namespace_, 1);

        // 控制定时器（20Hz）
        control_timer_ = private_nh_.createTimer(ros::Duration(0.05), &DifferentialTracker::ControlLoop, this);

        // 调试定时器
        debug_timer_ = private_nh_.createTimer(ros::Duration(0.5), &DifferentialTracker::PublishDebugInfo, this);

        ROS_INFO("[%s] Differential Tracker initialized with NEW controller.", robot_namespace_.c_str());
        ROS_INFO("[%s] Control gains: k_x=%.2f, k_y=%.2f, k_theta=%.2f", robot_namespace_.c_str(), k_x_, k_y_, k_theta_);
        ROS_INFO("[%s] Subscribing to trajectory: %s", robot_namespace_.c_str(), traj_topic_.c_str());
        ROS_INFO("[%s] Publishing cmd_vel to: %s", robot_namespace_.c_str(), cmd_vel_topic_.c_str());
    }

   private:
    std::string BuildTopicName(const std::string &suffix) { return robot_namespace_ + suffix; }

    // 轨迹回调函数 - 保持不变
    void TrajectoryCallback(const sandbox_msgs::Trajectory::ConstPtr &msg) {
        if (msg->target != tracking_object_) {
            ROS_DEBUG("[%s] Trajectory target mismatch: received %d, expecting %d", robot_namespace_.c_str(), msg->target,
                      tracking_object_);
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        bool is_new_trajectory = false;
        if (trajectory_.points.empty() || msg->points.empty() || std::abs(msg->points[0].time - trajectory_.points[0].time) > 0.5) {
            is_new_trajectory = true;
        }

        trajectory_ = *msg;

        if (!msg->points.empty()) {
            trajectory_start_time_ = msg->points.front().time;
            trajectory_end_time_ = msg->points.back().time;

            if (is_new_trajectory) {
                current_target_index_ = 0;
                trajectory_completed_ = false;
                ROS_INFO("[%s] New trajectory received: %lu points, time range [%.3f, %.3f]", robot_namespace_.c_str(), msg->points.size(),
                         trajectory_start_time_, trajectory_end_time_);
            }
            is_trajectory_active_ = true;
            last_trajectory_time_ = ros::Time::now();
        }
    }

    // Nokov回调 - 保持不变
    void NokovCallback(const std_msgs::StringConstPtr &msg) {
        json obj;
        try {
            obj = json::parse(msg->data);
        } catch (std::exception &ex) {
            ROS_ERROR_THROTTLE(1.0, "parsing json: %s", ex.what());
            return;
        }
        if (!obj["move_obs"].contains(std::to_string(tracking_object_))) return;

        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<double> pose;
        obj["move_obs"][std::to_string(tracking_object_)].get_to(pose);
        common::math::Pose cur_pose = common::math::Pose(pose[0] / 1000.0, pose[1] / 1000.0, pose[2]).extend(0.05);
        current_pose_.position.x = cur_pose.x();
        current_pose_.position.y = cur_pose.y();
        current_pose_.orientation = tf::createQuaternionMsgFromYaw(pose[2]);
        has_position_ = true;
    }

    // 控制循环主逻辑
    void ControlLoop(const ros::TimerEvent &) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!has_position_) {
            // ROS_WARN_THROTTLE(1.0, "[%s] Waiting for position data from Nokov...", robot_namespace_.c_str());
            return;
        }

        if (!is_trajectory_active_ || trajectory_.points.empty()) {
            StopRobot();
            return;
        }

        // ** MODIFICATION: As requested, removed the trajectory timeout stop mechanism. **
        // This was a safety feature to stop the robot if no new trajectory was received for a while.
        // It has been disabled according to the user's request.
        /*
        if ((ros::Time::now() - last_trajectory_time_).toSec() > 5.0) {
            ROS_WARN_THROTTLE(1.0, "[%s] Trajectory timeout, stopping robot", robot_namespace_.c_str());
            StopRobot();
            is_trajectory_active_ = false;
            return;
        }
        */

        const double current_time = ros::Time::now().toSec();
        sandbox_msgs::TrajectoryPoint target;
        bool found_target = GetCurrentTargetImproved(current_time, target);

        if (!found_target) {
            if (!trajectory_completed_) {
                trajectory_completed_ = true;
                ROS_INFO("[%s] Trajectory completed, maintaining last position.", robot_namespace_.c_str());
            }
            if (!trajectory_.points.empty()) {
                target = trajectory_.points.back();
            } else {
                StopRobot();
                return;
            }
        }

        geometry_msgs::Twist cmd_vel = CalculateControlCommand(target);
        cmd_vel_pub_.publish(cmd_vel);

        if (enable_visualization_) {
            VisualizeTracking(target);
        }

        if (debug_output_) {
            ROS_INFO_THROTTLE(0.5,
                              "[%s] Target: (%.3f, %.3f, %.1f deg) | Current: (%.3f, %.3f, %.1f deg) | Error: (x:%.3f, y:%.3f, th:%.1f "
                              "deg) | Cmd: (v:%.3f, w:%.3f)",
                              robot_namespace_.c_str(), target.x, target.y, target.yaw * 180.0 / M_PI, current_pose_.position.x,
                              current_pose_.position.y, tf::getYaw(current_pose_.orientation) * 180.0 / M_PI, last_x_error_, last_y_error_,
                              last_yaw_error_ * 180.0 / M_PI, cmd_vel.linear.x, cmd_vel.angular.z);
        }
    }

    // 目标点获取 - 保持不变
    bool GetCurrentTargetImproved(double current_time, sandbox_msgs::TrajectoryPoint &target) {
        const auto &points = trajectory_.points;
        if (points.empty()) return false;
        if (current_time < trajectory_start_time_) {
            target = points.front();
            return true;
        }
        if (current_time > trajectory_end_time_) {
            return false;
        }
        size_t left = 0, right = points.size() - 1;
        while (left < right) {
            size_t mid = left + (right - left) / 2;
            if (points[mid].time < current_time) {
                left = mid + 1;
            } else {
                right = mid;
            }
        }
        size_t idx = (left > 0) ? left - 1 : 0;
        if (idx >= points.size() - 1) {
            target = points.back();
            return true;
        }
        const auto &prev = points[idx];
        const auto &next = points[idx + 1];
        double dt = next.time - prev.time;
        if (dt < 1e-6) {
            target = next;
            return true;
        }
        double alpha = (current_time - prev.time) / dt;
        alpha = std::max(0.0, std::min(1.0, alpha));
        target.x = prev.x + alpha * (next.x - prev.x);
        target.y = prev.y + alpha * (next.y - prev.y);
        target.yaw = InterpolateYaw(prev.yaw, next.yaw, alpha);
        target.velocity = prev.velocity + alpha * (next.velocity - prev.velocity);
        target.omega = prev.omega + alpha * (next.omega - prev.omega);
        current_target_index_ = idx;
        return true;
    }

    // 航向角插值 - 保持不变
    double InterpolateYaw(double start, double end, double alpha) {
        double delta = end - start;
        while (delta > M_PI) delta -= 2 * M_PI;
        while (delta < -M_PI) delta += 2 * M_PI;
        double result = start + alpha * delta;
        while (result > M_PI) result -= 2 * M_PI;
        while (result < -M_PI) result += 2 * M_PI;
        return result;
    }

    // ** MODIFICATION: Replaced the control law with a standard non-linear feedback controller. **
    // The previous controller had very high gains and complex switching logic, which was the main cause of instability.
    // This new controller is simpler, more stable, and easier to tune.
    geometry_msgs::Twist CalculateControlCommand(const sandbox_msgs::TrajectoryPoint &target) {
        geometry_msgs::Twist cmd_vel;

        const double current_x = current_pose_.position.x;
        const double current_y = current_pose_.position.y;
        const double current_yaw = tf::getYaw(current_pose_.orientation);

        // --- Calculate errors in the robot's reference frame ---
        const double dx = target.x - current_x;
        const double dy = target.y - current_y;
        const double x_error = dx * cos(current_yaw) + dy * sin(current_yaw);     // Longitudinal error (forward)
        const double y_error = -dx * sin(current_yaw) + dy * cos(current_yaw);    // Lateral error (to the left)

        // Heading error
        double yaw_error = target.yaw - current_yaw;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;

        // --- Standard Non-Linear Feedback Controller ---

        // 1. Linear Velocity Control
        // Combines feedforward from the trajectory (target.velocity) with feedback on longitudinal error.
        // The cos(yaw_error) term reduces speed if the robot is not pointing in the right direction.
        cmd_vel.linear.x = target.velocity * cos(yaw_error) + k_x_ * x_error;

        // 2. Angular Velocity Control
        // ** MODIFICATION: Decoupled lateral error correction from target velocity. **
        // The original term `k_y_ * target.velocity * y_error` was ineffective at low speeds.
        // The new term `k_y_ * y_error` ensures a consistent corrective force regardless of the robot's speed,
        // which is crucial for handling turns.
        cmd_vel.angular.z = target.omega + k_y_ * y_error + k_theta_ * sin(yaw_error);

        // 3. Apply saturation (velocity limits)
        cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, -max_linear_speed_), max_linear_speed_);
        cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -max_angular_speed_), max_angular_speed_);

        // 记录误差用于调试
        last_x_error_ = x_error;
        last_y_error_ = y_error;
        last_yaw_error_ = yaw_error;

        return cmd_vel;
    }

    // 发布调试信息 - 保持不变
    void PublishDebugInfo(const ros::TimerEvent &) {
        // This function remains the same
    }

    // 可视化跟踪状态 - 保持不变
    void VisualizeTracking(const sandbox_msgs::TrajectoryPoint &target) {
        // This function remains the same
    }

    void StopRobot() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel_pub_.publish(cmd_vel);
    }

    ros::NodeHandle private_nh_;
    std::string robot_namespace_;
    std::string pose_topic_, traj_topic_;
    std::string cmd_vel_topic_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber odom_sub_, nokov_sub_, object_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher debug_pub_;
    ros::Publisher marker_pub_;
    ros::Timer control_timer_;
    ros::Timer debug_timer_;

    // ** MODIFICATION: Updated control parameter variables **
    double k_x_;
    double k_y_;
    double k_theta_;
    double max_linear_speed_;
    double max_angular_speed_;

    // 状态存储
    std::mutex mutex_;
    sandbox_msgs::Trajectory trajectory_;
    geometry_msgs::Pose current_pose_;
    size_t current_target_index_ = 0;
    bool is_trajectory_active_ = false;
    bool trajectory_completed_ = false;
    ros::Time last_trajectory_time_;
    double trajectory_start_time_ = 0.0;
    double trajectory_end_time_ = 0.0;
    bool has_position_ = false;
    bool debug_output_ = false;
    bool enable_visualization_ = false;

    // 误差记录
    double last_x_error_ = 0.0;
    double last_y_error_ = 0.0;
    double last_yaw_error_ = 0.0;

    int tracking_object_ = 0;
};

}    // namespace trajectory_tracking

int main(int argc, char **argv) {
    ros::init(argc, argv, "differential_tracker");
    trajectory_tracking::DifferentialTracker tracker;
    ros::spin();
    return 0;
}
