#ifndef MULTI_AGV_TRAJECTORY_GENERATOR_H
#define MULTI_AGV_TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <sandbox_msgs/Trajectory.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <mutex>
#include <unordered_map>

#include "common/data/discretized_trajectory.h"
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "map/map_generator.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using common::math::AABox2d;
using common::math::Box2d;
using common::math::Polygon2d;
using common::math::Pose;
using common::math::Vec2d;

class MultiAGVTrajectoryGenerator {
   public:
    MultiAGVTrajectoryGenerator(ros::NodeHandle& nh, const std::shared_ptr<map_generator::MapGenerator>& map_gen);

    // 获取当前AGV障碍物快照
    std::vector<Polygon2d> GetAGVObstaclesSnapshot();

    // 检查是否有AGV靠近中心线
    bool HasAGVNearCenterline(const std::vector<common::data::TrajectoryPoint>& ref_line, double tolerance = 0.1);

   private:
    // AGV状态结构体
    struct AGVState {
        int id;
        double s_position;       // 在参考线上的弧长位置
        double base_velocity;    // 基础速度
        double s_velocity;       // 沿参考线的实际速度
        Pose current_pose;       // 当前位姿
        bool is_active;          // 是否激活

        // 轨迹参数
        int trajectory_type;      // 轨迹类型
        double amplitude;         // 蛇形振幅
        double frequency;         // 蛇形频率
        double phase_offset;      // 相位偏移
        double lateral_offset;    // 固定横向偏移

        // 长轨迹管理
        double last_trajectory_end_s;                   // 上次发送轨迹的终点s位置
        ros::Time last_trajectory_sent_time;            // 上次发送轨迹的时间
        common::data::Trajectory current_trajectory;    // 当前轨迹
    };

    // 初始化函数
    bool InitializeReferenceLine();
    void InitializeAGVs();

    // 回调函数
    void nokovCallback(const std_msgs::StringConstPtr& msg);
    void checkAndPublishTrajectories(const ros::TimerEvent&);
    void publishDebugInfo(const ros::TimerEvent&);

    // 轨迹生成
    bool needsNewTrajectory(int agv_index);
    void sendLongTrajectory(int agv_index);
    common::data::Trajectory generateLongTrajectory(int agv_index);
    common::data::Trajectory generateShortTrajectory(int agv_index);    // 保留兼容

    // 辅助函数
    double ProjectToReferenceLine(double x, double y);
    void GetContinuousPoseFromS(double s, double amplitude, double frequency, double phase_offset, double lateral_offset, Pose& pose);
    void ApplySnakeOffsetWithLateral(double s, double amplitude, double frequency, double phase_offset, double lateral_offset,
                                     double center_x, double center_y, double theta, Pose& pose);
    double CalculatePathLengthRatio(double amplitude, double frequency);
    void UpdateAGVPose(AGVState& agv);

    // 可视化
    void visualizeAllTrajectories();
    void visualizeTrajectory(int agv_id, const common::data::Trajectory& trajectory);

    // 兼容接口（可选实现）
    void UpdateSimulation(const ros::TimerEvent&);
    void GenerateAndVisualizeFullPaths();
    void VisualizeAllPaths();

    // ROS节点句柄
    ros::NodeHandle nh_;

    // 发布器和订阅器
    std::unordered_map<int, ros::Publisher> trajectory_pubs_;
    ros::Publisher marker_pub_;
    ros::Publisher debug_pub_;
    ros::Subscriber nokov_sub_;
    ros::Timer publish_timer_;
    ros::Timer debug_timer_;

    // 地图生成器
    std::shared_ptr<map_generator::MapGenerator> map_generator_;

    // 参数
    bool enabled_;
    int num_agvs_;
    double base_agv_velocity_;
    double agv_radius_;
    std::vector<double> agv_lateral_offsets_;
    std::vector<int> agv_ids_;

    // 轨迹参数
    double trajectory_coverage_;     // 轨迹覆盖比例（如0.75表示3/4圈）
    double trajectory_overlap_;      // 重叠比例（如0.25表示1/4圈）
    double trajectory_lookahead_;    // 前视距离（短轨迹模式）
    double trajectory_dt_;           // 时间步长
    double publish_frequency_;       // 发布频率

    // 参考线数据
    std::vector<common::data::TrajectoryPoint> reference_points_;
    double total_length_;

    // AGV状态
    std::vector<AGVState> agvs_;
    std::unordered_map<int, common::math::Pose> current_poses_;
    std::unordered_map<int, bool> poses_initialized_;

    // 同步互斥锁
    std::mutex mutex_;

    double max_v_;
    double max_a_;
    double max_omega_;
};

#endif    // MULTI_AGV_TRAJECTORY_GENERATOR_H