#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <random>

#include "common/data/discretized_trajectory.h"
#include "common/math/pose.h"
#include "common/my_env.h"
#include "map/map_generator.h"
#include "planning/decider/dp_path_decider.h"
#include "planning/decider/dp_sv_graph.h"
#include "planning/trajectory_nlp/trajectory_optimizer.h"

using namespace common::data;

struct CoarseDecision {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
};

class Planner {
   public:
    explicit Planner();

    void ReadConfig();

    void set_current_state(const common::data::TrajectoryPoint &current_state) { current_state_ = current_state; }

    void set_current_state(double x, double y, double theta, double v, double phi, double a, double omega) {
        current_state_.x = x;
        current_state_.y = y;
        current_state_.theta = theta;
        current_state_.v = v;
        current_state_.phi = phi;
        current_state_.a = a;
        current_state_.omega = omega;
    }

    void set_current_state(double x, double y, double theta, double v) {
        current_state_.x = x;
        current_state_.y = y;
        current_state_.theta = theta;
        current_state_.v = v;
    }

    void set_start_pose(double x, double y, double theta) { start_pose_ = Pose(x, y, theta); }

    void set_start_pose(Pose start_pose) { start_pose_ = start_pose; }

    const Pose &start_pose() const { return start_pose_; }

    const TrajectoryPoint &current_state() const { return current_state_; }

    const Trajectory &traj_ref() const { return traj_ref_; }

    const double &start_time() const { return start_time_; }

    void set_start_time(double start_time) { start_time_ = start_time; }

    void set_traj_ref(Trajectory traj_ref) { traj_ref_ = traj_ref; }

    void Clear() { env_->points.Clear(); }

    bool RollingTimePlan(int driving_type, const Trajectory &last_trajectory, Trajectory &new_trajectory);

    bool FindStitchIndex(common::data::Trajectory dp_ref, int &index);

    void GetRefLine(const std::vector<map_generator::Road> &roads);

    bool GenerateRefLine(const common::data::DiscretizedTrajectory &road, const Pose current_pose, common::data::Trajectory &ref_line);

    void GenerateObstacles(const common::data::DiscretizedTrajectory &road, bool is_first, int index);

    std::vector<common::math::Vec2d> PushPoints(const common::data::Trajectory &points, double distance);
    void PushPoints(const common::data::Trajectory &points, double distance, common::data::Trajectory &result);

    Env env() { return env_; }

    std::vector<int> obs_index() { return obs_index_; }

    void CheckTrajFeasible(const Trajectory &result);

    void LogCounter(const int &counter);

    void GenerateStopTraj(const TrajectoryPoint &plan_start, const common::data::DiscretizedTrajectory &ref_line, Trajectory &traj);

    void TruncateTrajectoryToSafePoint(common::data::Trajectory &traj);

    std::vector<double> CalculateInfeasibilityVector(const common::data::Trajectory &traj);

    int FindSafeTrajectoryEndIndex(const std::vector<double> &infeasibility_vector);

   private:
    Env env_;
    DpDeciderConfig dp_path_config_;
    DpSVGraphConfig dp_speed_config_;
    TrajectoryNLPConfig traj_nlp_config_;

    std::shared_ptr<dp_path_decider::DpPathDecider> dp_path_decider_;
    std::shared_ptr<dp_velocity_decider::DpSVGraph> dp_sv_graph_;
    std::shared_ptr<trajectory_nlp::TrajectoryOptimizer> traj_opti_;

    std::shared_ptr<common::util::Color> color_;

    ros::NodeHandle nh_;
    Pose start_pose_, goal_pose_;
    TrajectoryPoint current_state_;
    TrajectoryPoint planned_start_state_;
    std::string env_file_;
    common::data::Trajectory traj_ref_;
    double nominal_v_ = 0.1;
    double start_time_ = 0.0;
    double stitch_time_ = 1.0;
    double road_ratio_ = 0.75;
    int initial_obs_index_ = 50;
    int initial_vel_index_ = 10;
    double min_s_distance_ = 0.5;
    double obs_min_l_ = 0.08;
    double obs_max_l_ = 0.22;
    double obs_length_ = 0.02;
    int obs_num_ = 4;
    double traj_v_max_ratio_ = -999.0;
    int initial_target_index_ = -999;
    double is_not_generate_target_ = -999.0;

    std::vector<int> virtual_obs_index_ = {-999, -999, -999, -999};
    std::vector<double> virtual_obs_l_ = {-999.0, -999.0, -999.0, -999.0};
    double virtual_obs_length_ = -999.0;

    std::vector<int> obs_index_;

    std::random_device rd_;
    std::mt19937 gen_;

    double traj_delta_time_ = 0.1;

    std::string log_filename_, log_time_other_filename_, log_cost_filename_, log_timecost_filename_;
    double min_obs_length_, max_obs_length_;
    double min_obs_width_, max_obs_width_;
    unsigned int random_seed_;    

    std::mt19937 random_generator_;                        
    std::uniform_real_distribution<double> length_dist_;  
    std::uniform_real_distribution<double> width_dist_;    
    std::uniform_real_distribution<double> heading_deviation_dist_;
    std::uniform_real_distribution<double> lateral_dist_;
    std::uniform_int_distribution<int> side_chooser_dist_;         
    std::uniform_real_distribution<double> left_lateral_dist_;    
    std::uniform_real_distribution<double> right_lateral_dist_;    

    double my_dp_time_;
    double my_nlp_time_;
    double my_liom_time_;
    double other_dp_time_;
    double other_nlp_time_;
    void LogTime(double dp_solve_time, double nlp_solve_time, const std::string &log_filename);
    void LogCost(double liom_cost, double nlp_cost, const std::string &log_filename);
    void LogTimeCost(double dp_solve_time, double nlp_solve_time, double liom_solve_time, double liom_cost, double nlp_cost,
                     const std::string &log_filename);

    double intercept_max_length_ = -999.0;
    double stop_extend_s_ = -999.0;
    double stop_acc_ratio_ = -999.0;
    int fault_add_stop_point_num_ = -999;
    double vel_move_dist_ = -999.0;

    bool PlanTrajectory(const MyEnvironment &my_env, int driving_type, TrajectoryPoint start, Trajectory &result);

    int FindClosestIndex(const std::vector<double> &vec, double value);

    void LogTime(const double time);

    common::math::Polygon2d GenerateRandomConvexPolygon(const common::math::Vec2d &center, double avg_radius, int min_vertices,
                                                        int max_vertices, double irregularity, double spikeyness);
};
