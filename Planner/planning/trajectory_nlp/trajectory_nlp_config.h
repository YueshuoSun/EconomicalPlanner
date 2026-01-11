#pragma once
#include <cmath>
#include <vector>

struct TrajectoryNLPConfig {
    // 基础参数
    int nfe = 101;
    double tf = 10.0;

    // 一刀切参数
    double cutoff_ratio = 0.25;    // 一刀切比例（前25%为近距离）

    // 走廊生成参数（根据距离调整）
    double corridor_step_near = 0.01;    // 近距离走廊步长
    double corridor_step_far = 0.025;    // 远距离走廊步长
    int corridor_max_iter = 100;
    double corridor_incremental_limit = 0.4;

    // 权重参数（两版本）
    // 近距离权重
    double w_opti_omega_near = 0.5;          // 近距离角速度权重
    double w_opti_a_near = 0.1;              // 近距离加速度权重
    double w_opti_track_near = 2.0;          // 近距离轨迹跟踪权重
    double w_opti_track_theta_near = 0.2;    // 近距离航向跟踪权重
    double w_opti_track_v_near = 10.0;       // 近距离速度跟踪权重

    // 远距离权重
    double w_opti_omega_far = 0.1;           // 远距离角速度权重
    double w_opti_a_far = 0.02;              // 远距离加速度权重
    double w_opti_track_far = 0.5;           // 远距离轨迹跟踪权重
    double w_opti_track_theta_far = 0.05;    // 远距离航向跟踪权重
    double w_opti_track_v_far = 2.5;         // 远距离速度跟踪权重

    // 末端权重倍数
    double end_ratio = 10.0;

    // 速度限制区域参数
    double regions_v_max_ratio = 0.9;
    double N = 1000.0;
    double w_f_regions = 1e7;
    double w_g_regions = 235.0;
    int limit_speed_range = 5;

    // 目标参数
    double range_lb = 0.5;
    double range_ub = 0.9;
    std::vector<double> target_range = {0.5, 0.9};
    double w_f_obj = 3000.0;
    double w_g_obj = 1.0 / 60.0;
    double ang_threshold = 0.25 * M_PI;
    int limit_target_range = 3;

    // 松弛参数
    double epsilon = 0.1;
    double epsilon_sq = 0.005;
    double lb_f_param = 200.0;

    // 框架参数
    int max_num_of_continous_failure = 4;

    // LIOM相关参数
    int var_num = 7;
    double infeasibility_weight_near = 1e6;    // 近距离不可行性权重
    double infeasibility_weight_far = 1e4;     // 远距离不可行性权重
    double infeasibility_weight_ratio = 10.0;
    double infeasibility_threshold = 0.00001;
    int infeasibility_failed_num = 1000;
    int iter_num = 5;

    // 计算得出的参数
    int cutoff_index = 0;    // 一刀切索引

    // 初始化函数
    void Initialize() {
        // 计算一刀切索引
        cutoff_index = static_cast<int>(nfe * cutoff_ratio);
        if (cutoff_index < 1) cutoff_index = 1;
        if (cutoff_index >= nfe - 1) cutoff_index = nfe - 2;
    }
};