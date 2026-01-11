#pragma once
#include <cmath>

struct DpDeciderConfig {
    // 基础参数
    double tf = 10.0;
    double s_coefficient = 1.0;
    double resample_resolution = 0.001;

    // 权重参数（两版本）
    double w_lateral = 1.0;
    double w_lateral_change = 0.2;
    double w_obstacle_near = 1e15;    // 近距离障碍物权重（硬约束）
    double w_obstacle_far = 100.0;    // 远距离障碍物权重（软约束）
    double w_target = 0.0;

    // 一刀切参数
    double cutoff_ratio = 0.25;      // 一刀切比例（前25%为近距离）
    double resource_ratio = 0.4;     // 近距离资源占比（40%的采样点）
    double node_multiplier = 1.5;    // 近距离节点倍数

    // 采样参数
    int total_layers = 25;               // 总层数
    int nominal_nodes_per_layer = 20;    // 名义上每层节点数

    // 道路参数
    double l_deviation = 0.200823;
    double road_half_width = 0.35;
    double center_line_obs_range_front = 0.30;
    double center_line_obs_range_rear = 0.20;

    // 其他参数
    double valid_angular = 0.25 * M_PI;
    double min_s_distance = 0.5;
    double initial_next_length = 0.2;
    double collision_check_resolution = 0.03;

    // 计算得出的参数（将在初始化时计算）
    int near_layers = 0;             // 近距离层数
    int far_layers = 0;              // 远距离层数
    int near_nodes_per_layer = 0;    // 近距离每层节点数
    int far_nodes_per_layer = 0;     // 远距离每层节点数
    double cutoff_distance = 0.0;    // 一刀切绝对距离

    // 初始化函数，计算派生参数
    void Initialize(double horizon) {
        // 计算一刀切绝对距离
        cutoff_distance = horizon * cutoff_ratio;

        // 计算近距离层数（占resource_ratio的层数资源）
        near_layers = static_cast<int>(total_layers * resource_ratio);
        far_layers = total_layers - near_layers;

        // 计算近距离每层节点数
        near_nodes_per_layer = static_cast<int>(nominal_nodes_per_layer * node_multiplier);

        // 计算总节点数和远距离每层节点数
        int total_nodes = total_layers * nominal_nodes_per_layer;
        int near_total_nodes = near_layers * near_nodes_per_layer;
        int remaining_nodes = total_nodes - near_total_nodes;

        // 远距离每层节点数（向下取整）
        far_nodes_per_layer = remaining_nodes / far_layers;
        if (far_nodes_per_layer < 1) {
            far_nodes_per_layer = 1;
        }
    }
};