#ifndef DP_SV_GRAPH_CONFIG_H
#define DP_SV_GRAPH_CONFIG_H

struct DpSVGraphConfig {
    // 基础参数
    double tf = 10.0;
    int nfe = 101;

    // 一刀切参数
    double cutoff_ratio = 0.25;      // 一刀切比例（前25%为近距离）
    double resource_ratio = 0.4;     // 近距离资源占比（40%的采样点）
    double node_multiplier = 1.5;    // 近距离节点倍数

    // 采样参数
    int total_s_layers = 100;    // 总S层数
    int nominal_v_nodes = 50;    // 名义上每层V节点数

    // 计算得出的参数（将在运行时计算）
    int near_s_layers = 0;    // 近距离S层数
    int far_s_layers = 0;     // 远距离S层数
    int near_v_nodes = 0;     // 近距离V节点数
    int far_v_nodes = 0;      // 远距离V节点数
    double cutoff_s = 0.0;    // 一刀切S距离

    // 权重参数（两版本）
    double w_velocity_near = 10.0;       // 近距离速度偏差权重
    double w_velocity_far = 2.0;         // 远距离速度偏差权重
    double w_acceleration_near = 1.0;    // 近距离加速度权重
    double w_acceleration_far = 0.1;     // 远距离加速度权重

    // 速度和加速度限制
    double v_min = 0.05;
    double v_ratio = 1.0;
    double a_ratio = 1.0;
    double v_reduction_rate = 0.8;

    // 代价函数参数
    double dp_cost_max = 1e8;
    double dp_cost_acc_max = 1e4;

    // 其他参数
    double dt_resolution = 0.001;
    double min_lateral_acceleration = 0.5;

    // 初始化函数
    void Initialize(double s_horizon) {
        // 计算一刀切S距离
        cutoff_s = s_horizon * cutoff_ratio;

        // 计算近距离S层数（占resource_ratio的层数资源）
        near_s_layers = static_cast<int>(total_s_layers * resource_ratio);
        far_s_layers = total_s_layers - near_s_layers;

        // 计算近距离V节点数
        near_v_nodes = static_cast<int>(nominal_v_nodes * node_multiplier);

        // 计算远距离V节点数（向下取整）
        int total_nodes = total_s_layers * nominal_v_nodes;
        int near_total_nodes = near_s_layers * near_v_nodes;
        int remaining_nodes = total_nodes - near_total_nodes;

        far_v_nodes = remaining_nodes / far_s_layers;
        if (far_v_nodes < 3) {
            far_v_nodes = 3;    // 保证最少3个速度节点
        }
    }
};

#endif    // DP_SV_GRAPH_CONFIG_H