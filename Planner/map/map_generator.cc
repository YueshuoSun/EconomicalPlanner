#include "map_generator.h"

using namespace common::math;
using namespace common::util;

namespace map_generator {
MapGenerator::MapGenerator(const MapParam &config) : map_param_(config) {
    road_width_ = 0.0;
    min_turn_radiu_ = map_param_.map_radius;
    x_first_line_ = map_param_.x_first_straight;
    x_straight_ = map_param_.x_second_straight;
    y_straight_ = map_param_.y_straight;
    curve_length_ = map_param_.curve_length;
    resolution_ = map_param_.resolution;
}

Vec2d RotatePt(const common::math::Vec2d &tp, double angle) {
    double x = tp.x() * cos(angle) - tp.y() * sin(angle);
    double y = tp.x() * sin(angle) + tp.y() * cos(angle);
    return {x, y};
}

void MapGenerator::GenerateRoadNodes() {
    common::data::TrajectoryPoint node_1;
    node_1.x = road_width_ * 0.5 + x_first_line_;
    node_1.y = -road_width_;
    node_1.theta = 0;

    common::data::TrajectoryPoint node_2;
    node_2.x = node_1.x + x_straight_;
    node_2.y = node_1.y;
    node_2.theta = 0;

    common::data::TrajectoryPoint node_3;
    node_3.x = node_2.x + min_turn_radiu_ + road_width_;
    node_3.y = node_2.y + min_turn_radiu_ + road_width_;
    node_3.theta = M_PI_2;

    common::data::TrajectoryPoint node_4;
    node_4.x = node_3.x;
    node_4.y = node_3.y + y_straight_;
    node_4.theta = M_PI_2;

    common::data::TrajectoryPoint node_5;
    node_5.x = node_2.x;
    node_5.y = node_4.y + min_turn_radiu_ + road_width_;
    node_5.theta = M_PI;

    common::data::TrajectoryPoint node_6;
    node_6.x = -node_5.x;
    node_6.y = node_5.y;
    node_6.theta = M_PI;

    common::data::TrajectoryPoint node_7;
    node_7.x = -node_4.x;
    node_7.y = node_4.y;
    node_7.theta = M_PI * 1.5;

    common::data::TrajectoryPoint node_8;
    node_8.x = -node_3.x;
    node_8.y = node_3.y;
    node_8.theta = M_PI * 1.5;

    common::data::TrajectoryPoint node_9;
    node_9.x = -node_2.x;
    node_9.y = node_2.y;
    node_9.theta = M_PI * 2.0;

    common::data::TrajectoryPoint node_10;
    node_10.x = -node_1.x;
    node_10.y = node_1.y;
    node_10.theta = M_PI * 2.0;

    road_node_.emplace_back(node_1);
    road_node_.emplace_back(node_2);
    road_node_.emplace_back(node_3);
    road_node_.emplace_back(node_4);
    road_node_.emplace_back(node_5);
    road_node_.emplace_back(node_6);
    road_node_.emplace_back(node_7);
    road_node_.emplace_back(node_8);
    road_node_.emplace_back(node_9);
    road_node_.emplace_back(node_10);
}

void MapGenerator::GenerateCurves() {
    common::math::QuinticSpiralPath spi_path2(0, 0, 0, M_PI_2, 0, 0, curve_length_);
    auto ss = common::util::LinSpaced(0, curve_length_, 60);
    std::vector<double> xs5(ss.size()), ys5(ss.size()), theta5(ss.size());
    for (int i = 0; i < ss.size(); i++) {
        xs5[i] = spi_path2.ComputeCartesianDeviationX<5>(ss[i]);
        ys5[i] = spi_path2.ComputeCartesianDeviationY<5>(ss[i]);
        theta5[i] = spi_path2.Evaluate(0, ss[i]);
    }

    common::data::Trajectory curve_5;
    for (int i = 0; i < xs5.size(); i++) {
        common::data::TrajectoryPoint tp;
        tp.x = xs5[i];
        tp.y = ys5[i];
        tp.theta = theta5[i];
        tp.s = ss[i];
        curve_5.emplace_back(tp);
    }
    spirit_curve_.emplace_back(curve_5);

    std::vector<double> xs6(xs5.size()), ys6(xs5.size()), theta6(xs5.size());
    for (int i = 0; i < xs5.size(); i++) {
        auto tp = RotatePt(Vec2d(xs5[i], ys5[i]), M_PI_2);
        xs6[i] = tp.x();
        ys6[i] = tp.y();
        theta6[i] = theta5[i] + M_PI_2;
    }

    common::data::Trajectory curve_6;
    for (int i = 0; i < xs6.size(); i++) {
        common::data::TrajectoryPoint tp;
        tp.x = xs6[i];
        tp.y = ys6[i];
        tp.theta = theta6[i];
        tp.s = ss[i];
        curve_6.emplace_back(tp);
    }
    spirit_curve_.emplace_back(curve_6);

    std::vector<double> xs7(xs5.size()), ys7(xs5.size()), theta7(xs5.size());
    for (int i = 0; i < xs5.size(); i++) {
        auto tp = RotatePt(Vec2d(xs5[i], ys5[i]), M_PI);
        xs7[i] = tp.x();
        ys7[i] = tp.y();
        theta7[i] = theta5[i] + M_PI;
    }

    common::data::Trajectory curve_7;
    for (int i = 0; i < xs7.size(); i++) {
        common::data::TrajectoryPoint tp;
        tp.x = xs7[i];
        tp.y = ys7[i];
        tp.theta = theta7[i];
        tp.s = ss[i];
        curve_7.emplace_back(tp);
    }
    spirit_curve_.emplace_back(curve_7);

    std::vector<double> xs8(xs5.size()), ys8(xs5.size()), theta8(xs5.size());
    for (int i = 0; i < xs8.size(); i++) {
        auto tp = RotatePt(Vec2d(xs5[i], ys5[i]), M_PI * 1.5);
        xs8[i] = tp.x();
        ys8[i] = tp.y();
        theta8[i] = theta5[i] + M_PI * 1.5;
    }

    common::data::Trajectory curve_8;
    for (int i = 0; i < xs8.size(); i++) {
        common::data::TrajectoryPoint tp;
        tp.x = xs8[i];
        tp.y = ys8[i];
        tp.theta = theta8[i];
        tp.s = ss[i];
        curve_8.emplace_back(tp);
    }
    spirit_curve_.emplace_back(curve_8);
}

void MapGenerator::GenerateRoads() {
    double resolution = resolution_;
    std::vector<common::data::TrajectoryPoint> road_seg_1;
    std::vector<double> xs1, ys1, thetas1, ss1;
    double len = std::sqrt(std::pow(road_node_[0].x - road_node_[1].x, 2) + std::pow(road_node_[0].y - road_node_[1].y, 2));
    int n = len / resolution;

    xs1 = LinSpaced(road_node_[0].x, road_node_[1].x, n);
    ys1 = LinSpaced(road_node_[0].y, road_node_[1].y, n);
    thetas1 = std::vector<double>(n, road_node_[0].theta);
    ss1 = LinSpaced(0, len, n);

    road_seg_1.resize(xs1.size());
    for (int i = 0; i < road_seg_1.size(); i++) {
        road_seg_1[i].x = xs1[i];
        road_seg_1[i].y = ys1[i];
        road_seg_1[i].theta = thetas1[i];
        road_seg_1[i].s = ss1[i];
    }

    std::vector<common::data::TrajectoryPoint> road_seg_2 = spirit_curve_[0];
    std::vector<double> xs2(road_seg_2.size()), ys2(road_seg_2.size()), thetas2(road_seg_2.size());
    for (int i = 0; i < road_seg_2.size(); i++) {
        road_seg_2[i].x += road_width_ * 0.5 + x_first_line_ + x_straight_;
        road_seg_2[i].y += -road_width_;
        road_seg_2[i].theta += 0;
        road_seg_2[i].s += road_seg_1.back().s;

        xs2[i] = road_seg_2[i].x;
        ys2[i] = road_seg_2[i].y;
        thetas2[i] = road_seg_2[i].theta;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_3;
    std::vector<double> xs3, ys3, thetas3, ss3;
    len = std::sqrt(std::pow(road_node_[2].x - road_node_[3].x, 2) + std::pow(road_node_[2].y - road_node_[3].y, 2));
    n = len / resolution;

    xs3 = LinSpaced(road_node_[2].x, road_node_[3].x, n);
    ys3 = LinSpaced(road_node_[2].y, road_node_[3].y, n);
    thetas3 = std::vector<double>(n, road_node_[2].theta);
    ss3 = LinSpaced(0, len, n);

    road_seg_3.resize(xs3.size());
    for (int i = 0; i < road_seg_3.size(); i++) {
        road_seg_3[i].x = xs3[i];
        road_seg_3[i].y = ys3[i];
        road_seg_3[i].theta = thetas3[i];
        road_seg_3[i].s = ss3[i] + road_seg_2.back().s;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_4 = spirit_curve_[1];
    std::vector<double> xs4(road_seg_4.size()), ys4(road_seg_4.size()), thetas4(road_seg_4.size());
    for (int i = 0; i < road_seg_4.size(); i++) {
        road_seg_4[i].x += road_width_ * 0.5 + min_turn_radiu_ + x_first_line_ + x_straight_ + road_width_;
        road_seg_4[i].y += min_turn_radiu_ + y_straight_;
        road_seg_4[i].theta += 0;
        road_seg_4[i].s += road_seg_3.back().s;

        xs4[i] = road_seg_4[i].x;
        ys4[i] = road_seg_4[i].y;
        thetas4[i] = road_seg_4[i].theta;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_5;
    std::vector<double> xs5, ys5, thetas5, ss5;
    len = std::sqrt(std::pow(road_node_[4].x - road_node_[5].x, 2) + std::pow(road_node_[4].y - road_node_[5].y, 2));
    n = len / resolution;

    xs5 = LinSpaced(road_node_[4].x, road_node_[5].x, n);
    ys5 = LinSpaced(road_node_[4].y, road_node_[5].y, n);
    thetas5 = std::vector<double>(n, road_node_[5].theta);
    ss5 = LinSpaced(0, len, n);

    road_seg_5.resize(xs5.size());
    for (int i = 0; i < road_seg_5.size(); i++) {
        road_seg_5[i].x = xs5[i];
        road_seg_5[i].y = ys5[i];
        road_seg_5[i].theta = thetas5[i];
        road_seg_5[i].s = ss5[i] + road_seg_4.back().s;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_6 = spirit_curve_[2];
    std::vector<double> xs6(road_seg_6.size()), ys6(road_seg_6.size()), thetas6(road_seg_6.size());
    for (int i = 0; i < road_seg_6.size(); i++) {
        road_seg_6[i].x += -road_width_ * 0.5 - x_first_line_ - x_straight_;
        road_seg_6[i].y += 2.0 * min_turn_radiu_ + y_straight_ + road_width_;
        road_seg_6[i].theta += 0;
        road_seg_6[i].s += road_seg_5.back().s;

        xs6[i] = road_seg_6[i].x;
        ys6[i] = road_seg_6[i].y;
        thetas6[i] = road_seg_6[i].theta;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_7;
    std::vector<double> xs7, ys7, thetas7, ss7;
    len = std::sqrt(std::pow(road_node_[6].x - road_node_[7].x, 2) + std::pow(road_node_[6].y - road_node_[7].y, 2));
    n = len / resolution;

    xs7 = LinSpaced(road_node_[6].x, road_node_[7].x, n);
    ys7 = LinSpaced(road_node_[6].y, road_node_[7].y, n);
    thetas7 = std::vector<double>(n, road_node_[6].theta);
    ss7 = LinSpaced(0, len, n);

    road_seg_7.resize(xs7.size());
    for (int i = 0; i < road_seg_7.size(); i++) {
        road_seg_7[i].x = xs7[i];
        road_seg_7[i].y = ys7[i];
        road_seg_7[i].theta = thetas7[i];
        road_seg_7[i].s = ss7[i] + road_seg_6.back().s;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_8 = spirit_curve_[3];
    std::vector<double> xs8(road_seg_8.size()), ys8(road_seg_8.size()), thetas8(road_seg_8.size());
    for (int i = 0; i < road_seg_8.size(); i++) {
        road_seg_8[i].x += -road_width_ * 1.5 - min_turn_radiu_ - x_first_line_ - x_straight_;
        road_seg_8[i].y += min_turn_radiu_;
        road_seg_8[i].theta += 0;
        road_seg_8[i].s += road_seg_7.back().s;

        xs8[i] = road_seg_8[i].x;
        ys8[i] = road_seg_8[i].y;
        thetas8[i] = road_seg_8[i].theta;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_9;
    std::vector<double> xs9, ys9, thetas9, ss9;
    len = std::sqrt(std::pow(road_node_[8].x - road_node_[9].x, 2) + std::pow(road_node_[8].y - road_node_[9].y, 2));
    n = len / resolution;

    xs9 = LinSpaced(road_node_[8].x, road_node_[9].x, n);
    ys9 = LinSpaced(road_node_[8].y, road_node_[9].y, n);
    thetas9 = std::vector<double>(n, road_node_[9].theta);
    ss9 = LinSpaced(0, len, n);

    road_seg_9.resize(xs9.size());
    for (int i = 0; i < road_seg_9.size(); i++) {
        road_seg_9[i].x = xs9[i];
        road_seg_9[i].y = ys9[i];
        road_seg_9[i].theta = thetas9[i];
        road_seg_9[i].s = ss9[i] + road_seg_8.back().s;
    }

    std::vector<common::data::TrajectoryPoint> road_seg_10;
    std::vector<double> xs10, ys10, thetas10, ss10;
    // CRITICAL BUG FIX: The original code used road_node_[25] which is an out-of-bounds access.
    // Corrected to use road_node_[9] for the start point of this segment.
    len = std::sqrt(std::pow(road_node_[9].x - road_node_[0].x, 2) + std::pow(road_node_[9].y - road_node_[0].y, 2));
    n = len / resolution;

    xs10 = LinSpaced(road_node_[9].x, road_node_[0].x, n);
    ys10 = LinSpaced(road_node_[9].y, road_node_[0].y, n);
    thetas10 = std::vector<double>(n, road_node_[9].theta);
    ss10 = LinSpaced(0, len, n);

    road_seg_10.resize(xs10.size());
    for (int i = 0; i < road_seg_10.size(); i++) {
        road_seg_10[i].x = xs10[i];
        road_seg_10[i].y = ys10[i];
        road_seg_10[i].theta = thetas10[i];
        road_seg_10[i].s = ss10[i];
    }

    std::vector<common::data::TrajectoryPoint> points_1;
    points_1.insert(points_1.end(), road_seg_1.begin(), road_seg_1.end());
    points_1.insert(points_1.end(), road_seg_2.begin(), road_seg_2.end());
    points_1.insert(points_1.end(), road_seg_3.begin(), road_seg_3.end());
    points_1.insert(points_1.end(), road_seg_4.begin(), road_seg_4.end());
    points_1.insert(points_1.end(), road_seg_5.begin(), road_seg_5.end());
    points_1.insert(points_1.end(), road_seg_6.begin(), road_seg_6.end());
    points_1.insert(points_1.end(), road_seg_7.begin(), road_seg_7.end());
    points_1.insert(points_1.end(), road_seg_8.begin(), road_seg_8.end());
    points_1.insert(points_1.end(), road_seg_9.begin(), road_seg_9.end());

    Road road_1;
    road_1.points = points_1;
    for (auto &tp : road_1.points) {
        tp.road_id = 2;
    }
    road_1.next_seg = {3, 4};
    roads_.emplace_back(road_1);

    std::vector<double> road_1_xs(points_1.size()), road_1_ys(points_1.size()), road_1_thetas(points_1.size()), road_1_ss(points_1.size());
    for (int i = 0; i < points_1.size(); i++) {
        road_1_xs[i] = points_1[i].x;
        road_1_ys[i] = points_1[i].y;
        road_1_thetas[i] = points_1[i].theta;
        road_1_ss[i] = points_1[i].s;
    }

    Road road_2;
    road_2.points = road_seg_10;
    for (auto &tp : road_2.points) {
        tp.road_id = 3;
    }
    road_2.next_seg = {2};
    roads_.emplace_back(road_2);
    std::vector<double> road_2_xs(road_seg_10.size()), road_2_ys(road_seg_10.size()), road_2_thetas(road_seg_10.size()),
        road_2_ss(road_seg_10.size());
    for (int i = 0; i < road_seg_10.size(); i++) {
        road_2_xs[i] = road_seg_10[i].x;
        road_2_ys[i] = road_seg_10[i].y;
        road_2_thetas[i] = road_seg_10[i].theta;
        road_2_ss[i] = road_seg_10[i].s;
    }

    for (int i = 0; i < roads_.size(); i++) {
        std::vector<int> repeat_record;
        for (int j = 0; j < roads_[i].points.size() - 1; j++) {
            if (hypot(roads_[i].points[j + 1].x - roads_[i].points[j].x, roads_[i].points[j + 1].y - roads_[i].points[j].y) < 0.001) {
                repeat_record.emplace_back(j + 1);
            }
        }

        std::vector<common::data::TrajectoryPoint> points;
        for (int j = 0; j < roads_[i].points.size(); j++) {
            bool flag = false;
            for (auto id : repeat_record) {
                if (j == id) {
                    flag = true;
                    break;
                }
            }
            if (!flag) points.emplace_back(roads_[i].points[j]);
        }
        roads_[i].points = points;
    }

    road_segment_.emplace_back(road_seg_1);
    road_segment_.emplace_back(road_seg_2);
    road_segment_.emplace_back(road_seg_3);
    road_segment_.emplace_back(road_seg_4);
    road_segment_.emplace_back(road_seg_5);
    road_segment_.emplace_back(road_seg_6);
    road_segment_.emplace_back(road_seg_7);
    road_segment_.emplace_back(road_seg_8);
    road_segment_.emplace_back(road_seg_9);
    road_segment_.emplace_back(road_seg_10);
}

void MapGenerator::GenerateMap() {
    // CRITICAL FIX: Clear all member vectors before generating the map.
    // This prevents data from accumulating across multiple calls, which was the
    // source of the probabilistic crash. Each call to GenerateMap() will now
    // start from a clean state, making the function idempotent and safe.
    road_node_.clear();
    spirit_curve_.clear();
    road_segment_.clear();
    roads_.clear();

    GenerateRoadNodes();
    GenerateCurves();
    GenerateRoads();
}

}    // namespace map_generator
