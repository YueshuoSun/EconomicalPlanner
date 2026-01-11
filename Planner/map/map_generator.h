#pragma once

#include <vector>

#include "common/data/discretized_trajectory.h"
#include "common/math/polygon2d.h"
#include "common/math/quintic_spiral_path.h"
#include "common/math/vec2d.h"
#include "common/obstacle.h"
#include "common/util/color.h"
#include "common/util/vector.h"
#include "common/visualization_plot.h"
#include "map_param.h"

namespace map_generator {

struct Road {
    common::data::Trajectory points;
    std::vector<int> next_seg;
};

class MapGenerator {
   public:
    MapGenerator();
    MapGenerator(const MapParam &config);
    void GenerateMap();
    void GenerateRoadNodes();
    void GenerateRoads();
    void GenerateCurves();
    void Visualize();

    void road(int ind, Road &result) const { result = roads_[ind]; }

    const std::vector<Road> roads() const { return roads_; }

    std::vector<common::data::TrajectoryPoint> road_node() const { return road_node_; }
    std::vector<common::data::Trajectory> road_segment() const { return road_segment_; }

   private:
    double road_width_;
    double min_turn_radiu_;
    double x_straight_;
    double y_straight_;
    double x_first_line_;
    double curve_length_;
    double resolution_;

    MapParam map_param_;

    std::vector<common::data::TrajectoryPoint> road_node_;
    std::vector<std::vector<common::data::TrajectoryPoint>> spirit_curve_;
    std::vector<common::data::Trajectory> road_segment_;

    std::vector<Road> roads_;
};

}    // namespace map_generator