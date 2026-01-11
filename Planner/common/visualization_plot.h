#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <mutex>

#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/util/color.h"
#include "common/vehicle_param.h"

using common::math::Polygon2d;
using common::math::Pose;
using common::math::Vec2d;
using common::util::Color;

namespace VisualizationPlot {
using Vector = std::vector<double>;

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1),
          int id = -1, const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1,
          const std::vector<Color> &color = {}, int id = -1, const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPath(const Vector &xs, const Vector &ys, const Vector &thetas, double bias = 120.0,
              double width = 0.1, int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs,
                    double max_velocity = 10.0, double width = 0.1, int id = -1,
                    const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, double width = 0.1,
                const Color &color = Color::White, int id = -1, const std::string &ns = "");

void PlotPose(const Pose &pose, double width = 0.1, const Color &color = Color::White, int id = -1,
              const std::string &ns = "");

void CalculateStarPoints(Vec2d center_point, double side_length, std::vector<double> &points_x,
                         std::vector<double> &points_y);

void PlotFilledStar(Vec2d center_point, double side_length, Color color, int id,
                    const std::string &ns);

void PlotBox(const common::math::Box2d &box, Color color, int id, const std::string &ns);

void PlotFilledPolygon(const Polygon2d &polygon, Color color, int id, const std::string &ns);

void PlotGradientLine(const Vector &xs, const Vector &ys, double width, const Color &start_color,
                      const Color &end_color, int id, const std::string &ns);

void PlotVehicleHeading(const common::math::Box2d &box, const Color &color, double width, int id,
                        const std::string &ns);

void Trigger();

void Clear();

void PlotCross(const double xs, const double ys, double width = 0.1,
               const Color &color = Color::White, int id = -1, const std::string &ns = "");

geometry_msgs::Point toPoint(const Vec2d &vec);
}    // namespace VisualizationPlot
