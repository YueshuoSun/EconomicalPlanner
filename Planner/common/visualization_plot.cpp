#include "visualization_plot.h"

#include <common/math/math_utils.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

namespace VisualizationPlot {
namespace {
std::string frame_ = "map";

ros::Publisher publisher_;
visualization_msgs::MarkerArray arr_;
}    // namespace
}    // namespace VisualizationPlot

void VisualizationPlot::Init(ros::NodeHandle &node, const std::string &frame,
                             const std::string &topic) {
    frame_ = frame;
    publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 1000, true);
    sleep(1);
}

void VisualizationPlot::Plot(const Vector &xs, const Vector &ys, double width, Color color, int id,
                             const std::string &ns) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = width;
    msg.color = color.toColorRGBA();

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        pt.z = 0.1 * id;
        msg.points.push_back(pt);
    }

    arr_.markers.push_back(msg);
}

void VisualizationPlot::Plot(const VisualizationPlot::Vector &xs,
                             const VisualizationPlot::Vector &ys, double width,
                             const std::vector<Color> &color, int id, const std::string &ns) {
    assert(xs.size() == color.size());

    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = width;

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        msg.points.push_back(pt);
        msg.colors.push_back(color[i].toColorRGBA());
    }

    arr_.markers.push_back(msg);
}

void VisualizationPlot::PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color,
                                    int id, const std::string &ns) {
    auto xxs = xs;
    auto yys = ys;
    xxs.push_back(xxs[0]);
    yys.push_back(yys[0]);
    Plot(xxs, yys, width, color, id, ns);
}

void VisualizationPlot::PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                                    const std::string &ns) {
    std::vector<double> xs, ys;
    for (auto &pt : polygon.points()) {
        xs.push_back(pt.x());
        ys.push_back(pt.y());
    }
    PlotPolygon(xs, ys, width, color, id, ns);
}

void VisualizationPlot::PlotPath(const Vector &xs, const Vector &ys, const Vector &thetas,
                                 double bias, double width, int id, const std::string &ns) {
    std::vector<Color> colors(xs.size());
    auto gears = common::math::GetPathGears(xs, ys, thetas);

    for (size_t i = 0; i < xs.size() - 1; i++) {
        colors[i] = Color::fromHSV(2 * bias - (gears[i] ? 1.0 : -1.0) * bias, 1.0, 1.0);
        colors[i].set_a(std::min(1.0, width / 0.02));
    }
    colors[colors.size() - 1] = colors.back();

    Plot(xs, ys, width, colors, id, ns);
}

void VisualizationPlot::PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs,
                                       double max_velocity, double width, int id,
                                       const std::string &ns) {
    std::vector<Color> colors(xs.size());

    for (size_t i = 0; i < xs.size(); i++) {
        double percent = (vs[i] / max_velocity);
        colors[i] = Color::fromHSV(120.0f - percent * 120, 1.0, 1.0);
    }

    Plot(xs, ys, width, colors, id, ns);
}

void VisualizationPlot::PlotPoints(const Vector &xs, const Vector &ys, double width,
                                   const Color &color, int id, const std::string &ns) {
    assert(xs.size() == ys.size());

    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns.empty() ? "Points" : ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = !xs.empty() ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = msg.scale.y = width;
    msg.color = color.toColorRGBA();

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        msg.points.push_back(pt);
    }

    arr_.markers.push_back(msg);
}

void VisualizationPlot::PlotCross(const double xs, const double ys, double width,
                                  const Color &color, int id, const std::string &ns) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns.empty() ? "Cross" : ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = msg.scale.y = width;
    msg.color = color.toColorRGBA();

    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = xs - width;
    p1.y = ys - width;
    p1.z = 0;
    p2.x = xs + width;
    p2.y = ys + width;
    p2.z = 0;
    p3.x = xs - width;
    p3.y = ys + width;
    p3.z = 0;
    p4.x = xs + width;
    p4.y = ys - width;
    p4.z = 0;

    msg.points.push_back(p1);
    msg.points.push_back(p2);
    msg.points.push_back(p3);
    msg.points.push_back(p4);

    arr_.markers.push_back(msg);
}

void VisualizationPlot::PlotPose(const Pose &pose, double width, const Color &color, int id,
                                 const std::string &ns) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns.empty() ? "Pose" : ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::ARROW;
    msg.pose.position.x = pose.x();
    msg.pose.position.y = pose.y();
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());
    msg.scale.x = width * 2;
    msg.scale.y = width * 0.2;
    msg.scale.z = width;
    msg.color = color.toColorRGBA();

    arr_.markers.push_back(msg);
}

void VisualizationPlot::CalculateStarPoints(Vec2d center_point, double side_length,
                                            std::vector<double> &points_x,
                                            std::vector<double> &points_y) {
    const double angle = 72.0 * M_PI / 180.0;
    const double radius = side_length / (2 * sin(M_PI / 5));

    for (int i = 0; i < 5; ++i) {
        points_x.push_back(center_point.x() + radius * cos(i * angle));
        points_y.push_back(center_point.y() + radius * sin(i * angle));

        points_x.push_back(center_point.x() + radius * cos((i + 2) * angle));
        points_y.push_back(center_point.y() + radius * sin((i + 2) * angle));
    }
}

void VisualizationPlot::PlotFilledStar(Vec2d center_point, double side_length, Color color, int id,
                                       const std::string &ns) {
    std::vector<double> points_x, points_y;
    CalculateStarPoints(center_point, side_length, points_x, points_y);

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.color = color.toColorRGBA();

    for (size_t i = 0; i < points_x.size(); i += 2) {
        geometry_msgs::Point p1, p2, p3;

        p1.x = center_point.x();
        p1.y = center_point.y();
        p1.z = 0.0;

        p2.x = points_x[i];
        p2.y = points_y[i];
        p2.z = 0.0;

        p3.x = points_x[i + 1];
        p3.y = points_y[i + 1];
        p3.z = 0.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
    }

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    arr_.markers.push_back(marker);
}

void VisualizationPlot::PlotBox(const common::math::Box2d &box, Color color, int id,
                                const std::string &ns) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, box.heading());
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    marker.pose.position.x = box.center().x();
    marker.pose.position.y = box.center().y();
    marker.pose.position.z = 0.0;

    marker.scale.x = box.length();
    marker.scale.y = box.width();
    marker.scale.z = 0.01;

    marker.color = color.toColorRGBA();

    arr_.markers.push_back(marker);
}

void VisualizationPlot::PlotFilledPolygon(const Polygon2d &polygon, Color color, int id,
                                          const std::string &ns) {
    if (polygon.points().size() < 3) {
        // A polygon must have at least 3 vertices to be filled.
        return;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color = color.toColorRGBA();

    // Triangulate the polygon to fill it.
    // This simple triangulation works for convex polygons.
    // It creates a fan of triangles from the first vertex.
    const auto &points = polygon.points();
    const geometry_msgs::Point p0 = toPoint(points[0]);

    for (size_t i = 1; i < points.size() - 1; ++i) {
        marker.points.push_back(p0);
        marker.points.push_back(toPoint(points[i]));
        marker.points.push_back(toPoint(points[i + 1]));
    }

    arr_.markers.push_back(marker);
}

void VisualizationPlot::PlotVehicleHeading(const common::math::Box2d &box, const Color &color,
                                           double width, int id, const std::string &ns) {
    visualization_msgs::Marker heading_marker;
    heading_marker.header.frame_id = frame_;
    heading_marker.header.stamp = ros::Time::now();
    heading_marker.ns = ns;
    heading_marker.id = id;
    heading_marker.action = visualization_msgs::Marker::ADD;
    heading_marker.type = visualization_msgs::Marker::LINE_LIST;    // 使用LINE_LIST绘制两条独立的线
    heading_marker.pose.orientation.w = 1.0;

    heading_marker.scale.x = width;    // 设置线宽
    heading_marker.color = color.toColorRGBA();

    // 手动计算车辆的关键点
    const double heading = box.heading();
    const double cos_h = cos(heading);
    const double sin_h = sin(heading);
    const double half_len = box.length() / 2.0;
    const double half_wid = box.width() / 2.0;

    // 前边中点
    geometry_msgs::Point front_mid;
    front_mid.x = box.center().x() + half_len * cos_h;
    front_mid.y = box.center().y() + half_len * sin_h;
    front_mid.z = 0.01;    // 略微抬高以避免与车身重叠

    // 左后顶点
    geometry_msgs::Point rear_left;
    rear_left.x = box.center().x() - half_len * cos_h + half_wid * -sin_h;
    rear_left.y = box.center().y() - half_len * sin_h + half_wid * cos_h;
    rear_left.z = 0.01;

    // 右后顶点
    geometry_msgs::Point rear_right;
    rear_right.x = box.center().x() - half_len * cos_h - half_wid * -sin_h;
    rear_right.y = box.center().y() - half_len * sin_h - half_wid * cos_h;
    rear_right.z = 0.01;

    // 添加第一条线: 左后 -> 前中
    heading_marker.points.push_back(rear_left);
    heading_marker.points.push_back(front_mid);

    // 添加第二条线: 右后 -> 前中
    heading_marker.points.push_back(rear_right);
    heading_marker.points.push_back(front_mid);

    arr_.markers.push_back(heading_marker);
}

void VisualizationPlot::PlotGradientLine(const Vector &xs, const Vector &ys, double width,
                                         const Color &highlight_color, const Color &base_color,
                                         int id, const std::string &ns) {
    if (xs.size() != ys.size() || xs.size() < 2) {
        return;    // Need at least two points to draw a line
    }

    // --- 1. 绘制阴影/外边框 (最底层, 略宽, 半透明黑色) ---
    // 这一层模拟了物体的边缘阴影，增加了立体感。
    visualization_msgs::Marker shadow_msg;
    shadow_msg.header.frame_id = frame_;
    shadow_msg.header.stamp = ros::Time();
    shadow_msg.ns = ns;
    shadow_msg.id = id + 20000;    // 使用一个独立的ID
    shadow_msg.action = visualization_msgs::Marker::ADD;
    shadow_msg.type = visualization_msgs::Marker::LINE_STRIP;
    shadow_msg.pose.orientation.w = 1.0;
    shadow_msg.scale.x = width * 1.2;    // 比基线略宽

    Color shadow_color = Color::Black;
    shadow_color.set_a(0.4);    // 40% 透明度的阴影
    shadow_msg.color = shadow_color.toColorRGBA();

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        pt.z = 0.0;    // 在最底层
        shadow_msg.points.push_back(pt);
    }
    arr_.markers.push_back(shadow_msg);

    // --- 2. 绘制基线 (中间层, 正常宽度, 不透明的主体颜色) ---
    visualization_msgs::Marker base_msg;
    base_msg.header.frame_id = frame_;
    base_msg.header.stamp = ros::Time();
    base_msg.ns = ns;
    base_msg.id = id;    // 使用传入的ID
    base_msg.action = visualization_msgs::Marker::ADD;
    base_msg.type = visualization_msgs::Marker::LINE_STRIP;
    base_msg.pose.orientation.w = 1.0;
    base_msg.scale.x = width;                     // 正常线宽
    base_msg.color = base_color.toColorRGBA();    // 基色完全不透明

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        pt.z = 0.001;    // 在阴影之上
        base_msg.points.push_back(pt);
    }
    arr_.markers.push_back(base_msg);

    // --- 3. 绘制高光线 (最顶层, 极细, 明亮的颜色) ---
    visualization_msgs::Marker highlight_msg;
    highlight_msg.header.frame_id = frame_;
    highlight_msg.header.stamp = ros::Time();
    highlight_msg.ns = ns;
    highlight_msg.id = id + 10000;    // 使用另一个独立的ID
    highlight_msg.action = visualization_msgs::Marker::ADD;
    highlight_msg.type = visualization_msgs::Marker::LINE_STRIP;
    highlight_msg.pose.orientation.w = 1.0;
    highlight_msg.scale.x = width * 0.4;                    // 高光线宽度为基线的40%
    highlight_msg.color = highlight_color.toColorRGBA();    // 高光线完全不透明

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        pt.z = 0.002;    // 在基线之上，防止Z-fighting闪烁
        highlight_msg.points.push_back(pt);
    }
    arr_.markers.push_back(highlight_msg);
}

void VisualizationPlot::Trigger() {
    publisher_.publish(arr_);
    arr_.markers.clear();
}

void VisualizationPlot::Clear() {
    arr_.markers.clear();

    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.ns = "Markers";

    msg.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(msg);
    publisher_.publish(arr);
}

geometry_msgs::Point VisualizationPlot::toPoint(const Vec2d &vec) {
    geometry_msgs::Point p;
    p.x = vec.x();
    p.y = vec.y();
    p.z = 0.0;
    return p;
}

// namespace {    // Anonymous namespace for local helpers

// }    // namespace