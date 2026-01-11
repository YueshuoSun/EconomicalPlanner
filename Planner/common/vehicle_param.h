#pragma once
#include <tuple>

#include "common/math/box2d.h"
#include "common/math/circle_2d.h"
#include "common/math/pose.h"

class VehicleParam {
   public:
    double rear_hang = 0.032;
    double front_hang = 0.036;
    double wheel_base = 0.143;
    double length = 0.211;
    double width = 0.211;
    double vehicle_buffer = 0.05;
    double app_vehicle_buffer = 0.03;
    double max_acceleration = 0.1;
    double max_deceleration = -0.1;
    double max_jerk = 1.0;

    double max_velocity = 0.25;
    double max_reverse_velocity = 0.0;

    double max_phi = 0.6;
    double min_phi = -0.6;
    double max_omega = 0.6;

    double radius;
    double c2x;

    double app_width;

    void GenerateDiscs() {
        radius = hypot(length * 0.5, width * 0.5);
        c2x = 0.5 * length - rear_hang;
    }

    template <class T>
    std::tuple<T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
        auto xc = x + c2x * cos(theta);
        auto yc = y + c2x * sin(theta);
        return std::make_tuple(xc, yc);
    }

    common::math::Circle2d GetDiscs(double x, double y, double theta) const {
        double xc, yc;
        std::tie(xc, yc) = GetDiscPositions(x, y, theta);
        return common::math::Circle2d(xc, yc, radius);
    }

    common::math::Circle2d GetDiscsBuffer(double x, double y, double theta,
                                          double buffer = 0.02) const {
        double xc, yc;
        std::tie(xc, yc) = GetDiscPositions(x, y, theta);
        return common::math::Circle2d(xc, yc, radius + buffer);
    }

    common::math::Box2d GenerateBox(const common::math::Pose &pose) const {
        double distance = length / 2 - rear_hang;
        return {pose.extend(distance), pose.theta(), length, width};
    }

    common::math::Box2d GenerateBoxBuffer(const common::math::Pose &pose) const {
        double distance = length / 2 - rear_hang;
        return {pose.extend(distance), pose.theta(), length + 2 * vehicle_buffer,
                width + 2 * vehicle_buffer};
    }

    common::math::Box2d GenerateBoxBuffer(const common::math::Pose &pose,
                                          const double buffer) const {
        double distance = length / 2 - rear_hang;
        return {pose.extend(distance), pose.theta(), length * buffer, width * buffer};
    }

    common::math::Box2d GenerateAppBox(const common::math::Pose &pose) const {
        double distance = length / 2 - rear_hang;
        return {pose.extend(distance), pose.theta(), length, app_width};
    }

    common::math::Box2d GenerateAppHalfBox(const common::math::Pose &pose) const {
        double distance = length / 2 - rear_hang;
        return {pose.extend(distance), pose.theta(), length, app_width / 2};
    }

    common::math::Box2d GenerateHalfBox(const common::math::Pose &pose) const {
        double distance = length / 2 - rear_hang;
        return {pose.extend(distance), pose.theta(), length, width / 2};
    }

    common::math::Vec2d GenerateTopLeftCorner(const common::math::Pose &pose) const {
        common::math::Vec2d top_left_corner;
        top_left_corner.set_x(pose.x() + (front_hang + wheel_base) * cos(pose.theta()) -
                              width / 2 * sin(pose.theta()));
        top_left_corner.set_y(pose.y() + (front_hang + wheel_base) * sin(pose.theta()) +
                              width / 2 * cos(pose.theta()));
        return top_left_corner;
    }

    common::math::Vec2d GenerateFrontCenter(const common::math::Pose &pose) const {
        common::math::Vec2d front_center;
        front_center.set_x(pose.x() + (front_hang + wheel_base) * cos(pose.theta()));
        front_center.set_y(pose.y() + (front_hang + wheel_base) * sin(pose.theta()));
        return front_center;
    }

    common::math::Vec2d GenerateRearCenter(const common::math::Pose &pose) const {
        common::math::Vec2d rear_center;
        rear_center.set_x(pose.x() - rear_hang * cos(pose.theta()));
        rear_center.set_y(pose.y() - rear_hang * sin(pose.theta()));
        return rear_center;
    }
};