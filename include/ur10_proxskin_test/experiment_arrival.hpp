#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "ur10_proxskin_test/arm_function.h"

// Wait at bottom/top turns and where X travel joins a vertical column.
inline bool experimentBoundary(const Vector3d &from, const Vector3d &at, const Vector3d &next)
{
    const double incoming = at.z() - from.z(), outgoing = next.z() - at.z();
    return incoming * outgoing < -1e-18 ||
           ((std::abs(incoming) > 1e-9) != (std::abs(outgoing) > 1e-9));
}

template<class Reader, class Publisher, class Recorder, class Active, class Safety>
bool waitForExperimentArrival(rclcpp::Node &node, const double *command, const VectorXd &goal,
                              Reader read, Publisher publish, Recorder record, Active active, Safety safety)
{
    const double position_limit = node.get_parameter("endpoint_position_tolerance_mm").as_double() / 1000.0;
    const double angle_limit = node.get_parameter("endpoint_orientation_tolerance_deg").as_double() * DEG2RAD;
    const double settle = node.get_parameter("endpoint_settle_seconds").as_double();
    const double timeout = node.get_parameter("endpoint_timeout_seconds").as_double();
    if (!std::isfinite(position_limit) || position_limit <= 0 || !std::isfinite(angle_limit) || angle_limit <= 0 ||
        !std::isfinite(settle) || settle <= 0 || !std::isfinite(timeout) || timeout <= settle)
    {
        RCLCPP_ERROR(node.get_logger(), "Invalid endpoint tolerances or timing.");
        return false;
    }
    ArmControl arm;
    Vector3d euler = goal.tail<3>();
    Matrix3d goal_rotation;
    arm.Eul2Rot(euler, goal_rotation);
    const auto started = std::chrono::steady_clock::now();
    auto settled_since = started;
    bool settling = false;
    rclcpp::Rate rate(node.get_parameter("control_rate").as_double());
    double position_error = 0.0, angle_error = 0.0;
    while (rclcpp::ok() && active())
    {
        double measured_joints[DoF];
        if (!read(measured_joints) || !safety(command)) return false;
        publish(command); // Continue servo commands until actual arrival is verified.
        record();
        VectorXd actual = VectorXd::Zero(6);
        arm.Joint2Cartesian(measured_joints, actual);
        Vector3d actual_euler = actual.tail<3>();
        Matrix3d actual_rotation;
        arm.Eul2Rot(actual_euler, actual_rotation);
        position_error = (actual.head<3>() - goal.head<3>()).norm();
        angle_error = Eigen::AngleAxisd(goal_rotation * actual_rotation.transpose()).angle();
        const auto now = std::chrono::steady_clock::now();
        if (position_error <= position_limit && angle_error <= angle_limit)
        {
            if (!settling) { settled_since = now; settling = true; }
            if (std::chrono::duration<double>(now - settled_since).count() >= settle)
            {
                RCLCPP_INFO(node.get_logger(), "Endpoint reached: position error %.3f mm, orientation error %.3f deg.",
                            position_error * 1000.0, angle_error * RAD2DEG);
                return true;
            }
        }
        else settling = false;
        if (std::chrono::duration<double>(now - started).count() >= timeout)
        {
            RCLCPP_ERROR(node.get_logger(), "Endpoint not reached within %.1f s: position error %.3f mm, orientation error %.3f deg. Experiment stopped.",
                         timeout, position_error * 1000.0, angle_error * RAD2DEG);
            return false;
        }
        rate.sleep();
    }
    return false;
}
