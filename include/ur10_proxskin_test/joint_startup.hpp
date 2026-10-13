#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "ur10_proxskin_test/joint_startup_plan.hpp"

inline void declareJointStartupParameters(rclcpp::Node &node)
{
    node.declare_parameter<std::vector<double>>("initial_arm_joint_deg", {-90.0, -90.0, -90.0, 90.0, -90.0});
    node.declare_parameter<double>("initial_move_velocity_rad_s", 0.1);
    node.declare_parameter<bool>("initial_pose_enabled", true);
    node.declare_parameter<bool>("startup_only", false);
    node.declare_parameter<bool>("base_mode", false);
    node.declare_parameter<bool>("base_hold_j6", true);
    node.declare_parameter<bool>("calibration_mode", false);
    node.declare_parameter<std::string>("calibration_file", "/home/ur/.ros/ur10_proxskin_test/sensor_zero.json");
    node.declare_parameter<bool>("sensor_zero_enabled", false);
    node.declare_parameter<std::string>("sensor_zero_robot_name", "");
    node.declare_parameter<std::vector<double>>("sensor_zero_pose", std::vector<double>{});
    node.declare_parameter<std::vector<double>>("sensor_zero_joint_rad", std::vector<double>{});
    node.declare_parameter<bool>("capture_start_mode", false);
    node.declare_parameter<bool>("start_setup_mode", false);
    node.declare_parameter<double>("start_distance_mm", 300.0);
    node.declare_parameter<double>("endpoint_position_tolerance_mm", 0.1);
    node.declare_parameter<double>("endpoint_orientation_tolerance_deg", 0.1);
    node.declare_parameter<double>("endpoint_settle_seconds", 0.25);
    node.declare_parameter<double>("endpoint_timeout_seconds", 5.0);
    node.declare_parameter<std::string>("start_file", "/home/ur/.ros/ur10_proxskin_test/experiment_start.json");
    node.declare_parameter<bool>("experiment_start_enabled", false);
    node.declare_parameter<std::string>("experiment_start_robot_name", "");
    node.declare_parameter<std::vector<double>>("experiment_start_pose", std::vector<double>{});
    node.declare_parameter<std::vector<double>>("experiment_start_joint_rad", std::vector<double>{});
}

inline VectorXd scanOriginPose(rclcpp::Node &node, const VectorXd &actual)
{
    const bool experiment = node.get_parameter("experiment_start_enabled").as_bool();
    if (!experiment && !node.get_parameter("sensor_zero_enabled").as_bool())
        return actual;
    const auto values = node.get_parameter(experiment ? "experiment_start_pose" : "sensor_zero_pose").as_double_array();
    if (values.size() != 6)
        throw std::invalid_argument("Saved reference pose must contain six values");
    return Eigen::Map<const VectorXd>(values.data(), 6);
}

template<class Reader, class Publisher, class Recorder, class Active, class Safety>
bool runJointStartup(rclcpp::Node &node, double *current,
                     Reader read, Publisher publish, Recorder record,
                     Active active, Safety safety)
{
    if (!read(current))
        return false;
    const bool base_mode = node.get_parameter("base_mode").as_bool();
    const bool experiment = node.get_parameter("experiment_start_enabled").as_bool() && !base_mode &&
                            !node.get_parameter("calibration_mode").as_bool();
    const bool calibrated = (experiment || node.get_parameter("sensor_zero_enabled").as_bool()) && !base_mode &&
                            !node.get_parameter("calibration_mode").as_bool();
    if (!base_mode && !calibrated && !node.get_parameter("initial_pose_enabled").as_bool())
        return true;
    StartupJoints initial{};
    std::copy_n(current, DoF, initial.begin());
    JointStartupPlan plan;
    try
    {
        auto arm_deg = node.get_parameter("initial_arm_joint_deg").as_double_array();
        double base_goal = std::numeric_limits<double>::quiet_NaN();
        if (calibrated)
        {
            const std::string prefix = experiment ? "experiment_start" : "sensor_zero";
            const auto joints = node.get_parameter(prefix + "_joint_rad").as_double_array();
            const auto pose = node.get_parameter(prefix + "_pose").as_double_array();
            if (joints.size() != 6 || pose.size() != 6 ||
                node.get_parameter(prefix + "_robot_name").as_string() != node.get_parameter("robot_name").as_string())
                throw std::invalid_argument("Missing or mismatched saved reference; save the reference again");
            for (int j = 0; j < DoF; ++j)
                if (!std::isfinite(joints[j]) || !std::isfinite(pose[j]) || std::abs(joints[j]) > 2 * PI)
                    throw std::invalid_argument("Invalid saved reference data");
            if (!experiment && std::abs(initial[0] - joints[0]) > 0.05 * DEG2RAD)
                throw std::invalid_argument("J1 differs from the saved calibration; restore the base angle or recalibrate");
            if (experiment) base_goal = joints[0];
            arm_deg.resize(5);
            for (int j = 1; j < DoF; ++j) arm_deg[j - 1] = joints[j] * RAD2DEG;
        }
        plan = makeJointStartupPlan(initial,
            arm_deg,
            node.get_parameter("initial_move_velocity_rad_s").as_double(),
            node.get_parameter("control_rate").as_double(), base_goal,
            base_mode && node.get_parameter("base_hold_j6").as_bool());
        if (calibrated)
        {
            ArmControl arm;
            VectorXd predicted = VectorXd::Zero(6);
            arm.Joint2Cartesian(plan.goal.data(), predicted);
            const VectorXd zero = scanOriginPose(node, predicted);
            Vector3d expected_eul = zero.tail<3>(), predicted_eul = predicted.tail<3>();
            Matrix3d expected_rotation, predicted_rotation;
            arm.Eul2Rot(expected_eul, expected_rotation);
            arm.Eul2Rot(predicted_eul, predicted_rotation);
            if ((predicted.head<3>() - zero.head<3>()).norm() > 0.0005 ||
                Eigen::AngleAxisd(expected_rotation * predicted_rotation.transpose()).angle() > 0.0005)
                throw std::invalid_argument("Saved reference pose does not match the saved joints; save it again");
        }
    }
    catch (const std::invalid_argument &error)
    {
        RCLCPP_ERROR(node.get_logger(), "Initial move rejected: %s", error.what());
        return false;
    }
    RCLCPP_INFO(node.get_logger(), "Initial joint target (deg): %.2f %.2f %.2f %.2f %.2f %.2f; %s",
                plan.goal[0] * RAD2DEG, plan.goal[1] * RAD2DEG, plan.goal[2] * RAD2DEG,
                plan.goal[3] * RAD2DEG, plan.goal[4] * RAD2DEG, plan.goal[5] * RAD2DEG,
                experiment ? "returning to saved experiment start (all six joints)" :
                    (calibrated ? "J1 held, returning to saved sensor zero" :
                     (plan.held[5] ? "J1/J6 held, initializing J2..J5" : "J1 held, initializing J2..J6")));
    auto checked_publish = [&](const StartupJoints &command)
    {
        if (!read(current) || !safety(command.data()))
            return false;
        for (int j = 0; j < DoF; ++j)
            if (plan.held[j] && std::abs(current[j] - initial[j]) > 0.5 * DEG2RAD)
            {
                RCLCPP_ERROR(node.get_logger(), "A held joint moved more than 0.5 degrees; stopping initial move.");
                return false;
            }
        publish(command.data());
        record();
        return true;
    };
    rclcpp::Rate rate(node.get_parameter("control_rate").as_double());
    for (size_t i = 1; i < plan.commands.size(); ++i)
    {
        if (!rclcpp::ok() || !active() || !checked_publish(plan.commands[i]))
            return false;
        rate.sleep();
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (rclcpp::ok() && active() && std::chrono::steady_clock::now() < deadline)
    {
        if (!checked_publish(plan.goal) || !read(current))
            return false;
        double error = 0.0;
        for (int j = 0; j < DoF; ++j)
            error = std::max(error, std::abs(current[j] - plan.goal[j]));
        bool at_zero = true;
        if (calibrated)
        {
            ArmControl arm;
            VectorXd measured = VectorXd::Zero(6);
            arm.Joint2Cartesian(current, measured);
            const VectorXd origin = scanOriginPose(node, measured);
            Vector3d measured_eul = measured.tail<3>(), origin_eul = origin.tail<3>();
            Matrix3d measured_rotation, origin_rotation;
            arm.Eul2Rot(measured_eul, measured_rotation);
            arm.Eul2Rot(origin_eul, origin_rotation);
            at_zero = (measured.head<3>() - origin.head<3>()).norm() <= 0.0005 &&
                Eigen::AngleAxisd(origin_rotation * measured_rotation.transpose()).angle() <= 0.0005;
        }
        if (error <= 0.2 * DEG2RAD && at_zero)
        {
            RCLCPP_INFO(node.get_logger(), "%s", experiment ? "Saved experiment start pose reached." : calibrated ? "Saved sensor zero pose reached." :
                (base_mode ? "Base initial pose reached." : "Common initial pose reached; capturing scan origin."));
            return true;
        }
        rate.sleep();
    }
    RCLCPP_ERROR(node.get_logger(), "Initial move did not settle within 3 seconds.");
    return false;
}
