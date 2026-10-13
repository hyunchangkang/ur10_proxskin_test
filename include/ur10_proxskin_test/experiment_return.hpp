#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include "ur10_proxskin_test/experiment_cancel.hpp"
#include "ur10_proxskin_test/joint_startup.hpp"

// Called only after a user cancellation and after closing the experiment CSV.
// Restore the same validated six-joint reference used before each experiment.
template<class Reader, class Publisher, class Safety>
void returnAfterExperimentCancel(rclcpp::Node &node, std::atomic<bool> &running,
                                 Reader read, Publisher publish, Safety safety)
{
    if (!rclcpp::ok() || !node.get_parameter("experiment_start_enabled").as_bool()) return;
    running = true;
    ExperimentCancelInput keyboard(true, [&running]() { running = false; });
    RCLCPP_INFO(node.get_logger(), "Returning to saved experiment start after cancellation.");
    RCLCPP_INFO(node.get_logger(), "CSV closed. Press q again to stop the return (no Enter needed).");

    // Let the driver's 100 ms target watchdog stop the interrupted trajectory
    // before taking fresh feedback for the return plan.
    const auto stopped = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (rclcpp::ok() && running && std::chrono::steady_clock::now() < stopped)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    double current[DoF];
    const bool reached = rclcpp::ok() && running && runJointStartup(node, current,
        read, publish, []() {}, [&running]() { return running.load(); }, safety);
    keyboard.stop();
    running = false;
    if (keyboard.requested())
        RCLCPP_INFO(node.get_logger(), "Return cancelled by q. Motion commands stopped.");
    else if (reached)
        RCLCPP_INFO(node.get_logger(), "Return complete. Saved experiment start reached.");
    else if (rclcpp::ok())
        RCLCPP_ERROR(node.get_logger(), "Return to saved experiment start failed; motion commands stopped.");
}
