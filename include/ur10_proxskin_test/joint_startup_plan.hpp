#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "ur10_proxskin_test/arm_function.h"

using StartupJoints = std::array<double, DoF>;

struct JointStartupPlan
{
    StartupJoints goal{};
    std::array<bool, DoF> held{};
    std::vector<StartupJoints> commands;
};

inline JointStartupPlan makeJointStartupPlan(const StartupJoints &initial,
                                             const std::vector<double> &arm_deg,
                                             double velocity, double rate_hz,
                                             double base_goal = std::numeric_limits<double>::quiet_NaN(),
                                             bool hold_j6 = false)
{
    if (!std::isfinite(velocity) || velocity <= 0.0 || !std::isfinite(rate_hz) || rate_hz <= 0.0)
        throw std::invalid_argument("Initial move speed and control rate must be positive");
    for (double angle : initial)
        if (!std::isfinite(angle) || std::abs(angle) > 2.0 * PI + 1e-10)
            throw std::invalid_argument("Current joint angle is invalid or outside -360..360 degrees");
    JointStartupPlan plan;
    plan.goal = initial;
    if (arm_deg.size() != 5)
        throw std::invalid_argument("initial_arm_joint_deg must contain J2..J6 (five angles)");
    plan.held[0] = true;
    if (!std::isnan(base_goal))
    {
        if (!std::isfinite(base_goal) || std::abs(base_goal) > 2.0 * PI)
            throw std::invalid_argument("Saved base joint is outside -360..360 degrees");
        plan.goal[0] = base_goal;
        plan.held[0] = false;
    }
    for (int j = 1; j < DoF; ++j)
    {
        if (!std::isfinite(arm_deg[j - 1]) || std::abs(arm_deg[j - 1]) > 360.0)
            throw std::invalid_argument("Initial arm angle must be within -360..360 degrees");
        plan.goal[j] = arm_deg[j - 1] * DEG2RAD;
    }
    if (hold_j6)
    {
        plan.held[5] = true;
        plan.goal[5] = initial[5];
    }
    double largest = 0.0;
    for (int j = 0; j < DoF; ++j)
        largest = std::max(largest, std::abs(plan.goal[j] - initial[j]));
    // Smooth start/end; peak speed of this cubic is 1.5 * delta / duration.
    const double duration = std::max(0.25, 1.5 * largest / velocity);
    const int count = std::max(1, static_cast<int>(std::ceil(duration * rate_hz)));
    plan.commands.push_back(initial);
    for (int i = 1; i <= count; ++i)
    {
        const double t = static_cast<double>(i) / count;
        const double fraction = t * t * (3.0 - 2.0 * t);
        StartupJoints command{};
        for (int j = 0; j < DoF; ++j)
            command[j] = plan.held[j] ? initial[j] : initial[j] + fraction * (plan.goal[j] - initial[j]);
        plan.commands.push_back(command);
    }
    return plan;
}
