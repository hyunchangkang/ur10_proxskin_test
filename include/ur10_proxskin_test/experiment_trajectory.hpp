#pragma once

#include <string>
#include "ur10_proxskin_test/vertical_scan_trajectory.hpp"

// Base-frame Z: start -> start - travel -> start, with no initial Z offset.
inline std::vector<VerticalScanPoint> makeDownwardScanTrajectory(
    const Eigen::Vector3d &origin, double travel, double step, int repeats, double dwell)
{
    if (!origin.allFinite() || !std::isfinite(travel) || travel <= 0.0 ||
        !std::isfinite(step) || step <= 0.0 || repeats < 1 ||
        !std::isfinite(dwell) || dwell < 0.0)
        throw std::invalid_argument("Invalid downward scan settings");
    std::vector<VerticalScanPoint> path{{origin, 0.0}};
    auto append_z = [&](double z, double pause) {
        const double from = path.back().position.z();
        const int count = std::max(1, static_cast<int>(std::ceil(std::abs(z - from) / step)));
        for (int i = 1; i <= count; ++i)
            path.push_back({Eigen::Vector3d(origin.x(), origin.y(),
                from + (z - from) * static_cast<double>(i) / count), 0.0});
        path.back().dwell_seconds = pause;
    };
    for (int cycle = 0; cycle < repeats; ++cycle)
    {
        if (dwell == 0.0)
        {
            append_z(origin.z() - travel, 0.0);
            append_z(origin.z(), 0.0);
        }
        else
        {
            const int stops = static_cast<int>(std::ceil(travel * 100.0));
            path.back().dwell_seconds = dwell;
            for (int cm = 1; cm <= stops; ++cm)
                append_z(origin.z() - std::min(cm / 100.0, travel), dwell);
            for (int cm = stops - 1; cm >= 0; --cm)
                append_z(origin.z() - cm / 100.0, dwell);
        }
    }
    return path;
}

// Return to the starting Z before moving to the next horizontal column.
inline std::vector<Eigen::Vector3d> makeDownwardColumnTrajectory(
    const Eigen::Vector3d &origin, double travel, double span, double spacing,
    int repeats, const std::string &direction, const std::string &axis = "x")
{
    if (!std::isfinite(span) || span < 0.0 || (axis != "x" && axis != "y") ||
        (direction != "centered" && direction != "positive" && direction != "negative"))
        throw std::invalid_argument("Invalid scan axis, span or direction");
    const int horizontal = axis == "x" ? 0 : 1;
    // Validate Z and spacing before calculating any column counts.
    const auto vertical = makeDownwardScanTrajectory(origin, travel, spacing, repeats, 0.0);
    const double first = direction == "centered" ? origin[horizontal] + span / 2.0 : origin[horizontal];
    const double last = direction == "positive" ? origin[horizontal] + span :
        (direction == "negative" ? origin[horizontal] - span : origin[horizontal] - span / 2.0);
    std::vector<Eigen::Vector3d> path{origin};
    auto append_horizontal = [&](double value) {
        const double from = path.back()[horizontal];
        const int count = std::max(1, static_cast<int>(std::ceil(std::abs(value - from) / spacing)));
        for (int i = 1; i <= count; ++i)
        {
            Eigen::Vector3d point = origin;
            point[horizontal] = from + (value - from) * static_cast<double>(i) / count;
            path.push_back(point);
        }
    };
    if (first != origin[horizontal]) append_horizontal(first);
    const int intervals = static_cast<int>(std::ceil(span / spacing));
    for (int col = 0; col <= intervals; ++col)
    {
        const double value = first + (last >= first ? 1.0 : -1.0) * std::min(col * spacing, span);
        if (col > 0) append_horizontal(value);
        for (size_t i = 1; i < vertical.size(); ++i)
        {
            Eigen::Vector3d point = vertical[i].position;
            point[horizontal] = value;
            path.push_back(point);
        }
    }
    return path;
}
