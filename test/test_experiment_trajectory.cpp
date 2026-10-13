#include <iostream>
#include <stdexcept>
#include "ur10_proxskin_test/experiment_trajectory.hpp"
#include "ur10_proxskin_test/joint_startup_plan.hpp"

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    const Eigen::Vector3d start(0.45, -0.2, 0.7);
    for (int repeats : {1, 3})
    {
        const auto path = makeDownwardScanTrajectory(start, 0.3, 0.001, repeats, 0.0);
        require((path.front().position - start).norm() < 1e-12, "must start at saved pose");
        require((path.back().position - start).norm() < 1e-12, "must return to saved pose");
        int bottom_visits = 0;
        for (size_t i = 0; i < path.size(); ++i)
        {
            const auto &p = path[i].position;
            require(std::abs(p.x() - start.x()) < 1e-12 && std::abs(p.y() - start.y()) < 1e-12,
                    "vertical motion must keep XY fixed");
            require(p.z() <= start.z() + 1e-12 && p.z() >= start.z() - 0.3 - 1e-12,
                    "Z must stay between saved start and exactly 30 cm below");
            if (std::abs(p.z() - (start.z() - 0.3)) < 1e-12) ++bottom_visits;
            if (i) require((p - path[i - 1].position).norm() <= 0.001 + 1e-12,
                           "path sampling must obey step size");
        }
        require(bottom_visits == repeats, "one bottom visit per round trip");
    }
    const auto stepped = makeDownwardScanTrajectory(start, 0.3, 0.001, 1, 2.0);
    int stops = 0;
    for (const auto &p : stepped)
        if (p.dwell_seconds > 0.0)
        {
            ++stops;
            const double cm = (start.z() - p.position.z()) * 100.0;
            require(std::abs(cm - std::round(cm)) < 1e-9, "stop only at whole cm positions");
        }
    require(stops == 61, "pause at start, all descending cm, and all ascending cm");
    for (double travel : {0.2, 0.1255, 0.0055})
    {
        const auto path = makeDownwardScanTrajectory(start, travel, 0.001, 2, 2.0);
        require((path.back().position - start).norm() < 1e-12,
                "arbitrary stepped distance must return to start");
        int bottom_visits = 0;
        for (size_t i = 0; i < path.size(); ++i)
        {
            const auto &point = path[i];
            const double offset = start.z() - point.position.z();
            require(offset >= -1e-12 && offset <= travel + 1e-12,
                    "stepped scan must not pass the requested endpoint");
            if (std::abs(offset - travel) < 1e-12)
            {
                ++bottom_visits;
                require(point.dwell_seconds == 2.0, "pause at fractional endpoint");
            }
            else if (point.dwell_seconds > 0.0)
                require(std::abs(offset * 100.0 - std::round(offset * 100.0)) < 1e-9,
                        "intermediate stops must stay on the 1 cm grid");
            if (i) require((point.position - path[i - 1].position).norm() <= 0.001 + 1e-12,
                           "fractional segments must obey sampling step size");
        }
        require(bottom_visits == 2, "one fractional bottom visit per round trip");
    }
    for (const std::string direction : {"centered", "positive", "negative"})
    {
        // Non-divisible X span verifies that the final short column step ends exactly at the boundary.
        const auto path = makeDownwardColumnTrajectory(start, 0.3, 0.0055, 0.002, 2, direction);
        int bottom_visits = 0;
        for (size_t i = 0; i < path.size(); ++i)
        {
            require(std::abs(path[i].y() - start.y()) < 1e-12, "X-Z columns must keep Y fixed");
            if (std::abs(path[i].z() - start.z() + 0.3) < 1e-12) ++bottom_visits;
            if (i && std::abs(path[i].x() - path[i - 1].x()) > 1e-12)
                require(std::abs(path[i].z() - start.z()) < 1e-12 &&
                        std::abs(path[i - 1].z() - start.z()) < 1e-12,
                        "X movement must happen at starting Z after complete round trips");
        }
        require(bottom_visits == 8, "four X columns, two round trips each");
        const double last_x = start.x() + (direction == "positive" ? 0.0055 :
            (direction == "negative" ? -0.0055 : -0.00275));
        require(std::abs(path.back().x() - last_x) < 1e-12 &&
                std::abs(path.back().z() - start.z()) < 1e-12, "correct final X and starting Z");
    }
    const auto single = makeDownwardColumnTrajectory(start, 0.3, 0.0, 0.001, 1, "centered");
    require((single.back() - start).norm() < 1e-12, "zero X span is one vertical column");
    for (double distance : {0.0, -0.3, std::numeric_limits<double>::quiet_NaN()})
    {
        bool rejected = false;
        try { makeDownwardScanTrajectory(start, distance, 0.001, 1, 0.0); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "invalid Z travel must be rejected");
    }
    StartupJoints current{0.1, -1.5, -1.5, -1.5, 1.5, -1.5};
    const std::vector<double> saved_arm{-85, -95, -80, 88, -70};
    const auto restore = makeJointStartupPlan(current, saved_arm, 0.1, 125.0, 0.2);
    require(!restore.held[0] && std::abs(restore.goal[0] - 0.2) < 1e-12,
            "experiment startup must restore saved J1 as well as J2..J6");
    const auto legacy = makeJointStartupPlan(current, saved_arm, 0.1, 125.0);
    require(legacy.held[0] && legacy.goal[0] == current[0], "legacy base mode must still hold J1");
    const auto base = makeJointStartupPlan(current, saved_arm, 0.1, 125.0,
                                          std::numeric_limits<double>::quiet_NaN(), true);
    require(base.held[0] && base.held[5], "menu base must hold J1 and J6");
    for (const auto &command : base.commands)
        require(command[0] == current[0] && command[5] == current[5],
                "J1 and J6 must stay unchanged throughout base motion");
    for (int joint = 1; joint < 5; ++joint)
        require(std::abs(base.goal[joint] - saved_arm[joint - 1] * DEG2RAD) < 1e-12,
                "holding J6 must retain the J2..J5 base targets");
    std::cout << "PASS: 30 cm down/up, repetitions, 1 cm stops, X-Z columns and saved joint restoration\n";
}
