#include <iostream>
#include <stdexcept>
#include "ur10_proxskin_test/pose_alignment.hpp"

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    const double deg = std::acos(-1.0) / 180.0;
    const auto world_from_base = rotationFromRpy(Eigen::Vector3d(0, 0, 180) * deg);
    const auto measured = rotationFromRpy(Eigen::Vector3d(176.1, 2.45, -105.17) * deg);
    const auto goal = nearestToolZAlignedRotation(measured, world_from_base);
    require(((world_from_base * goal).col(2) + Eigen::Vector3d::UnitZ()).norm() < 1e-10,
            "live measured pose must align tool Z downward");
    require(goal(2, 2) < -0.99, "downward tool must remain downward after alignment");
    require(Eigen::AngleAxisd(goal * measured.transpose()).angle() < 20 * deg,
            "alignment must correct tilt without flipping the tool");
    for (double yaw : {-157.3, -35.0, 0.0, 28.5, 89.0, 170.0})
        for (double roll : {0.0, 180.0})
        {
            const auto aligned = rotationFromRpy(Eigen::Vector3d(roll, 0.0, yaw) * deg);
            require((nearestToolZAlignedRotation(aligned, world_from_base) - aligned).norm() < 1e-12,
                    "arbitrary yaw must stay unchanged when tool Z is already aligned");
        }
    for (const Eigen::Vector3d &rpy : {Eigen::Vector3d(2, -3, 4), Eigen::Vector3d(180, 89.999, -90),
                                     Eigen::Vector3d(-179, -1, 179), Eigen::Vector3d(45, 30, -60)})
    {
        const auto current = rotationFromRpy(rpy * deg);
        const auto aligned = nearestToolZAlignedRotation(current, world_from_base);
        const Eigen::Matrix3d axes = world_from_base * aligned;
        require(std::abs(axes.determinant() - 1.0) < 1e-12, "alignment must be a proper rotation");
        require(std::abs(axes(2, 2)) > 0.999999, "tool Z must be parallel to world Z");
        const Eigen::AngleAxisd correction(aligned * current.transpose());
        if (correction.angle() > 1e-10)
            require(std::abs(correction.axis().dot(current.col(2))) < 1e-10,
                    "tilt correction must not add a rotation about tool Z");
        require((nearestToolZAlignedRotation(aligned, world_from_base) - aligned).norm() < 1e-12,
                "already-aligned poses must stay aligned");
        const auto path = makeAlignmentRotations(current, aligned, 0.25 * deg);
        require(path.front().angularDistance(Eigen::Quaterniond(current)) < 1e-10,
                "orientation path starts at the measured orientation");
        require(path.back().angularDistance(Eigen::Quaterniond(aligned)) < 1e-10,
                "orientation path ends at the aligned orientation");
        for (size_t i = 1; i < path.size(); ++i)
            require(path[i-1].angularDistance(path[i]) <= 0.25 * deg + 1e-10,
                    "intermediate rotation steps must be small");
    }
    const auto arbitrary_reference = rotationFromRpy(Eigen::Vector3d(15, -10, 35) * deg);
    const auto rotated_goal = nearestToolZAlignedRotation(measured, arbitrary_reference);
    const Eigen::Matrix3d axes = arbitrary_reference * rotated_goal;
    require(std::abs(axes(2, 2)) > 0.999999,
            "tool-Z alignment must work with rotated reference axes");
    const Eigen::Matrix3d twisted = goal * Eigen::AngleAxisd(73 * deg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    require(toolZAlignmentError(goal, twisted).norm() < 1e-10,
            "tool-Z error must ignore rotation about the tool axis");
    require(toolZAlignmentError(goal, measured).norm() > 1e-3,
            "tool-Z error must still detect tilt");
    bool rejected = false;
    try { nearestToolZAlignedRotation(Eigen::Matrix3d::Zero(), world_from_base); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "invalid rotations must be rejected");
    std::cout << "PASS: tool-Z tilt alignment, arbitrary yaw retention and smooth rotations\n";
}
