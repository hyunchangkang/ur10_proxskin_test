#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

inline Eigen::Matrix3d rotationFromRpy(const Eigen::Vector3d &rpy)
{
    if (!rpy.allFinite()) throw std::invalid_argument("Invalid reference RPY");
    return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// Correct tool-Z tilt with the shortest rotation. Do not snap tool X/Y or yaw.
inline Eigen::Matrix3d nearestToolZAlignedRotation(const Eigen::Matrix3d &current,
                                                  const Eigen::Matrix3d &reference_from_base)
{
    if (!current.allFinite() || !reference_from_base.allFinite() ||
        (current.transpose() * current - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        (reference_from_base.transpose() * reference_from_base - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        std::abs(current.determinant() - 1.0) > 1e-6 ||
        std::abs(reference_from_base.determinant() - 1.0) > 1e-6)
        throw std::invalid_argument("Alignment requires proper rotation matrices");
    const Eigen::Matrix3d in_reference = reference_from_base * current;
    const Eigen::Vector3d target_z(0.0, 0.0, in_reference(2, 2) >= 0.0 ? 1.0 : -1.0);
    const auto correction = Eigen::Quaterniond::FromTwoVectors(in_reference.col(2), target_z);
    return reference_from_base.transpose() * correction.toRotationMatrix() * in_reference;
}

inline Eigen::Vector3d toolZAlignmentError(const Eigen::Matrix3d &target,
                                          const Eigen::Matrix3d &actual)
{
    const Eigen::AngleAxisd correction(
        Eigen::Quaterniond::FromTwoVectors(actual.col(2), target.col(2)));
    return correction.axis() * correction.angle();
}

inline std::vector<Eigen::Quaterniond> makeAlignmentRotations(
    const Eigen::Matrix3d &start, const Eigen::Matrix3d &goal, double max_step_rad)
{
    if (!std::isfinite(max_step_rad) || max_step_rad <= 0.0)
        throw std::invalid_argument("Alignment step must be positive");
    const Eigen::Quaterniond from(start), to(goal);
    const double angle = from.angularDistance(to);
    const int steps = std::max(1, static_cast<int>(std::ceil(angle / max_step_rad)));
    std::vector<Eigen::Quaterniond> rotations;
    for (int i = 0; i <= steps; ++i)
        rotations.push_back(from.slerp(static_cast<double>(i) / steps, to).normalized());
    return rotations;
}
