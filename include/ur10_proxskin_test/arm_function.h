#ifndef UR10_PROXSKIN_TEST__ARM_FUNCTION_H_
#define UR10_PROXSKIN_TEST__ARM_FUNCTION_H_

#include <iostream>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;
using Eigen::Quaternion;

// Constants
constexpr double PI = 3.14159265359;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;
constexpr double SAMPLING_TIME_TRAJ = 0.008; // 125Hz (matches urcb2_driver)
constexpr int DoF = 6;

// Standard UR10 DH parameters (Craig/distal convention). These values are
// well-established (used consistently across ur10_driver/ur_10_control/etc. in
// this workspace) and were independently sanity-checked here: the resulting
// forward kinematics reach (~0.6-1.24 m) matches the published UR10 spec.
namespace UR10Params
{
    constexpr double d1 = 0.1273;
    constexpr double a2 = -0.612;
    constexpr double a3 = -0.5723;
    constexpr double d4 = 0.163941;
    constexpr double d5 = 0.1157;
    constexpr double d6 = 0.0922;
}

// UR10 kinematics + trapezoidal trajectory generator.
//
// NOTE on history: ur_10_control/arm_function.cpp (which this was originally ported
// from) contains a second, independent closed-form "forward()"/"inverse()" analytic
// IK pair. That pair was tested here (see proxskin_test's IK debugging session)
// and found to NOT round-trip correctly (forward(q) then inverse() does not recover
// q) -- it has a transcription bug somewhere and must not be used. Only the
// mechanical Craig-DH chain (T_craig()/forwardKinematics()), which was independently
// verified to produce physically sane reach values, is kept here. IK is solved
// numerically (finite-difference Jacobian + Levenberg-Marquardt) in proxskin_test
// instead of relying on any hand-derived analytic Jacobian/inverse formula.
class ArmControl
{
private:
    const double d1 = UR10Params::d1;
    const double a2 = UR10Params::a2;
    const double a3 = UR10Params::a3;
    const double d4 = UR10Params::d4;
    const double d5 = UR10Params::d5;
    const double d6 = UR10Params::d6;

    const double th_offset[DoF] = {0, 0, 0, 0, 0, 0};
    const double dh_d[DoF] = {d1, 0, 0, d4, d5, d6};
    const double dh_a[DoF] = {0, a2, a3, 0, 0, 0};
    const double dh_alpha[DoF] = {PI / 2, 0, 0, PI / 2, -PI / 2, 0};

    // Internal scratch state used by Joint2Cartesian()
    Matrix4d T06;
    Matrix3d R06;
    Quaterniond q06;
    Vector3d eul06;

public:
    ArmControl();
    ~ArmControl();

    // Rotation
    void Qua2Euler(Quaterniond &q, Vector3d &eul);
    void Euler2Qua(Vector3d &eul, Quaterniond &q);
    void Rot2Qua(Matrix3d &R, Quaterniond &q);
    void Eul2Rot(Vector3d &eul, Matrix3d &R);

    // Transformation
    void Joint2Cartesian(double *th, VectorXd &pose);
    void PoseEul2Tf(const VectorXd &pose, Ref<Matrix<double, 4, 4>> T_);

    // Kinematics - mechanical Craig-DH chain (verified convention)
    Matrix4d T_craig(double th, double d, double al, double a);
    void forwardKinematics(double *currentpos, Matrix4d &T);

    // Trajectory
    void JointTrajectoryTrapezoidal(double *th_ini, double *th_cmd, MatrixXd &th_out, MatrixXd &vel_out, double vel_des = 0.6, double vel_start = 0.0, double vel_end = 0.0);
};

#endif // UR10_PROXSKIN_TEST__ARM_FUNCTION_H_
