#include "ur10_proxskin_test/arm_function.h"

ArmControl::ArmControl()
{
    T06 = Matrix4d::Zero(4, 4);
    R06 = Matrix3d::Zero(3, 3);
    q06 = Quaterniond(0, 0, 0, 0);
    eul06 = Vector3d::Zero(3);
}

ArmControl::~ArmControl()
{
}

////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------Rotation----------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::Qua2Euler(Quaterniond &q, Vector3d &eul)
{
    // roll
    eul(0) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    // pitch
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        eul(1) = copysign(PI / 2, sinp);
    else
        eul(1) = asin(sinp);
    // yaw
    eul(2) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}

void ArmControl::Euler2Qua(Vector3d &eul, Quaterniond &q)
{
    double r_ = eul(0);
    double p_ = eul(1);
    double y_ = eul(2);

    q.w() = cos(y_ / 2) * cos(p_ / 2) * cos(r_ / 2) + sin(y_ / 2) * sin(p_ / 2) * sin(r_ / 2);
    q.x() = cos(y_ / 2) * cos(p_ / 2) * sin(r_ / 2) - sin(y_ / 2) * sin(p_ / 2) * cos(r_ / 2);
    q.y() = sin(y_ / 2) * cos(p_ / 2) * sin(r_ / 2) + cos(y_ / 2) * sin(p_ / 2) * cos(r_ / 2);
    q.z() = sin(y_ / 2) * cos(p_ / 2) * cos(r_ / 2) - cos(y_ / 2) * sin(p_ / 2) * sin(r_ / 2);
}

void ArmControl::Rot2Qua(Matrix3d &R, Quaterniond &q)
{
    // The ported ur_10_control version of this function determined each quaternion
    // component's sign via round((R21-R12)*1000)/1000 etc. That rounds any rotation
    // smaller than ~0.057 deg down to exactly zero, which silently turns near-identity
    // rotations into the identity quaternion -- this broke finite-difference Jacobians
    // (small perturbations) during IK debugging. Eigen's own matrix->quaternion
    // conversion is numerically robust for all rotation magnitudes, so use that instead.
    q = Quaterniond(R);
    q.normalize();
}

void ArmControl::Eul2Rot(Vector3d &eul, Matrix3d &R)
{
    double sr = sin(eul(0));
    double cr = cos(eul(0));
    double sp = sin(eul(1));
    double cp = cos(eul(1));
    double sy = sin(eul(2));
    double cy = cos(eul(2));
    R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr;
}

////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------Transformation-------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::Joint2Cartesian(double *th, VectorXd &pose)
{
    forwardKinematics(th, T06);

    R06 = T06.block(0, 0, 3, 3);
    Rot2Qua(R06, q06);
    Qua2Euler(q06, eul06);
    pose << T06(0, 3), T06(1, 3), T06(2, 3), eul06(0), eul06(1), eul06(2);
}

void ArmControl::PoseEul2Tf(const VectorXd &pose, Ref<Matrix<double, 4, 4>> T_)
{
    Vector3d eul_temp;
    eul_temp << pose(3), pose(4), pose(5);
    Matrix3d R_temp;
    Eul2Rot(eul_temp, R_temp);
    T_.block(0, 0, 3, 3) = R_temp;
    T_(0, 3) = pose(0);
    T_(1, 3) = pose(1);
    T_(2, 3) = pose(2);
    T_(3, 0) = 0;
    T_(3, 1) = 0;
    T_(3, 2) = 0;
    T_(3, 3) = 1;
}

////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Kinematics---------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////

// Standard Craig-DH homogeneous transform between consecutive joint frames.
Matrix4d ArmControl::T_craig(double th, double d, double al, double a)
{
    Matrix4d T_craig_;
    T_craig_ << cos(th), -sin(th), 0, a,
        sin(th) * cos(al), cos(th) * cos(al), -sin(al), -d * sin(al),
        sin(th) * sin(al), cos(th) * sin(al), cos(al), d * cos(al),
        0, 0, 0, 1;
    return T_craig_;
}

void ArmControl::forwardKinematics(double *currentpos, Matrix4d &T)
{
    double theta[DoF];
    for (int i = 0; i < DoF; i++)
    {
        theta[i] = currentpos[i] + th_offset[i];
    }

    // T_craig(th, d, al, a) implements the modified/Craig-DH transform
    // Rx(al) * Tx(a) * Rz(th) * Tz(d), i.e. it applies the incoming twist/link-length
    // BEFORE this joint's own rotation. That means (al, a) for joint i must be the
    // PREVIOUS row's values (dh_alpha[i-1], dh_a[i-1]), not dh_alpha[i]/dh_a[i] --
    // using matched indices collapses joints 5 and 6 onto a single rotation axis
    // (verified numerically: their Jacobian columns became identical for every tested
    // configuration) because dh_alpha[4]=-pi/2 immediately followed by dh_alpha[5]=0
    // leaves the last joint with no twist relative to the one before it. Shifting by
    // one index (first joint gets no incoming twist/offset) resolves this and matches
    // the standard UR10 modified-DH parameterization.
    Matrix4d T_ = Matrix4d::Identity();
    double alpha_prev = 0.0;
    double a_prev = 0.0;
    for (int i = 0; i < DoF; i++)
    {
        T_ = T_ * T_craig(theta[i], dh_d[i], alpha_prev, a_prev);
        alpha_prev = dh_alpha[i];
        a_prev = dh_a[i];
    }
    T = T_;
}

////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Trajectory---------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::JointTrajectoryTrapezoidal(double *th_ini, double *th_cmd, MatrixXd &th_out, MatrixXd &vel_out, double vel_des, double vel_start, double vel_end)
{
    double acc_des = 2 * vel_des;

    double th_diff[DoF];
    double max_th_error = 0;
    for (int i = 0; i < DoF; i++)
    {
        th_diff[i] = th_cmd[i] - th_ini[i];
        while (th_diff[i] > PI)
            th_diff[i] -= 2 * PI;
        while (th_diff[i] < -PI)
            th_diff[i] += 2 * PI;

        if (max_th_error < fabs(th_diff[i]))
            max_th_error = fabs(th_diff[i]);
    }

    if (max_th_error < 0.001)
    {
        th_out.resize(1, DoF);
        vel_out.resize(1, DoF);
        for (int i = 0; i < DoF; i++)
        {
            th_out(0, i) = th_ini[i];
            vel_out(0, i) = 0;
        }
        return;
    }

    vel_start = std::max(0.0, std::min(vel_des, vel_start));
    vel_end = std::max(0.0, std::min(vel_des, vel_end));

    double vel_start_ratio = vel_start / vel_des;
    double vel_end_ratio = vel_end / vel_des;

    double Tf = max_th_error / vel_des;
    double Tb_accel = (1.0 - vel_start_ratio) * vel_des / acc_des;
    double Tb_decel = (1.0 - vel_end_ratio) * vel_des / acc_des;

    if (acc_des >= 4 * max_th_error / pow(Tf, 2))
    {
        Tb_accel = Tf / 2 - sqrt(pow(acc_des * Tf, 2) - 4 * acc_des * max_th_error) / (2 * acc_des);
        Tb_decel = Tb_accel;
    }

    int acc_step = static_cast<int>(round(Tb_accel / SAMPLING_TIME_TRAJ));
    int decel_step = static_cast<int>(round(Tb_decel / SAMPLING_TIME_TRAJ));
    int lin_step = static_cast<int>(round((Tf - Tb_accel - Tb_decel) / SAMPLING_TIME_TRAJ));
    int step = acc_step + lin_step + decel_step + 1;

    th_out.resize(step, DoF);
    vel_out.resize(step, DoF);

    for (int i = 0; i < DoF; i++)
    {
        double vel_lin = th_diff[i] / (Tf - (Tb_accel + Tb_decel) / 2.0);
        double acc_joint = (fabs(Tb_accel) > 1e-6) ? (vel_lin * (1.0 - vel_start_ratio)) / Tb_accel : 0;
        double decel_joint = (fabs(Tb_decel) > 1e-6) ? (vel_lin * (1.0 - vel_end_ratio)) / Tb_decel : 0;

        double vel_start_i = vel_lin * vel_start_ratio;
        double vel_end_i = vel_lin * vel_end_ratio;

        for (int j = 0; j < step; j++)
        {
            double t = j * SAMPLING_TIME_TRAJ;

            if (j <= acc_step)
            {
                th_out(j, i) = th_ini[i] + vel_start_i * t + 0.5 * acc_joint * t * t;
                vel_out(j, i) = vel_start_i + acc_joint * t;
            }
            else if (j <= acc_step + lin_step)
            {
                double t_lin = (j - acc_step) * SAMPLING_TIME_TRAJ;
                double pos_after_accel = th_ini[i] + vel_start_i * Tb_accel + 0.5 * acc_joint * Tb_accel * Tb_accel;
                th_out(j, i) = pos_after_accel + vel_lin * t_lin;
                vel_out(j, i) = vel_lin;
            }
            else
            {
                double t_decel = (j - acc_step - lin_step) * SAMPLING_TIME_TRAJ;
                double pos_before_decel = th_ini[i] + vel_start_i * Tb_accel + 0.5 * acc_joint * Tb_accel * Tb_accel + vel_lin * (Tf - Tb_accel - Tb_decel);
                th_out(j, i) = pos_before_decel + vel_lin * t_decel - 0.5 * decel_joint * t_decel * t_decel;
                vel_out(j, i) = vel_lin - decel_joint * t_decel;
            }
        }

        th_out(step - 1, i) = th_ini[i] + th_diff[i];
        vel_out(step - 1, i) = vel_end_i;
    }
}
