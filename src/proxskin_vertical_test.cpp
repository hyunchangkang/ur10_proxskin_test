/**
 * @file proxskin_vertical_test.cpp
 * @brief UR10 sensor scan along base-frame Z only, with fixed TCP X and Y.
 *
 * This ROS2 port talks directly to urcb2_driver instead of ur_modern_driver:
 *   Subscribe: {robot_name}/joint_states (sensor_msgs/JointState)
 *   Publish:   {robot_name}/targetJ      (std_msgs/Float64MultiArray)
 * urcb2_driver streams targetJ to the robot via servoj at 125Hz, so there is no
 * FollowJointTrajectory action server here -- motion is streamed directly.
 *
 * This node also connects to the eCAN TCP server using the Prox_sensor_React protocol and directly tracks
 * configured CAN IDs while the vertical scan is running.
 *
 * Run:
 *   ros2 run urcb2_driver single_arm
 *   ros2 launch ur10_proxskin_test proxskin_soc_test.launch.py
 */

#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <netinet/in.h>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ur10_proxskin_test/arm_function.h"
#include "ur10_proxskin_test/vertical_scan_trajectory.hpp"
#include "ur10_proxskin_test/experiment_trajectory.hpp"
#include "ur10_proxskin_test/experiment_csv.hpp"
#include "ur10_proxskin_test/ecan_tcp_client.hpp"
#include "ur10_proxskin_test/experiment_arrival.hpp"
#include "ur10_proxskin_test/experiment_cancel.hpp"
#include "ur10_proxskin_test/experiment_return.hpp"
#include "ur10_proxskin_test/joint_startup.hpp"
#include "ur10_proxskin_test/sensor_zero.hpp"
#include "ur10_proxskin_test/pose_alignment.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ProxskinVerticalTestNode : public rclcpp::Node
{
public:
    ProxskinVerticalTestNode() : Node("proxskin_vertical_test")
    {
        declare_parameter<std::string>("robot_name", "UR10_right");
        declare_parameter<bool>("enable_ecan", false);
        declare_parameter<std::string>("csv_save_dir", "/home/ur/ros2_ws/src/ur10_proxskin_test/data");
        declare_parameter<std::string>("can_server_ip", "192.168.253.225");
        declare_parameter<int>("can_server_port", 4001);
        declare_parameter<double>("reconnect_interval_sec", 1.0);
        declare_parameter<double>("socket_connect_timeout_sec", 1.0);
        declare_parameter<double>("sensor_stale_timeout_sec", 0.5);
        declare_parameter<std::vector<int64_t>>("tracked_can_ids", {0x41, 0x42, 0x43});
        declare_parameter<double>("control_rate", 125.0);
        // Offsets are relative to the TCP pose captured immediately before motion.
        declare_parameter<double>("z_min_offset_mm", 10.0);
        declare_parameter<double>("z_max_offset_mm", 300.0);
        declare_parameter<double>("z_step_mm", 1.0);
        declare_parameter<int>("repeat_count", 3);
        declare_parameter<double>("dwell_seconds", 0.0);
        declare_parameter<std::string>("z_direction", "up");
        declare_parameter<double>("max_actual_xy_error_mm", 5.0);
        declare_parameter<double>("scan_speed_mm_s", 50.0);
        declare_parameter<double>("max_joint_velocity_rad_s", 1.0);
        declareJointStartupParameters(*this);
        declare_parameter<int>("csv_sensor_can_id", 0x41);
        declare_parameter<std::string>("csv_sensor_channel", "front");
        declare_parameter<double>("calibration_speed_mm_s", 10.0);
        // Current ur_description: world <- controller base is a 180 deg Z rotation.
        declare_parameter<std::vector<double>>("alignment_world_from_base_rpy_deg", {0.0, 0.0, 180.0});
        declare_parameter<double>("alignment_angular_speed_deg_s", 5.0);
        declare_parameter<double>("alignment_joint_speed_rad_s", 0.1);
        declare_parameter<double>("alignment_max_position_error_mm", 5.0);

        robot_name_ = get_parameter("robot_name").as_string();
        enable_ecan_ = get_parameter("enable_ecan").as_bool();
        experiment_csv_enabled_ = get_parameter("experiment_start_enabled").as_bool();
        csv_sensor_can_id_ = get_parameter("csv_sensor_can_id").as_int();
        csv_sensor_channel_ = get_parameter("csv_sensor_channel").as_string();
        if (csv_sensor_can_id_ < -1 || csv_sensor_can_id_ > 255 ||
            (csv_sensor_channel_ != "front" && csv_sensor_channel_ != "back"))
            throw std::invalid_argument("Invalid CSV sensor selection");
        csv_save_dir_ = get_parameter("csv_save_dir").as_string();
        can_server_ip_ = get_parameter("can_server_ip").as_string();
        can_server_port_ = get_parameter("can_server_port").as_int();
        sensor_stale_timeout_sec_ = get_parameter("sensor_stale_timeout_sec").as_double();
        if (can_server_port_ < 1 || can_server_port_ > 65535 ||
            !std::isfinite(sensor_stale_timeout_sec_) || sensor_stale_timeout_sec_ <= 0 ||
            !std::isfinite(get_parameter("reconnect_interval_sec").as_double()) ||
            get_parameter("reconnect_interval_sec").as_double() <= 0 ||
            !std::isfinite(get_parameter("socket_connect_timeout_sec").as_double()) ||
            get_parameter("socket_connect_timeout_sec").as_double() <= 0)
            throw std::invalid_argument("Invalid eCAN connection or freshness parameters");
        tracked_can_ids_ = sanitizeTrackedCanIds(
            get_parameter("tracked_can_ids").as_integer_array());
        if (csv_sensor_can_id_ >= 0 && std::find(tracked_can_ids_.begin(), tracked_can_ids_.end(),
                static_cast<uint32_t>(csv_sensor_can_id_)) == tracked_can_ids_.end())
            tracked_can_ids_.push_back(static_cast<uint32_t>(csv_sensor_can_id_));
        control_rate_ = get_parameter("control_rate").as_double();
        for (uint32_t can_id : tracked_can_ids_)
        {
            tracked_can_frames_.emplace_back(can_id);
        }

        for (int i = 0; i < DoF; i++)
        {
            current_joint_pos_[i] = 0.0;
            current_joint_vel_[i] = 0.0;
        }

        std::string joint_states_topic = robot_name_ + "/joint_states";
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, 10,
            std::bind(&ProxskinVerticalTestNode::jointStateCallback, this, _1));

        std::string target_topic = robot_name_ + "/targetJ";
        target_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(target_topic, 10);
        if (enable_ecan_)
        {
            can_status_timer_ = create_wall_timer(
                2s, std::bind(&ProxskinVerticalTestNode::logTrackedCanSummary, this));
        }

        RCLCPP_INFO(get_logger(), "Robot name: %s", robot_name_.c_str());
        RCLCPP_INFO(get_logger(), "Subscribing to: %s", joint_states_topic.c_str());
        RCLCPP_INFO(get_logger(), "Publishing to:  %s", target_topic.c_str());
        if (enable_ecan_)
        {
            RCLCPP_INFO(get_logger(), "eCAN TCP client will connect to %s:%d", can_server_ip_.c_str(), can_server_port_);
            RCLCPP_INFO(get_logger(), "Tracking CAN IDs: %s", formatTrackedCanIds(tracked_can_ids_).c_str());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "eCAN is disabled; experiment CSV records robot distance with an empty sensor column.");
        }

        if (experiment_csv_enabled_)
            csv_timer_ = create_wall_timer(10ms, [this]() { logExperimentCsvRow(); });
        running_ = true;
        if (enable_ecan_)
        {
            ecan_thread_ = std::thread(&ProxskinVerticalTestNode::ecanReceiveLoop, this);
        }
        control_thread_ = std::thread(&ProxskinVerticalTestNode::controlLoop, this);
    }

    ~ProxskinVerticalTestNode() override
    {
        running_ = false;
        ecan_running_ = false;
        if (ecan_thread_.joinable())
        {
            ecan_thread_.join();
        }
        if (control_thread_.joinable())
        {
            control_thread_.join();
        }
        stopCsvLogging();
    }

private:
    struct TrackedCanFrame
    {
        TrackedCanFrame() = default;
        explicit TrackedCanFrame(uint32_t id) : can_id(id) {}

        uint32_t can_id;
        uint32_t front_be_u32 = 0;
        uint32_t back_be_u32 = 0;
        std::chrono::steady_clock::time_point last_seen{};
        bool received = false;
    };

    std::vector<uint32_t> sanitizeTrackedCanIds(const std::vector<int64_t> &raw_ids)
    {
        std::vector<uint32_t> ids;
        ids.reserve(raw_ids.size());
        for (int64_t raw_id : raw_ids)
        {
            if (raw_id < 0 || raw_id > 0xFF)
            {
                RCLCPP_WARN(get_logger(), "Ignoring invalid tracked CAN ID value: %ld", raw_id);
                continue;
            }

            const auto can_id = static_cast<uint32_t>(raw_id);
            if (std::find(ids.begin(), ids.end(), can_id) == ids.end())
            {
                ids.push_back(can_id);
            }
        }

        if (ids.empty())
        {
            RCLCPP_WARN(get_logger(), "No valid tracked_can_ids provided. Falling back to 0x41 0x42 0x43.");
            ids = {0x41, 0x42, 0x43};
        }

        return ids;
    }

    std::string formatTrackedCanIds(const std::vector<uint32_t> &ids) const
    {
        std::ostringstream oss;
        for (size_t i = 0; i < ids.size(); i++)
        {
            if (i > 0)
            {
                oss << " ";
            }
            oss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << ids[i] << std::dec;
        }
        return oss.str();
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < static_cast<size_t>(DoF))
            return;
        for (int i = 0; i < DoF; ++i)
            if (!std::isfinite(msg->position[i]))
                return;
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < DoF; ++i)
        {
            current_joint_pos_[i] = msg->position[i];
            if (msg->velocity.size() > static_cast<size_t>(i))
            {
                current_joint_vel_[i] = msg->velocity[i];
            }
        }
        last_feedback_time_ = std::chrono::steady_clock::now();
        ++feedback_sequence_;
        joint_state_received_ = true;
    }

    void updateTrackedCanFrame(uint32_t can_id, uint32_t front_be_u32, uint32_t back_be_u32)
    {
        bool logging_enabled = can_logging_enabled_.load();
        bool first_reception = false;
        {
            std::lock_guard<std::mutex> lock(can_mutex_);
            for (auto &tracked : tracked_can_frames_)
            {
                if (tracked.can_id != can_id)
                {
                    continue;
                }

                first_reception = !tracked.received;
                tracked.received = true;
                tracked.front_be_u32 = front_be_u32;
                tracked.back_be_u32 = back_be_u32;
                tracked.last_seen = std::chrono::steady_clock::now();
                break;
            }
        }

        if (logging_enabled && first_reception)
        {
            RCLCPP_INFO(get_logger(), "Started receiving tracked CAN ID 0x%02X", can_id);
        }
    }

    void logTrackedCanSummary()
    {
        if (!can_logging_enabled_.load())
        {
            return;
        }

        std::vector<TrackedCanFrame> snapshot;
        {
            std::lock_guard<std::mutex> lock(can_mutex_);
            snapshot = tracked_can_frames_;
        }

        bool any_received = false;
        for (const auto &tracked : snapshot)
        {
            any_received = any_received || tracked.received;
        }

        if (!any_received)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Waiting for CAN IDs %s from eCAN TCP on %s:%d",
                formatTrackedCanIds(tracked_can_ids_).c_str(), can_server_ip_.c_str(), can_server_port_);
            return;
        }

        auto now_time = std::chrono::steady_clock::now();
        std::ostringstream oss;
        oss << "Tracked CAN summary:";
        for (const auto &tracked : snapshot)
        {
            oss << " 0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << tracked.can_id
                << std::dec;
            if (!tracked.received)
            {
                oss << "(missing)";
                continue;
            }

            double age_s = std::chrono::duration<double>(now_time - tracked.last_seen).count();
            oss << "(front_be=" << tracked.front_be_u32
                << ", back_be=" << tracked.back_be_u32
                << ", age=" << std::fixed << std::setprecision(3) << age_s << "s)";
        }

        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }

    bool hasTrackedCanData() const
    {
        std::lock_guard<std::mutex> lock(can_mutex_);
        for (const auto &tracked : tracked_can_frames_)
        {
            if (tracked.received &&
                (!experiment_csv_enabled_ || csv_sensor_can_id_ < 0 ||
                 tracked.can_id == static_cast<uint32_t>(csv_sensor_can_id_)) &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - tracked.last_seen).count()
                    <= sensor_stale_timeout_sec_)
            {
                return true;
            }
        }
        return false;
    }

    void waitForEcanReady()
    {
        RCLCPP_INFO(get_logger(),
                    "Waiting for eCAN readiness: TCP connected and fresh selected CAN sensor data received...");

        while (rclcpp::ok() && running_)
        {
            const bool connected = ecan_client_connected_.load();
            const bool has_data = hasTrackedCanData();
            if (connected && has_data)
            {
                RCLCPP_INFO(get_logger(), "eCAN is ready. TCP connected and tracked CAN data has been received.");
                logTrackedCanSummary();
                return;
            }

            if (!connected)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Still waiting for eCAN TCP connection to %s:%d...",
                    can_server_ip_.c_str(), can_server_port_);
            }
            else if (!has_data)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "eCAN TCP is connected, but the selected sensor has no fresh data (tracked CAN IDs: %s).",
                    formatTrackedCanIds(tracked_can_ids_).c_str());
            }

            std::this_thread::sleep_for(100ms);
        }
    }

    void ecanReceiveLoop()
    {
        runEcanTcpClient(*this, can_server_ip_, can_server_port_,
            get_parameter("reconnect_interval_sec").as_double(),
            get_parameter("socket_connect_timeout_sec").as_double(), ecan_client_connected_,
            [this]() { return ecan_running_.load(); },
            [this](const EcanFrame &frame) { updateTrackedCanFrame(frame.can_id, frame.front, frame.back); },
            [this]() {
                std::lock_guard<std::mutex> lock(can_mutex_);
                for (auto &frame : tracked_can_frames_) frame.received = false;
            });
    }

    bool startCsvLogging()
    {
        std::lock_guard<std::mutex> csv_lock(csv_mutex_);
        if (csv_file_.is_open())
        {
            return true;
        }

        try
        {
            std::filesystem::create_directories(csv_save_dir_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create CSV directory %s: %s",
                         csv_save_dir_.c_str(), e.what());
            return false;
        }

        const auto now_sys = std::chrono::system_clock::now();
        const auto now_time_t = std::chrono::system_clock::to_time_t(now_sys);
        std::tm tm_buf{};
        localtime_r(&now_time_t, &tm_buf);

        std::ostringstream filename;
        filename << "proxskin_vertical_" << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S") << ".csv";
        const auto csv_name = experiment_csv_enabled_ ? experimentCsvFilename((get_parameter("dwell_seconds").as_double() > 0.0 ? "stepped" : "vertical")) : filename.str();
        csv_file_path_ = (std::filesystem::path(csv_save_dir_) / csv_name).string();

        csv_file_.open(csv_file_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_file_path_.c_str());
            csv_file_path_.clear();
            return false;
        }

        csv_start_time_ = std::chrono::steady_clock::now();
        if (experiment_csv_enabled_)
        {
            csv_file_.imbue(std::locale::classic());
            csv_first_sample_time_ = {};
            writeExperimentCsvHeader(csv_file_);
            csv_file_.flush();
            RCLCPP_INFO(get_logger(), "CSV logging started: %s", csv_file_path_.c_str());
            RCLCPP_INFO(get_logger(), "Distance reference: start %.2f mm, bottom 0.00 mm; CSV samples latest robot feedback and CAN sensor at 100 Hz.", csv_travel_mm_);
            return true;
        }
        csv_file_ << "time,x,y,z";
        for (uint32_t can_id : tracked_can_ids_)
        {
            csv_file_ << "," << std::uppercase << std::hex << can_id << std::dec << "_front"
                      << "," << std::uppercase << std::hex << can_id << std::dec << "_back";
        }
        csv_file_ << "\n";
        csv_file_.flush();
        RCLCPP_INFO(get_logger(), "CSV logging started: %s", csv_file_path_.c_str());
        return true;
    }

    void stopCsvLogging()
    {
        std::lock_guard<std::mutex> csv_lock(csv_mutex_);
        if (!csv_file_.is_open())
        {
            return;
        }

        csv_file_.close();
        if (!csv_file_path_.empty())
        {
            RCLCPP_INFO(get_logger(), "CSV logging finished: %s", csv_file_path_.c_str());
        }
    }

    // A 100 Hz timer co-samples the latest measured robot and sensor values.
    void logExperimentCsvRow()
    {
        std::lock_guard<std::mutex> csv_lock(csv_mutex_);
        if (!csv_file_.is_open() || !running_ || !rclcpp::ok()) return;
        const auto sample_time = std::chrono::steady_clock::now();
        double joints[DoF];
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!joint_state_received_ || sample_time - last_feedback_time_ > 250ms) return;
            std::copy_n(current_joint_pos_, DoF, joints);
        }
        if (csv_first_sample_time_ == std::chrono::steady_clock::time_point{})
            csv_first_sample_time_ = sample_time;
        const double elapsed_s = std::chrono::duration<double>(sample_time - csv_first_sample_time_).count();
        VectorXd pose = VectorXd::Zero(6);
        csv_arm_.Joint2Cartesian(joints, pose);
        const double distance_mm = experimentDistanceMm(pose.z(), csv_reference_z_m_, csv_travel_mm_);
        std::optional<uint32_t> sensor;
        if (enable_ecan_ && ecan_client_connected_ && csv_sensor_can_id_ >= 0)
        {
            std::lock_guard<std::mutex> lock(can_mutex_);
            for (const auto &frame : tracked_can_frames_)
                if (frame.can_id == static_cast<uint32_t>(csv_sensor_can_id_) && frame.received &&
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - frame.last_seen).count()
                        <= sensor_stale_timeout_sec_)
                    sensor = csv_sensor_channel_ == "front" ? frame.front_be_u32 : frame.back_be_u32;
        }
        writeExperimentCsvRow(csv_file_, elapsed_s, distance_mm, sensor);
        if (sample_time - last_csv_flush_time_ >= 1s)
        {
            csv_file_.flush();
            last_csv_flush_time_ = sample_time;
            RCLCPP_INFO(get_logger(), "Time %.3f s | Robot distance %.2f mm", elapsed_s, distance_mm);
        }
        if (!csv_file_)
        {
            RCLCPP_ERROR(get_logger(), "CSV write failed; stopping experiment: %s", csv_file_path_.c_str());
            running_ = false;
        }
    }

    void logCsvRow()
    {
        if (experiment_csv_enabled_) return;  // Recorded by the independent 100 Hz timer.
        if (!can_logging_enabled_.load() || !csv_file_.is_open())
        {
            return;
        }

        double joints[DoF];
        getCurrentJoints(joints);

        VectorXd pose_eul = VectorXd::Zero(6);
        arm_.Joint2Cartesian(joints, pose_eul);

        std::vector<TrackedCanFrame> snapshot;
        {
            std::lock_guard<std::mutex> lock(can_mutex_);
            snapshot = tracked_can_frames_;
        }

        const auto elapsed_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - csv_start_time_).count();

        csv_file_ << std::fixed << std::setprecision(6)
                  << elapsed_sec
                  << ","
                  << pose_eul(0)
                  << "," << pose_eul(1)
                  << "," << pose_eul(2);

        for (const auto &tracked : snapshot)
        {
            csv_file_ << "," << tracked.front_be_u32
                      << "," << tracked.back_be_u32;
        }
        csv_file_ << "\n";
    }

    void publishTargetJoints(const double *target)
    {
        if (!running_ || !rclcpp::ok()) return;
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(DoF);
        for (int i = 0; i < DoF; i++)
        {
            msg.data[i] = target[i];
        }
        target_pub_->publish(msg);
    }

    void getCurrentJoints(double *out)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < DoF; i++)
        {
            out[i] = current_joint_pos_[i];
        }
    }

    bool getFreshCurrentJoints(double *out)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!joint_state_received_.load() ||
            std::chrono::steady_clock::now() - last_feedback_time_ > 250ms)
        {
            RCLCPP_ERROR(get_logger(), "Joint feedback is missing or older than 250 ms; aborting scan.");
            return false;
        }
        for (int i = 0; i < DoF; ++i)
            out[i] = current_joint_pos_[i];
        return true;
    }

    // Every position fixes X/Y; dwell marks are attached to 1 cm stops and endpoints.
    std::vector<VerticalScanPoint> generateVerticalTrajectory(const Vector3d &origin)
    {
        const std::string direction = get_parameter("z_direction").as_string();
        if (direction != "up" && direction != "down")
        {
            RCLCPP_ERROR(get_logger(), "z_direction must be up or down.");
            return {};
        }
        try
        {
            if (get_parameter("experiment_start_enabled").as_bool())
                return makeDownwardScanTrajectory(origin,
                    get_parameter("z_max_offset_mm").as_double() / 1000.0,
                    get_parameter("z_step_mm").as_double() / 1000.0,
                    get_parameter("repeat_count").as_int(),
                    get_parameter("dwell_seconds").as_double());
            return makeVerticalScanTrajectory(
                origin,
                get_parameter("z_min_offset_mm").as_double() / 1000.0,
                get_parameter("z_max_offset_mm").as_double() / 1000.0,
                get_parameter("z_step_mm").as_double() / 1000.0,
                get_parameter("repeat_count").as_int(),
                direction == "up" ? 1.0 : -1.0,
                get_parameter("dwell_seconds").as_double());
        }
        catch (const std::invalid_argument &error)
        {
            RCLCPP_ERROR(get_logger(), "%s", error.what());
            return {};
        }
    }

    // 6D pose error [pos(3); rotation-vector(3)] between two [x,y,z,roll,pitch,yaw]
    // poses. Orientation error goes through a rotation-matrix/quaternion difference
    // (not raw Euler subtraction) so it stays well-behaved near +-180 deg wraps.
    VectorXd poseErrorVec(const VectorXd &target_eul, const VectorXd &current_eul)
    {
        Vector3d pos_err = target_eul.head<3>() - current_eul.head<3>();

        Vector3d eul_t = target_eul.tail<3>();
        Vector3d eul_c = current_eul.tail<3>();
        Matrix3d R_t, R_c;
        arm_.Eul2Rot(eul_t, R_t);
        arm_.Eul2Rot(eul_c, R_c);
        Matrix3d R_err = R_t * R_c.transpose();
        Quaterniond q_err(R_err);
        q_err.normalize();

        Vector3d rot_err;
        double sin_half = q_err.vec().norm();
        if (sin_half < 1e-9)
        {
            rot_err.setZero();
        }
        else
        {
            double angle = 2.0 * atan2(sin_half, q_err.w());
            while (angle > PI)
                angle -= 2 * PI;
            while (angle < -PI)
                angle += 2 * PI;
            rot_err = (angle / sin_half) * q_err.vec();
        }

        VectorXd e(6);
        e << pos_err, rot_err;
        return e;
    }

    // Numerical IK via finite-difference Jacobian + Levenberg-Marquardt.
    // Deliberately avoids any hand-derived analytic Jacobian or closed-form inverse:
    // both of ur_10_control's ported alternatives (Jacobian()+Inverse_Newton_Quat, and
    // the closed-form forward()/inverse() pair) were tested here and found to NOT
    // round-trip correctly -- see arm_function.h for details. The Jacobian below is
    // computed by numerically differentiating forwardKinematics() itself, so it is
    // self-consistent by construction, and steps that don't reduce error are rejected
    // (increasing damping) instead of always being accepted.
    //
    // `num_free_joints` lets joints beyond that index be held fixed at `seed`'s value
    // (used to lock the wrist joints -- see fix_wrist_joints). When fewer than 6 joints
    // are free, orientation generally can't be independently satisfied too, so
    // `position_only` drops orientation; `tool_z_only` constrains tool-Z tilt
    // while allowing rotation about that axis (five constraints with J6 held).
    bool solveIKNumeric(const VectorXd &target_pose, const double *seed, double *out,
                        int max_iter = 200, int num_free_joints = DoF, bool position_only = false,
                        bool tool_z_only = false)
    {
        constexpr double H = 1e-6;        // finite-difference step (rad)
        constexpr double MAX_STEP = 0.05; // per-joint step clamp (rad) per iteration
        constexpr double POS_TOL = 5e-5;  // m
        constexpr double ANG_TOL = 5e-4;  // rad

        double cur[DoF];
        for (int i = 0; i < DoF; i++)
            cur[i] = seed[i];

        auto errVec = [&](const VectorXd &pose) -> VectorXd
        {
            VectorXd full = poseErrorVec(target_pose, pose);
            if (tool_z_only)
                full.tail<3>() = toolZAlignmentError(rotationFromRpy(target_pose.tail<3>()),
                                                    rotationFromRpy(pose.tail<3>()));
            return position_only ? VectorXd(full.head<3>()) : full;
        };
        auto converged = [&](const VectorXd &err) -> bool
        {
            if (position_only)
                return err.norm() < POS_TOL;
            return err.head<3>().norm() < POS_TOL && err.tail<3>().norm() < ANG_TOL;
        };

        VectorXd pose_cur = VectorXd::Zero(6);
        arm_.Joint2Cartesian(cur, pose_cur);
        VectorXd e = errVec(pose_cur);
        double err_norm = e.norm();
        double lambda = 1e-3;

        for (int iter = 0; iter < max_iter; iter++)
        {
            if (!rclcpp::ok() || !running_) return false;
            if (converged(e))
            {
                for (int i = 0; i < DoF; i++)
                    out[i] = cur[i];
                return true;
            }

            MatrixXd J(e.size(), num_free_joints);
            for (int j = 0; j < num_free_joints; j++)
            {
                double qh[DoF];
                for (int k = 0; k < DoF; k++)
                    qh[k] = cur[k];
                qh[j] += H;
                VectorXd pose_h = VectorXd::Zero(6);
                arm_.Joint2Cartesian(qh, pose_h);
                J.col(j) = (errVec(pose_h) - e) / H;
            }

            MatrixXd A = J.transpose() * J + lambda * MatrixXd::Identity(num_free_joints, num_free_joints);
            VectorXd b = -J.transpose() * e;
            VectorXd dq = A.ldlt().solve(b);
            for (int j = 0; j < num_free_joints; j++)
                dq(j) = std::max(-MAX_STEP, std::min(MAX_STEP, dq(j)));

            double cand[DoF];
            for (int j = 0; j < DoF; j++)
                cand[j] = cur[j];
            for (int j = 0; j < num_free_joints; j++)
                cand[j] = cur[j] + dq(j);

            VectorXd pose_cand = VectorXd::Zero(6);
            arm_.Joint2Cartesian(cand, pose_cand);
            VectorXd e_cand = errVec(pose_cand);
            double cand_norm = e_cand.norm();

            if (cand_norm < err_norm)
            {
                for (int j = 0; j < num_free_joints; j++)
                    cur[j] = cand[j];
                e = e_cand;
                err_norm = cand_norm;
                lambda = std::max(lambda * 0.7, 1e-7);
            }
            else
            {
                lambda = std::min(lambda * 3.0, 1e4);
            }
        }

        for (int i = 0; i < DoF; i++)
            out[i] = cur[i];
        return false;
    }

    // Check command and measured XY before each joint command.
    bool safetyCheck(const double *target)
    {
        double actual[DoF];
        if (!getFreshCurrentJoints(actual))
            return false;

        if (origin_xy_valid_)
        {
            VectorXd target_pose = VectorXd::Zero(6);
            VectorXd actual_pose = VectorXd::Zero(6);
            arm_.Joint2Cartesian(const_cast<double *>(target), target_pose);
            arm_.Joint2Cartesian(actual, actual_pose);
            const double target_xy_error = std::hypot(target_pose(0) - origin_x_, target_pose(1) - origin_y_);
            const double actual_xy_error = std::hypot(actual_pose(0) - origin_x_, actual_pose(1) - origin_y_);
            if (target_xy_error > 0.0005 || actual_xy_error > max_actual_xy_error_m_)
            {
                RCLCPP_ERROR(get_logger(),
                             "XY limit exceeded: target %.2f mm (limit 0.5), feedback %.2f mm (limit %.2f).",
                             target_xy_error * 1000.0, actual_xy_error * 1000.0,
                             max_actual_xy_error_m_ * 1000.0);
                return false;
            }
        }

        double max_diff = 0.0;
        for (int i = 0; i < DoF; i++)
        {
            max_diff = std::max(max_diff, std::fabs(target[i] - actual[i]));
        }

        constexpr double ABORT_THRESHOLD = 15.0 * DEG2RAD;
        if (max_diff > ABORT_THRESHOLD)
        {
            RCLCPP_ERROR(get_logger(),
                         "SAFETY ABORT: commanded target diverged from actual position by %.2f deg (limit 15 deg).",
                         max_diff * RAD2DEG);
            return false;
        }
        return true;
    }

    bool dwellAtWaypoint(const double *target, const VerticalScanPoint &point, bool require_arrival = false)
    {
        if (!require_arrival && point.dwell_seconds <= 0.0) return true;
        VectorXd goal = VectorXd::Zero(6);
        arm_.Joint2Cartesian(const_cast<double *>(target), goal);
        goal.head<3>() = point.position;
        if (!waitForExperimentArrival(*this, target, goal,
                [this](double *q) { return getFreshCurrentJoints(q); },
                [this](const double *q) { publishTargetJoints(q); },
                [this]() { logCsvRow(); }, [this]() { return running_.load(); },
                [this](const double *q) { return safetyCheck(q); })) return false;
        if (point.dwell_seconds <= 0.0) return true;
        RCLCPP_INFO(get_logger(), "Holding Z=%.4f m for %.2f seconds; CSV logging continues.",
                    point.position.z(), point.dwell_seconds);
        const auto started = std::chrono::steady_clock::now();
        rclcpp::Rate rate(control_rate_);
        while (rclcpp::ok() && running_ &&
               std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() < point.dwell_seconds)
        {
            if (!safetyCheck(target)) return false;
            publishTargetJoints(target);
            logCsvRow();
            rate.sleep();
        }
        return rclcpp::ok() && running_;
    }

    bool moveCalibrationZ(const VectorXd &base_pose, double delta_mm, StartupJoints &hold)
    {
        double current[DoF];
        if (!getFreshCurrentJoints(current)) return false;
        const bool hold_j6 = get_parameter("start_setup_mode").as_bool();
        const double initial_j6 = current[5];
        VectorXd start = VectorXd::Zero(6);
        arm_.Joint2Cartesian(current, start);
        const double end_z = start.z() + delta_mm / 1000.0;
        if (std::abs(end_z - base_pose.z()) > 0.300001)
        {
            RCLCPP_WARN(get_logger(), "Calibration adjustment exceeds +/-300 mm from base; no move.");
            return true;
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(delta_mm))));
        std::vector<StartupJoints> joints(1);
        std::copy_n(current, DoF, joints[0].begin());
        // Check the complete adjustment before moving. Every waypoint keeps base X/Y and orientation.
        for (int i = 1; i <= steps; ++i)
        {
            VectorXd target = base_pose;
            target.z() = start.z() + (end_z - start.z()) * i / steps;
            StartupJoints next{};
            if (!solveIKNumeric(target, joints.back().data(), next.data(), 200, hold_j6 ? 5 : DoF))
            {
                RCLCPP_WARN(get_logger(), "Calibration IK failed; no move. Choose another adjustment.");
                return true;
            }
            VectorXd achieved = VectorXd::Zero(6);
            arm_.Joint2Cartesian(next.data(), achieved);
            if ((achieved.head<3>() - target.head<3>()).norm() > 0.0005 ||
                std::any_of(next.begin(), next.end(), [](double q) { return !std::isfinite(q) || std::abs(q) > 2 * PI; }))
            {
                RCLCPP_WARN(get_logger(), "Calibration waypoint rejected; no move.");
                return true;
            }
            joints.push_back(next);
        }
        const double speed = get_parameter("calibration_speed_mm_s").as_double();
        const double joint_speed = get_parameter("max_joint_velocity_rad_s").as_double();
        if (!std::isfinite(speed) || speed <= 0 || !std::isfinite(joint_speed) || joint_speed <= 0)
        {
            RCLCPP_ERROR(get_logger(), "Calibration speeds must be positive.");
            return false;
        }
        rclcpp::Rate rate(control_rate_);
        auto checked_adjustment = [&](const double *command) {
            if (!safetyCheck(command)) return false;
            if (hold_j6)
            {
                double measured[DoF];
                if (!getFreshCurrentJoints(measured)) return false;
                if (std::abs(command[5] - initial_j6) > 1e-12 ||
                    std::abs(measured[5] - initial_j6) > 0.5 * DEG2RAD)
                {
                    RCLCPP_ERROR(get_logger(), "Held J6 moved beyond tolerance. Z adjustment stopped.");
                    return false;
                }
            }
            return true;
        };
        for (size_t i = 1; i < joints.size(); ++i)
        {
            int ticks = std::max(1, static_cast<int>(std::ceil(std::abs(delta_mm) / steps / speed * control_rate_)));
            for (int j = 0; j < DoF; ++j)
                ticks = std::max(ticks, static_cast<int>(std::ceil(std::abs(joints[i][j] - joints[i-1][j]) / joint_speed * control_rate_)));
            for (int tick = 1; tick <= ticks; ++tick)
            {
                StartupJoints command{};
                for (int j = 0; j < DoF; ++j)
                    command[j] = joints[i-1][j] + (joints[i][j] - joints[i-1][j]) * tick / ticks;
                if (!rclcpp::ok() || !running_ || !checked_adjustment(command.data())) return false;
                publishTargetJoints(command.data());
                rate.sleep();
            }
        }
        hold = joints.back();
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (rclcpp::ok() && running_ && std::chrono::steady_clock::now() < deadline)
        {
            if (!checked_adjustment(hold.data()) || !getFreshCurrentJoints(current)) return false;
            publishTargetJoints(hold.data());
            VectorXd measured = VectorXd::Zero(6);
            arm_.Joint2Cartesian(current, measured);
            if (std::abs(measured.z() - end_z) <= 0.0005 &&
                (!hold_j6 || std::abs(current[5] - initial_j6) <= 0.1 * DEG2RAD))
            {
                RCLCPP_INFO(get_logger(), "Calibration Z adjustment reached. Offset from base: %+.3f mm", (measured.z() - base_pose.z()) * 1000.0);
                return true;
            }
            rate.sleep();
        }
        RCLCPP_ERROR(get_logger(), "Calibration move did not settle; stopping without saving.");
        return false;
    }

    bool readCalibrationLine(int fd, const StartupJoints &hold, std::string &buffer, std::string &line)
    {
        // Keep publishing the stationary target while waiting for operator input.
        rclcpp::Rate rate(control_rate_);
        while (rclcpp::ok() && running_)
        {
            if (!safetyCheck(hold.data())) return false;
            publishTargetJoints(hold.data());
            const auto newline = buffer.find('\n');
            if (newline != std::string::npos)
            {
                line = buffer.substr(0, newline);
                buffer.erase(0, newline + 1);
                return true;
            }
            pollfd input{fd, POLLIN, 0};
            const int ready = poll(&input, 1, 0);
            if (ready < 0 && errno != EINTR) return false;
            if (ready > 0 && (input.revents & POLLIN))
            {
                char bytes[256];
                const ssize_t count = read(fd, bytes, sizeof(bytes));
                if (count <= 0) return false;
                buffer.append(bytes, static_cast<size_t>(count));
                if (buffer.size() > 4096) return false;
            }
            if (ready > 0 && (input.revents & (POLLHUP | POLLERR | POLLNVAL))) return false;
            rate.sleep();
        }
        return false;
    }

    bool alignExperimentStart(StartupJoints &current, const VectorXd &start,
                              const Matrix3d &goal_rotation)
    {
        const double angular_speed = get_parameter("alignment_angular_speed_deg_s").as_double() * DEG2RAD;
        const double joint_speed = get_parameter("alignment_joint_speed_rad_s").as_double();
        const double actual_limit = get_parameter("alignment_max_position_error_mm").as_double() / 1000.0;
        if (!std::isfinite(control_rate_) || control_rate_ <= 0.0 ||
            !std::isfinite(angular_speed) || angular_speed <= 0.0 ||
            !std::isfinite(joint_speed) || joint_speed <= 0.0 ||
            !std::isfinite(actual_limit) || actual_limit <= 0.0)
        {
            RCLCPP_ERROR(get_logger(), "Invalid alignment speed or position limit. Start pose was not saved.");
            return false;
        }
        const Matrix3d start_rotation = rotationFromRpy(start.tail<3>());
        const auto rotations = makeAlignmentRotations(start_rotation, goal_rotation, 0.25 * DEG2RAD);
        std::vector<StartupJoints> planned{current};
        for (size_t i = 1; i < rotations.size(); ++i)
        {
            VectorXd target = start; // XYZ remains at the captured point throughout alignment.
            Quaterniond quaternion = rotations[i];
            Vector3d rpy;
            arm_.Qua2Euler(quaternion, rpy);
            target.tail<3>() = rpy;
            StartupJoints next{};
            if (!solveIKNumeric(target, planned.back().data(), next.data(), 200, 5, false, true))
            {
                RCLCPP_ERROR(get_logger(), "Alignment IK failed at point %zu. No alignment motion started; start pose was not saved.", i);
                return false;
            }
            VectorXd achieved = VectorXd::Zero(6);
            arm_.Joint2Cartesian(next.data(), achieved);
            const double axis_error = toolZAlignmentError(rotationFromRpy(target.tail<3>()),
                                                         rotationFromRpy(achieved.tail<3>())).norm();
            if ((target.head<3>() - achieved.head<3>()).norm() > 0.0001 || axis_error > 0.001 ||
                std::abs(next[5] - planned.front()[5]) > 1e-12)
            {
                RCLCPP_ERROR(get_logger(), "Alignment pose error exceeded tolerance. No alignment motion started.");
                return false;
            }
            for (int j = 0; j < DoF; ++j)
                if (!std::isfinite(next[j]) || std::abs(next[j]) > 2 * PI ||
                    std::abs(next[j] - planned.back()[j]) > 0.05)
                {
                    RCLCPP_ERROR(get_logger(), "Alignment joint range/continuity check failed. No alignment motion started.");
                    return false;
                }
            planned.push_back(next);
        }
        auto checked_publish = [&](const StartupJoints &command) {
            if (!rclcpp::ok() || !running_ || !safetyCheck(command.data()) ||
                !getFreshCurrentJoints(current.data())) return false;
            VectorXd commanded = VectorXd::Zero(6), actual = VectorXd::Zero(6);
            arm_.Joint2Cartesian(const_cast<double *>(command.data()), commanded);
            arm_.Joint2Cartesian(current.data(), actual);
            if (std::abs(command[5] - planned.front()[5]) > 1e-12 ||
                std::abs(current[5] - planned.front()[5]) > 0.5 * DEG2RAD)
            {
                RCLCPP_ERROR(get_logger(), "Held J6 moved beyond tolerance. Alignment stopped; start pose was not saved.");
                return false;
            }
            if ((commanded.head<3>() - start.head<3>()).norm() > 0.0005 ||
                (actual.head<3>() - start.head<3>()).norm() > actual_limit)
            {
                RCLCPP_ERROR(get_logger(), "TCP XYZ moved beyond the alignment limit. Alignment stopped; start pose was not saved.");
                return false;
            }
            publishTargetJoints(command.data());
            return true;
        };
        // Reject changes to the arm made while IK was being planned.
        if (!getFreshCurrentJoints(current.data())) return false;
        for (int j = 0; j < DoF; ++j)
            if (std::abs(current[j] - planned.front()[j]) > 0.2 * DEG2RAD)
            {
                RCLCPP_ERROR(get_logger(), "Robot moved during alignment planning. Run mode 1 again.");
                return false;
            }
        RCLCPP_INFO(get_logger(), "Alignment path checked. Holding TCP XYZ and J6; aligning tool Z only...");
        rclcpp::Rate rate(control_rate_);
        for (size_t i = 1; i < planned.size(); ++i)
        {
            const double angle = rotations[i-1].angularDistance(rotations[i]);
            int ticks = std::max(1, static_cast<int>(std::ceil(angle / angular_speed * control_rate_)));
            for (int j = 0; j < DoF; ++j)
                ticks = std::max(ticks, static_cast<int>(std::ceil(
                    std::abs(planned[i][j] - planned[i-1][j]) / joint_speed * control_rate_)));
            for (int tick = 1; tick <= ticks; ++tick)
            {
                StartupJoints command{};
                for (int j = 0; j < DoF; ++j)
                    command[j] = planned[i-1][j] + (planned[i][j] - planned[i-1][j]) * tick / ticks;
                if (!checked_publish(command)) return false;
                rate.sleep();
            }
        }
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        auto settled_since = deadline;
        while (rclcpp::ok() && running_ && std::chrono::steady_clock::now() < deadline)
        {
            if (!checked_publish(planned.back()) || !getFreshCurrentJoints(current.data())) return false;
            VectorXd actual = VectorXd::Zero(6);
            arm_.Joint2Cartesian(current.data(), actual);
            const double position_error = (actual.head<3>() - start.head<3>()).norm();
            const double angle_error = toolZAlignmentError(goal_rotation, rotationFromRpy(actual.tail<3>())).norm();
            const auto now = std::chrono::steady_clock::now();
            if (position_error <= 0.0001 && angle_error <= 0.1 * DEG2RAD &&
                std::abs(current[5] - planned.front()[5]) <= 0.1 * DEG2RAD)
            {
                if (settled_since == deadline) settled_since = now;
                if (now - settled_since >= 250ms)
                {
                    RCLCPP_INFO(get_logger(), "Alignment reached: XYZ error %.3f mm, tool-Z tilt error %.3f deg; J6 held.",
                        position_error * 1000.0, angle_error * RAD2DEG);
                    return true;
                }
            }
            else settled_since = deadline;
            rate.sleep();
        }
        RCLCPP_ERROR(get_logger(), "Alignment did not settle. Start pose was not saved.");
        return false;
    }

    bool alignCurrentStartPose(StartupJoints &joints, VectorXd &pose)
    {
        if (!getFreshCurrentJoints(joints.data())) return false;
        pose = VectorXd::Zero(6);
        arm_.Joint2Cartesian(joints.data(), pose);
        try
        {
            const auto reference_rpy = get_parameter("alignment_world_from_base_rpy_deg").as_double_array();
            if (reference_rpy.size() != 3)
                throw std::invalid_argument("alignment_world_from_base_rpy_deg must have three values");
            const Matrix3d world_from_base = rotationFromRpy(
                Vector3d(reference_rpy[0], reference_rpy[1], reference_rpy[2]) * DEG2RAD);
            const Matrix3d goal = nearestToolZAlignedRotation(rotationFromRpy(pose.tail<3>()), world_from_base);
            const Vector3d goal_axis = world_from_base * goal.col(2);
            RCLCPP_INFO(get_logger(), "Start setup: keep XYZ=(%.4f, %.4f, %.4f) m and J6=%.2f deg; align tool Z to world (%.0f, %.0f, %.0f).",
                pose.x(), pose.y(), pose.z(), joints[5] * RAD2DEG,
                goal_axis.x(), goal_axis.y(), goal_axis.z());
            if (!alignExperimentStart(joints, pose, goal)) return false;
            if (!getFreshCurrentJoints(joints.data())) return false;
            arm_.Joint2Cartesian(joints.data(), pose);
            RCLCPP_INFO(get_logger(), "Aligned TCP RPY in controller base: roll=%.2f pitch=%.2f yaw=%.2f deg; J6 held.",
                pose[3] * RAD2DEG, pose[4] * RAD2DEG, pose[5] * RAD2DEG);
            return true;
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(get_logger(), "Cannot align current pose: %s", error.what());
            return false;
        }
    }

    bool captureExperimentStart(double start_distance_mm)
    {
        try
        {
            if (!std::isfinite(start_distance_mm) || start_distance_mm <= 0.0)
                throw std::invalid_argument("Start distance must be a positive number in mm");
            StartupJoints joints{};
            VectorXd pose = VectorXd::Zero(6);
            if (!alignCurrentStartPose(joints, pose)) return false;
            // Save measured feedback after the final alignment.
            const auto path = get_parameter("start_file").as_string();
            saveExperimentStart(path, robot_name_, pose, joints, true, start_distance_mm);
            RCLCPP_INFO(get_logger(), "Aligned experiment start saved: X=%.4f Y=%.4f Z=%.4f m (%s).",
                pose.x(), pose.y(), pose.z(), path.c_str());
            RCLCPP_INFO(get_logger(), "Saved start distance: %.2f mm. Modes 2-4 use this distance for motion and CSV.", start_distance_mm);
            return true;
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(get_logger(), "Cannot save experiment start: %s", error.what());
            return false;
        }
    }

    static std::string setupInput(std::string text)
    {
        const auto first = text.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        text = text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
        for (char &c : text) if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
        return text;
    }

    bool readSetupLine(int fd, std::string &buffer, std::string &line)
    {
        // No target commands while waiting: pendant/free-drive changes must be possible.
        while (rclcpp::ok() && running_)
        {
            const auto newline = buffer.find('\n');
            if (newline != std::string::npos)
            {
                line = setupInput(buffer.substr(0, newline));
                buffer.erase(0, newline + 1);
                return true;
            }
            pollfd input{fd, POLLIN, 0};
            const int ready = poll(&input, 1, 50);
            if (ready < 0 && errno != EINTR) return false;
            if (ready > 0 && (input.revents & POLLIN))
            {
                char bytes[256];
                const ssize_t count = read(fd, bytes, sizeof(bytes));
                if (count <= 0) return false;
                buffer.append(bytes, static_cast<size_t>(count));
                if (buffer.size() > 4096) return false;
            }
            if (ready > 0 && (input.revents & (POLLHUP | POLLERR | POLLNVAL))) return false;
        }
        return false;
    }

    void runStartSetup()
    {
        const int fd = open("/dev/tty", O_RDWR | O_NONBLOCK);
        if (fd < 0)
        {
            RCLCPP_ERROR(get_logger(), "Start setup requires an interactive terminal. Use mode:=capture start_distance_mm:=200 for automated saving.");
            return;
        }
        struct CloseTerminal { int fd; ~CloseTerminal() { close(fd); } } terminal{fd};
        std::string buffer, line;
        while (rclcpp::ok() && running_)
        {
            std::cout << "\nUse current position as start? (y/n, q to cancel):\n" << std::flush;
            if (!readSetupLine(fd, buffer, line)) return;
            if (line == "q" || line == "cancel") return;
            if (line == "y" || line == "yes")
            {
                double distance = 0.0;
                while (rclcpp::ok() && running_)
                {
                    std::cout << "\nStart distance (mm):\n" << std::flush;
                    if (!readSetupLine(fd, buffer, line)) return;
                    if (line == "q" || line == "cancel") return;
                    try
                    {
                        distance = parseExperimentStartDistanceMm(line);
                        break;
                    }
                    catch (const std::invalid_argument &error)
                    {
                        std::cout << error.what() << '\n' << std::flush;
                    }
                }
                if (distance > 0 && captureExperimentStart(distance)) return;
                continue;
            }
            if (line != "n" && line != "no")
            {
                std::cout << "Enter y or n.\n";
                continue;
            }
            std::cout << "\nAdjust position with the pendant in UR base Z, or enter +10 / -1 (mm).\n"
                         "Type a to align here, e to return to the start question, or q to cancel setup.\n" << std::flush;
            while (rclcpp::ok() && running_)
            {
                std::cout << "\nZ adjustment:\n" << std::flush;
                if (!readSetupLine(fd, buffer, line)) return;
                if (line == "e") break;
                if (line == "q" || line == "cancel") return;
                if (line == "a" || line == "align")
                {
                    StartupJoints aligned_joints{};
                    VectorXd aligned_pose = VectorXd::Zero(6);
                    if (alignCurrentStartPose(aligned_joints, aligned_pose))
                        std::cout << "\nAligned. Continue adjusting; start pose has not been saved.\n" << std::flush;
                    else
                        std::cout << "\nAlignment failed. Adjust the pose and try a again.\n" << std::flush;
                    continue;
                }
                try
                {
                    const auto command = parseCalibrationCommand(line);
                    if (command.action != CalibrationCommand::Move)
                    {
                        std::cout << "Enter a Z adjustment, a to align, or e to return.\n";
                        continue;
                    }
                    StartupJoints current{};
                    if (!getFreshCurrentJoints(current.data())) return;
                    VectorXd current_pose = VectorXd::Zero(6);
                    arm_.Joint2Cartesian(current.data(), current_pose);
                    // Refresh the anchor after any pendant move; hold the current XY/RPY.
                    origin_x_ = current_pose.x(); origin_y_ = current_pose.y(); origin_xy_valid_ = true;
                    if (!moveCalibrationZ(current_pose, command.delta_mm, current)) return;
                    origin_xy_valid_ = false;
                }
                catch (const std::exception &error)
                {
                    origin_xy_valid_ = false;
                    std::cout << "Invalid adjustment: " << error.what() << '\n' << std::flush;
                }
            }
        }
    }

    void runCalibration()
    {
        const int fd = open("/dev/tty", O_RDWR | O_NONBLOCK);
        if (fd < 0)
        {
            RCLCPP_ERROR(get_logger(), "Calibration requires an interactive terminal; no move.");
            return;
        }
        struct CloseTerminal { int fd; ~CloseTerminal() { close(fd); } } terminal{fd};
        double current[DoF];
        if (!runJointStartup(*this, current,
                [this](double *q) { return getFreshCurrentJoints(q); },
                [this](const double *q) { publishTargetJoints(q); }, []() {},
                [this]() { return running_.load(); },
                [this](const double *q) { return safetyCheck(q); })) return;
        VectorXd base_pose = VectorXd::Zero(6);
        arm_.Joint2Cartesian(current, base_pose);
        origin_x_ = base_pose.x(); origin_y_ = base_pose.y(); origin_xy_valid_ = true;
        StartupJoints hold{};
        std::copy_n(current, DoF, hold.begin());
        RCLCPP_INFO(get_logger(), "Calibration ready. X/Y and orientation are held; Z is adjusted in whole millimeters.");
        std::string buffer, line;
        while (rclcpp::ok() && running_)
        {
            std::cout << "\nAdjust Z: +10 / -1 (mm) | done to save | cancel to exit\nZ adjustment: " << std::flush;
            if (!readCalibrationLine(fd, hold, buffer, line)) return;
            try
            {
                const auto command = parseCalibrationCommand(line);
                if (command.action == CalibrationCommand::Cancel)
                {
                    RCLCPP_INFO(get_logger(), "Calibration cancelled; existing sensor zero file is unchanged.");
                    return;
                }
                if (command.action == CalibrationCommand::Help) continue;
                if (command.action == CalibrationCommand::Move)
                {
                    if (!moveCalibrationZ(base_pose, command.delta_mm, hold)) return;
                    continue;
                }
                if (!getFreshCurrentJoints(current)) return;
                VectorXd zero_pose = VectorXd::Zero(6);
                arm_.Joint2Cartesian(current, zero_pose);
                StartupJoints zero_joints{};
                std::copy_n(current, DoF, zero_joints.begin());
                const std::string path = get_parameter("calibration_file").as_string();
                saveSensorZero(path, robot_name_, base_pose, zero_pose, zero_joints);
                RCLCPP_INFO(get_logger(), "Sensor zero saved: %s (base offset %+.3f mm).", path.c_str(), (zero_pose.z() - base_pose.z()) * 1000.0);
                return;
            }
            catch (const std::exception &error)
            {
                RCLCPP_WARN(get_logger(), "Calibration input/save error: %s", error.what());
            }
        }
    }

    void runVerticalScan()
    {
        if (get_parameter("base_mode").as_bool())
        {
            // Move to the common initial pose with J1/J6 held; no sensors or scan.
            double current[DoF];
            runJointStartup(*this, current,
                [this](double *q) { return getFreshCurrentJoints(q); },
                [this](const double *q) { publishTargetJoints(q); },
                []() {},
                [this]() { return running_.load(); },
                [this](const double *q) { return safetyCheck(q); });
            return;
        }
        const double scan_speed_m_s = get_parameter("scan_speed_mm_s").as_double() / 1000.0;
        const double max_joint_vel = get_parameter("max_joint_velocity_rad_s").as_double();
        const double max_actual_xy_error_mm = get_parameter("max_actual_xy_error_mm").as_double();
        if (!std::isfinite(control_rate_) || control_rate_ <= 0.0 ||
            !std::isfinite(scan_speed_m_s) || scan_speed_m_s <= 0.0 ||
            !std::isfinite(max_joint_vel) || max_joint_vel <= 0.0 ||
            !std::isfinite(max_actual_xy_error_mm) || max_actual_xy_error_mm <= 0.0)
        {
            RCLCPP_ERROR(get_logger(), "Control rate, scan speed, joint velocity and XY error limit must be positive.");
            return;
        }
        max_actual_xy_error_m_ = max_actual_xy_error_mm / 1000.0;
        const double control_dt = 1.0 / control_rate_;

        if (enable_ecan_)
        {
            waitForEcanReady();
            if (!rclcpp::ok() || !running_) return;
            can_logging_enabled_ = true;
            if (!experiment_csv_enabled_ && !startCsvLogging()) return;
        }

        // Menu scans restore the saved experiment start; direct/legacy modes use
        // their configured startup reference.
        double current[DoF];
        if (!runJointStartup(*this, current,
                [this](double *q) { return getFreshCurrentJoints(q); },
                [this](const double *q) { publishTargetJoints(q); },
                [this]() { logCsvRow(); },
                [this]() { return running_.load(); },
                [this](const double *q) { return safetyCheck(q); }))
            return;
        if (get_parameter("startup_only").as_bool())
            return;
        VectorXd origin_pose = VectorXd::Zero(6);
        arm_.Joint2Cartesian(current, origin_pose);
        origin_pose = scanOriginPose(*this, origin_pose);
        Vector3d origin = origin_pose.head<3>();
        origin_x_ = origin.x();
        origin_y_ = origin.y();
        origin_xy_valid_ = true;
        RCLCPP_INFO(get_logger(), "Scan origin: X=%.4f Y=%.4f Z=%.4f m. Orientation is held fixed.",
                    origin.x(), origin.y(), origin.z());

        const std::vector<VerticalScanPoint> path = generateVerticalTrajectory(origin);
        if (path.size() < 2)
        {
            RCLCPP_ERROR(get_logger(), "No vertical scan trajectory; aborting.");
            return;
        }

        // Solve and check the entire path before sending the first command.
        std::vector<std::array<double, DoF>> planned_joints;
        planned_joints.reserve(path.size());
        std::array<double, DoF> seed{};
        for (int j = 0; j < DoF; ++j)
            seed[j] = current[j];
        planned_joints.push_back(seed);
        for (size_t p = 1; p < path.size(); ++p)
        {
            if (!rclcpp::ok() || !running_) return;
            VectorXd target_pose = origin_pose;
            target_pose.head<3>() = path[p].position;
            std::array<double, DoF> solution{};
            if (!solveIKNumeric(target_pose, seed.data(), solution.data()))
            {
                if (!rclcpp::ok() || !running_) return;
                RCLCPP_ERROR(get_logger(), "No IK solution at vertical point %zu. Scan motion has not started.", p);
                return;
            }
            VectorXd achieved = VectorXd::Zero(6);
            arm_.Joint2Cartesian(solution.data(), achieved);
            if (std::hypot(achieved(0) - origin_x_, achieved(1) - origin_y_) > 0.0005 ||
                std::abs(achieved(2) - path[p].position.z()) > 0.0005)
            {
                RCLCPP_ERROR(get_logger(), "IK pose error at point %zu exceeds 0.5 mm. Scan motion has not started.", p);
                return;
            }
            planned_joints.push_back(solution);
            seed = solution;
        }
        RCLCPP_INFO(get_logger(), "Vertical path preflight passed; starting %zu points.", path.size());
        if (!running_ || !rclcpp::ok() || !getFreshCurrentJoints(current))
            return;
        if (experiment_csv_enabled_)
        {
            csv_reference_z_m_ = origin.z();
            csv_travel_mm_ = get_parameter("z_max_offset_mm").as_double();
            if (!startCsvLogging()) return;
        }
        logCsvRow();

        rclcpp::Rate rate(control_rate_);
        if (!dwellAtWaypoint(planned_joints.front().data(), path.front(), true))
            return;
        for (size_t p = 1; p < path.size() && rclcpp::ok() && running_; ++p)
        {
            const auto &from = planned_joints[p - 1];
            const auto &to = planned_joints[p];
            const double distance = (path[p].position - path[p - 1].position).norm();
            int n_sub = std::max(1, static_cast<int>(std::ceil(distance / (scan_speed_m_s * control_dt))));
            double max_joint_step = 0.0;
            for (int j = 0; j < DoF; ++j)
                max_joint_step = std::max(max_joint_step, std::abs(to[j] - from[j]));
            n_sub = std::max(n_sub, static_cast<int>(std::ceil(max_joint_step / (max_joint_vel * control_dt))));

            for (int k = 1; k <= n_sub && rclcpp::ok() && running_; ++k)
            {
                const double alpha = static_cast<double>(k) / n_sub;
                std::array<double, DoF> command{};
                for (int j = 0; j < DoF; ++j)
                    command[j] = from[j] + alpha * (to[j] - from[j]);
                if (!safetyCheck(command.data()))
                {
                    RCLCPP_ERROR(get_logger(), "Vertical scan aborted at point %zu.", p);
                    return;
                }
                publishTargetJoints(command.data());
                logCsvRow();
                rate.sleep();
            }
            const bool boundary = p + 1 == path.size() || experimentBoundary(
                path[p - 1].position, path[p].position, path[p + 1].position);
            if (!dwellAtWaypoint(to.data(), path[p], boundary)) return;
        }

        if (rclcpp::ok() && running_)
        {
            RCLCPP_INFO(get_logger(), "Vertical scan completed.");
        }
    }

    void controlLoop()
    {
        const bool experiment = get_parameter("experiment_start_enabled").as_bool();
        ExperimentCancelInput keyboard(experiment, [this]() { running_ = false; });
        if (experiment)
        {
            if (keyboard.available())
                RCLCPP_INFO(get_logger(), "Press q to cancel this experiment and return to the saved start (no Enter needed).");
            else
                RCLCPP_WARN(get_logger(), "No interactive terminal; keyboard q cancellation is unavailable.");
        }
        while (rclcpp::ok() && running_ && !joint_state_received_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for joint feedback on %s. Start the driver in another terminal: ros2 run urcb2_driver single_arm",
                joint_state_sub_->get_topic_name());
            std::this_thread::sleep_for(100ms);
        }
        if (!joint_state_received_ && running_ && rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Failed to receive joint states. Is urcb2_driver running?");
        }
        if (joint_state_received_ && running_ && rclcpp::ok())
        {
            RCLCPP_INFO(get_logger(), "Joint states received. Waiting for stabilization...");
            for (int i = 0; i < 10 && running_ && rclcpp::ok(); ++i)
                std::this_thread::sleep_for(50ms);
            if (running_ && rclcpp::ok())
            {
                if (get_parameter("start_setup_mode").as_bool()) runStartSetup();
                else if (get_parameter("capture_start_mode").as_bool()) captureExperimentStart(get_parameter("start_distance_mm").as_double());
                else if (get_parameter("calibration_mode").as_bool()) runCalibration();
                else runVerticalScan();
            }
        }
        keyboard.stop();
        stopCsvLogging();
        if (keyboard.requested())
        {
            // The startup joint path may change XY, just as the initial restore
            // before a scan does. Keep joint tracking/freshness checks enabled.
            origin_xy_valid_ = false;
            returnAfterExperimentCancel(*this, running_,
                [this](double *q) { return getFreshCurrentJoints(q); },
                [this](const double *q) { publishTargetJoints(q); },
                [this](const double *q) { return safetyCheck(q); });
        }
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    // Parameters
    std::string robot_name_;
    bool enable_ecan_ = false;
    bool experiment_csv_enabled_ = false;
    int csv_sensor_can_id_ = 0x41;
    double sensor_stale_timeout_sec_ = 0.5;
    std::string csv_sensor_channel_;
    double csv_reference_z_m_ = 0.0;
    double csv_travel_mm_ = 0.0;
    uint64_t feedback_sequence_ = 0;
    std::chrono::steady_clock::time_point csv_first_sample_time_{};
    std::chrono::steady_clock::time_point last_csv_flush_time_{};
    std::string csv_save_dir_;
    std::string can_server_ip_;
    int can_server_port_ = 4001;
    std::vector<uint32_t> tracked_can_ids_;
    double control_rate_;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double max_actual_xy_error_m_ = 0.005;
    bool origin_xy_valid_ = false;

    // State
    double current_joint_pos_[DoF];
    double current_joint_vel_[DoF];
    std::mutex state_mutex_;
    std::atomic<bool> joint_state_received_{false};
    std::chrono::steady_clock::time_point last_feedback_time_{};
    std::atomic<bool> running_{false};
    std::atomic<bool> can_logging_enabled_{false};
    std::atomic<bool> ecan_client_connected_{false};
    std::atomic<bool> ecan_running_{true};
    std::thread ecan_thread_;
    std::thread control_thread_;
    mutable std::mutex can_mutex_;
    std::vector<TrackedCanFrame> tracked_can_frames_;

    ArmControl arm_;
    ArmControl csv_arm_;
    std::mutex csv_mutex_;
    rclcpp::TimerBase::SharedPtr csv_timer_;
    std::ofstream csv_file_;
    std::string csv_file_path_;
    std::chrono::steady_clock::time_point csv_start_time_{};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr can_status_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProxskinVerticalTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
