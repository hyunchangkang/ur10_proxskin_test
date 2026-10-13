/**
 * @file proxskin_test.cpp
 * @brief UR-10 YZ raster scan node, ported from sung-jin123/UR-10-PROXSKIN-TEST
 *        (originally a ROS1 + Pinocchio + ur_modern_driver/FollowJointTrajectory node).
 *
 * This ROS2 port talks directly to urcb2_driver instead of ur_modern_driver:
 *   Subscribe: {robot_name}/joint_states (sensor_msgs/JointState)
 *   Publish:   {robot_name}/targetJ      (std_msgs/Float64MultiArray)
 * urcb2_driver streams targetJ to the robot via servoj at 125Hz, so there is no
 * FollowJointTrajectory action server here -- motion is streamed directly.
 *
 * This node also connects to the eCAN TCP server using the Prox_sensor_React protocol and directly tracks
 * configured CAN IDs while the raster scan is running.
 *
 * Run:
 *   ros2 run urcb2_driver single_arm
 *   ros2 run ur10_proxskin_test proxskin_test
 */

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ur10_proxskin_test/arm_function.h"
#include "ur10_proxskin_test/joint_startup.hpp"
#include "ur10_proxskin_test/experiment_trajectory.hpp"
#include "ur10_proxskin_test/experiment_csv.hpp"
#include "ur10_proxskin_test/ecan_tcp_client.hpp"
#include "ur10_proxskin_test/experiment_arrival.hpp"
#include "ur10_proxskin_test/experiment_cancel.hpp"
#include "ur10_proxskin_test/experiment_return.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ProxskinTestNode : public rclcpp::Node
{
public:
    ProxskinTestNode() : Node("proxskin_test")
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
        // Scan area is relative to the measured TCP after the common joint initialization.
        // This avoids hardcoding absolute base-frame coordinates that were tuned for a
        // different robot's mounting (the original repo's X/Y/Z values do not apply here).
        declare_parameter<double>("x_offset_mm", 0.0);
        declare_parameter<double>("y_span_mm", 600.0);
        declare_parameter<std::string>("y_direction", "centered");
        declare_parameter<double>("x_span_mm", 600.0);
        declare_parameter<std::string>("x_direction", "centered");
        declare_parameter<double>("z_span_mm", 300.0);
        declare_parameter<double>("spacing_mm", 1.0);
        // Used by the interactive scan menu. The original centered raster path
        // remains available when use_relative_vertical_range is false.
        declare_parameter<bool>("use_relative_vertical_range", false);
        declare_parameter<double>("z_min_offset_mm", 10.0);
        declare_parameter<double>("z_max_offset_mm", 300.0);
        declare_parameter<int>("repeat_count", 1);
        declare_parameter<double>("scan_speed_mm_s", 50.0);
        declare_parameter<bool>("use_fixed_orientation", false);
        declare_parameter<double>("roll_deg", -179.0);
        declare_parameter<double>("pitch_deg", 0.01);
        declare_parameter<double>("yaw_deg", -89.0);
        // When true, wrist_1/2/3 (joints 4-6) are held at fixed angles instead of being
        // solved by IK; only shoulder_pan/shoulder_lift/elbow move to satisfy the target
        // XYZ position. Note the TCP orientation is then NOT held constant -- it drifts
        // with whatever shoulder/elbow configuration each scan point needs, since only 3
        // joints are left to satisfy 3 position constraints. use_fixed_orientation/roll/
        // pitch/yaw are ignored while this is enabled.
        declare_parameter<bool>("fix_wrist_joints", false);
        declare_parameter<double>("wrist1_deg", -90.0);
        declare_parameter<double>("wrist2_deg", -90.0);
        declare_parameter<double>("wrist3_deg", 0.0);
        declare_parameter<double>("approach_vel_rad_s", 0.1);
        declare_parameter<double>("max_joint_velocity_rad_s", 1.0);
        declareJointStartupParameters(*this);
        declare_parameter<int>("csv_sensor_can_id", 0x41);
        declare_parameter<std::string>("csv_sensor_channel", "front");

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
            std::bind(&ProxskinTestNode::jointStateCallback, this, _1));

        std::string target_topic = robot_name_ + "/targetJ";
        target_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(target_topic, 10);
        if (enable_ecan_)
        {
            can_status_timer_ = create_wall_timer(
                2s, std::bind(&ProxskinTestNode::logTrackedCanSummary, this));
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
            RCLCPP_WARN(get_logger(), "eCAN is disabled; experiment CSV records robot distance with an empty sensor column.");
        }

        if (experiment_csv_enabled_)
            csv_timer_ = create_wall_timer(10ms, [this]() { logExperimentCsvRow(); });
        running_ = true;
        if (enable_ecan_)
        {
            ecan_thread_ = std::thread(&ProxskinTestNode::ecanReceiveLoop, this);
        }
        control_thread_ = std::thread(&ProxskinTestNode::controlLoop, this);
    }

    ~ProxskinTestNode() override
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
        filename << "proxskin_" << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S") << ".csv";
        const auto csv_name = experiment_csv_enabled_ ? experimentCsvFilename("columns") : filename.str();
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
        std::copy_n(current_joint_pos_, DoF, out);
        return true;
    }

    // 'ㄹ' pattern raster centered on `origin` (the pose the robot was jogged to just
    // before scanning started): X fixed at origin.x + x_offset, Y/Z sweep +-span/2
    // around origin, alternating top<->bottom every column.
    std::vector<Vector3d> generateRasterTrajectory(const Vector3d &origin)
    {
        if (get_parameter("experiment_start_enabled").as_bool())
        {
            try
            {
                return makeDownwardColumnTrajectory(origin,
                    get_parameter("z_max_offset_mm").as_double() / 1000.0,
                    get_parameter("x_span_mm").as_double() / 1000.0,
                    get_parameter("spacing_mm").as_double() / 1000.0,
                    get_parameter("repeat_count").as_int(),
                    get_parameter("x_direction").as_string(), "x");
            }
            catch (const std::invalid_argument &error)
            {
                RCLCPP_ERROR(get_logger(), "%s", error.what());
                return {};
            }
        }
        double x_offset = get_parameter("x_offset_mm").as_double() / 1000.0;
        double y_span = get_parameter("y_span_mm").as_double() / 1000.0;
        double z_span = get_parameter("z_span_mm").as_double() / 1000.0;
        double dy = get_parameter("spacing_mm").as_double() / 1000.0;
        const bool relative_range = get_parameter("use_relative_vertical_range").as_bool();
        const double min_offset = get_parameter("z_min_offset_mm").as_double() / 1000.0;
        const double max_offset = get_parameter("z_max_offset_mm").as_double() / 1000.0;
        const int repeats = get_parameter("repeat_count").as_int();

        if (!std::isfinite(y_span) || !std::isfinite(dy) || y_span < 0.0 || dy <= 0.0 ||
            (relative_range && (!std::isfinite(min_offset) || !std::isfinite(max_offset) ||
                                min_offset <= 0.0 || max_offset <= min_offset || repeats < 1)))
        {
            RCLCPP_ERROR(get_logger(), "Invalid raster settings: require Y span >= 0, spacing > 0, and 0 < Z min < Z max with repetitions >= 1.");
            return {};
        }

        double X = origin(0) + x_offset;
        double y_start = origin(1) + y_span / 2.0;
        double y_end = origin(1) - y_span / 2.0;
        double z_top = relative_range ? origin(2) + max_offset : origin(2) + z_span / 2.0;
        double z_bot = relative_range ? origin(2) + min_offset : origin(2) - z_span / 2.0;
        int Nvert = std::max(2, static_cast<int>(std::round((z_top - z_bot) / dy)) + 1);

        std::vector<double> yCols;
        double dir = (y_end < y_start) ? -1.0 : 1.0;
        for (double y = y_start; dir * (y_end - y) > 0.0; y += dir * dy)
        {
            yCols.push_back(y);
        }
        if (yCols.empty() || fabs(yCols.back() - y_end) > 1e-9)
        {
            yCols.push_back(y_end);
        }

        std::vector<Vector3d> trajectory;
        bool dirDown = true;

        if (relative_range)
        {
            // Approach the first column from the common initial pose
            // along a sampled Cartesian path.
            trajectory.push_back(origin);
            const Vector3d first_top(X, y_start, z_top);
            const int steps = std::max(1, static_cast<int>(std::ceil((first_top - origin).norm() / dy)));
            for (int i = 1; i <= steps; ++i)
                trajectory.push_back(origin + (static_cast<double>(i) / steps) * (first_top - origin));
        }

        RCLCPP_INFO(get_logger(), "Generating raster trajectory: X=%.3f m, Y %.3f->%.3f m (%.2f mm spacing), Z %.3f<->%.3f m, %d columns",
                    X, y_start, y_end, dy * 1000.0, z_top, z_bot, static_cast<int>(yCols.size()));

        for (double yk : yCols)
        {
            if (relative_range)
            {
                // At each column, make complete top -> bottom -> top cycles.
                // The first point of the next column stays at the same Z.
                trajectory.emplace_back(X, yk, z_top);
                for (int cycle = 0; cycle < repeats; ++cycle)
                {
                    for (int leg = 0; leg < 2; ++leg)
                    {
                        const double start_z = leg == 0 ? z_top : z_bot;
                        const double end_z = leg == 0 ? z_bot : z_top;
                        for (int i = 1; i < Nvert; ++i)
                        {
                            const double t = static_cast<double>(i) / (Nvert - 1);
                            trajectory.emplace_back(X, yk, start_z + t * (end_z - start_z));
                        }
                    }
                }
            }
            else
            {
                for (int i = 0; i < Nvert; i++)
                {
                    double t = static_cast<double>(i) / (Nvert - 1);
                    double z = dirDown ? (z_top + t * (z_bot - z_top)) : (z_bot + t * (z_top - z_bot));
                    trajectory.emplace_back(X, yk, z);
                }
                dirDown = !dirDown;
            }
        }

        RCLCPP_INFO(get_logger(), "Raster trajectory generated: %zu points", trajectory.size());
        return trajectory;
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
    // `position_only` drops the orientation rows from the error/cost entirely.
    bool solveIKNumeric(const VectorXd &target_pose, const double *seed, double *out,
                        int max_iter = 200, int num_free_joints = DoF, bool position_only = false)
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

    // Stream from current joint targets to `goal` joints using a trapezoidal profile,
    // publishing at control_rate_. Aborts if actual feedback diverges too far (safety).
    bool streamTrapezoidalMove(const double *start, const double *goal, double vel_des)
    {
        MatrixXd traj, vel;
        arm_.JointTrajectoryTrapezoidal(const_cast<double *>(start), const_cast<double *>(goal), traj, vel, vel_des, 0.0, 0.0);

        rclcpp::Rate rate(control_rate_);
        for (int i = 0; i < traj.rows() && rclcpp::ok() && running_; i++)
        {
            double target[DoF];
            for (int j = 0; j < DoF; j++)
                target[j] = traj(i, j);

            if (!safetyCheck(target))
                return false;

            publishTargetJoints(target);
            logCsvRow();
            rate.sleep();
        }
        return true;
    }

    // Aborts the scan if the commanded target has drifted too far from the actual
    // robot position, mirroring the safety check used in ur_10_control_node.
    bool safetyCheck(const double *target)
    {
        double actual[DoF];
        if (!getFreshCurrentJoints(actual))
            return false;

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

    void runRasterScan()
    {
        bool use_fixed_orientation = get_parameter("use_fixed_orientation").as_bool();
        double roll = get_parameter("roll_deg").as_double() * DEG2RAD;
        double pitch = get_parameter("pitch_deg").as_double() * DEG2RAD;
        double yaw = get_parameter("yaw_deg").as_double() * DEG2RAD;
        bool fix_wrist_joints = get_parameter("fix_wrist_joints").as_bool();
        double wrist_fixed[3] = {
            get_parameter("wrist1_deg").as_double() * DEG2RAD,
            get_parameter("wrist2_deg").as_double() * DEG2RAD,
            get_parameter("wrist3_deg").as_double() * DEG2RAD,
        };
        int num_free_joints = fix_wrist_joints ? 3 : DoF;
        double approach_vel = get_parameter("approach_vel_rad_s").as_double();
        double max_joint_vel = get_parameter("max_joint_velocity_rad_s").as_double();
        double scan_speed_m_s = get_parameter("scan_speed_mm_s").as_double() / 1000.0;
        double control_dt = 1.0 / control_rate_;

        if (enable_ecan_)
        {
            waitForEcanReady();
            if (!rclcpp::ok() || !running_) return;
            can_logging_enabled_ = true;
            if (!experiment_csv_enabled_ && !startCsvLogging()) return;
        }

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

        VectorXd origin_pose_eul = VectorXd::Zero(6);
        arm_.Joint2Cartesian(current, origin_pose_eul);
        origin_pose_eul = scanOriginPose(*this, origin_pose_eul);
        Vector3d origin_pos = origin_pose_eul.head<3>();

        Vector3d fixed_eul;
        if (use_fixed_orientation)
        {
            fixed_eul << roll, pitch, yaw;
        }
        else
        {
            fixed_eul = origin_pose_eul.tail<3>();
        }
        RCLCPP_INFO(get_logger(), "Scan origin captured: x=%.4f y=%.4f z=%.4f (use_fixed_orientation=%s, fix_wrist_joints=%s)",
                    origin_pos(0), origin_pos(1), origin_pos(2),
                    use_fixed_orientation ? "true" : "false", fix_wrist_joints ? "true" : "false");
        if (fix_wrist_joints)
        {
            RCLCPP_INFO(get_logger(), "Wrist joints locked at (deg): wrist1=%.1f wrist2=%.1f wrist3=%.1f "
                                      "(TCP orientation will drift with shoulder/elbow, not held fixed)",
                        wrist_fixed[0] * RAD2DEG, wrist_fixed[1] * RAD2DEG, wrist_fixed[2] * RAD2DEG);
        }

        std::vector<Vector3d> cart_trajectory = generateRasterTrajectory(origin_pos);
        if (cart_trajectory.empty())
        {
            RCLCPP_ERROR(get_logger(), "Empty raster trajectory, aborting.");
            return;
        }

        // ----- Move to start position -----
        VectorXd start_pose = VectorXd::Zero(6);
        start_pose << cart_trajectory[0](0), cart_trajectory[0](1), cart_trajectory[0](2),
            fixed_eul(0), fixed_eul(1), fixed_eul(2);
        RCLCPP_INFO(get_logger(), "Target start pose (m): x=%.4f y=%.4f z=%.4f",
                    start_pose(0), start_pose(1), start_pose(2));

        // IK seed: shoulder/elbow from the actual current pose (good initial guess),
        // wrist forced to the locked values so the solved start_joint already carries
        // them -- the trapezoidal move below then also brings the real wrist there.
        double ik_seed[DoF];
        for (int i = 0; i < DoF; i++)
            ik_seed[i] = current[i];
        if (fix_wrist_joints)
        {
            ik_seed[3] = wrist_fixed[0];
            ik_seed[4] = wrist_fixed[1];
            ik_seed[5] = wrist_fixed[2];
        }

        double start_joint[DoF];
        bool ik_ok = solveIKNumeric(start_pose, ik_seed, start_joint, 500, num_free_joints, fix_wrist_joints);
        if (!rclcpp::ok() || !running_) return;

        VectorXd achieved_pose = VectorXd::Zero(6);
        arm_.Joint2Cartesian(start_joint, achieved_pose);
        double pos_err = (start_pose.head<3>() - achieved_pose.head<3>()).norm();
        RCLCPP_INFO(get_logger(), "IK joints (deg): %.1f %.1f %.1f %.1f %.1f %.1f",
                    start_joint[0] * RAD2DEG, start_joint[1] * RAD2DEG, start_joint[2] * RAD2DEG,
                    start_joint[3] * RAD2DEG, start_joint[4] * RAD2DEG, start_joint[5] * RAD2DEG);
        RCLCPP_INFO(get_logger(), "IK achieved pose (m): x=%.4f y=%.4f z=%.4f | position error=%.4f m",
                    achieved_pose(0), achieved_pose(1), achieved_pose(2), pos_err);

        if (!ik_ok)
        {
            RCLCPP_ERROR(get_logger(), "No IK solution exists for the start position. Aborting.");
            return;
        }

        if (!get_parameter("use_relative_vertical_range").as_bool())
        {
            RCLCPP_INFO(get_logger(), "Moving to raster scan start position...");
            if (!streamTrapezoidalMove(current, start_joint, approach_vel))
            {
                RCLCPP_ERROR(get_logger(), "Move to start position aborted.");
                return;
            }
        }

        if (experiment_csv_enabled_)
        {
            csv_reference_z_m_ = origin_pos.z();
            csv_travel_mm_ = get_parameter("z_max_offset_mm").as_double();
            if (!startCsvLogging()) return;
            logCsvRow();
        }

        // ----- Execute raster scan -----
        RCLCPP_INFO(get_logger(), "Starting raster scan (%zu points)...", cart_trajectory.size());
        rclcpp::Rate rate(control_rate_);
        double last_joint[DoF];
        for (int i = 0; i < DoF; i++)
            last_joint[i] = start_joint[i];

        size_t ik_failures = 0;
        for (size_t p = 1; p < cart_trajectory.size() && rclcpp::ok() && running_; p++)
        {
            VectorXd target_pose = VectorXd::Zero(6);
            target_pose << cart_trajectory[p](0), cart_trajectory[p](1), cart_trajectory[p](2),
                fixed_eul(0), fixed_eul(1), fixed_eul(2);

            double next_joint[DoF];
            if (!solveIKNumeric(target_pose, last_joint, next_joint, 200, num_free_joints, fix_wrist_joints))
            {
                if (!running_ || !rclcpp::ok()) return;
                ik_failures++;
                RCLCPP_ERROR(get_logger(), "No IK solution at point %zu; aborting scan.", p);
                return;
            }

            double distance = (cart_trajectory[p] - cart_trajectory[p - 1]).norm();
            double duration = distance / scan_speed_m_s;
            int n_sub = std::max(1, static_cast<int>(std::round(duration / control_dt)));

            double max_joint_step = 0.0;
            for (int j = 0; j < DoF; j++)
                max_joint_step = std::max(max_joint_step, std::fabs(next_joint[j] - last_joint[j]));
            int n_sub_vel = static_cast<int>(std::ceil(max_joint_step / (max_joint_vel * control_dt)));
            n_sub = std::max(n_sub, n_sub_vel);

            for (int k = 1; k <= n_sub && rclcpp::ok() && running_; k++)
            {
                double alpha = static_cast<double>(k) / n_sub;
                double interp[DoF];
                for (int j = 0; j < DoF; j++)
                    interp[j] = last_joint[j] + alpha * (next_joint[j] - last_joint[j]);

                if (!safetyCheck(interp))
                {
                    RCLCPP_ERROR(get_logger(), "Aborting raster scan at point %zu.", p);
                    return;
                }
                publishTargetJoints(interp);
                logCsvRow();
                rate.sleep();
            }

            for (int j = 0; j < DoF; j++)
                last_joint[j] = next_joint[j];

            if (experiment_csv_enabled_ && (p + 1 == cart_trajectory.size() || experimentBoundary(
                    cart_trajectory[p - 1], cart_trajectory[p], cart_trajectory[p + 1])))
            {
                if (!waitForExperimentArrival(*this, next_joint, target_pose,
                        [this](double *q) { return getFreshCurrentJoints(q); },
                        [this](const double *q) { publishTargetJoints(q); },
                        [this]() { logCsvRow(); }, [this]() { return running_.load(); },
                        [this](const double *q) { return safetyCheck(q); })) return;
            }
            if (p % 500 == 0)
            {
                RCLCPP_INFO(get_logger(), "Progress: %zu / %zu points", p, cart_trajectory.size());
            }
        }

        if (!rclcpp::ok() || !running_) return;
        // Hold final position briefly to prevent force-limit trips
        rclcpp::Rate hold_rate(control_rate_);
        for (int i = 0; i < static_cast<int>(control_rate_ / 2.0) && running_ && rclcpp::ok(); i++)
        {
            if (!safetyCheck(last_joint)) return;
            publishTargetJoints(last_joint);
            logCsvRow();
            hold_rate.sleep();
        }

        if (running_ && rclcpp::ok())
            RCLCPP_INFO(get_logger(), "Raster scan completed. Points with no IK solution: %zu", ik_failures);
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
            if (running_ && rclcpp::ok()) runRasterScan();
        }
        keyboard.stop();
        stopCsvLogging();
        if (keyboard.requested())
            returnAfterExperimentCancel(*this, running_,
                [this](double *q) { return getFreshCurrentJoints(q); },
                [this](const double *q) { publishTargetJoints(q); },
                [this](const double *q) { return safetyCheck(q); });
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
    auto node = std::make_shared<ProxskinTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
