#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include "ur10_proxskin_test/joint_startup_plan.hpp"

struct CalibrationCommand
{
    enum Action { Move, Save, Cancel, Help } action;
    double delta_mm = 0.0;
};

inline CalibrationCommand parseCalibrationCommand(std::string text)
{
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        throw std::invalid_argument("Enter a Z adjustment or 'done'.");
    text = text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
    for (char &c : text)
        if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    if (text == "됐다" || text == "저장" || text == "완료" || text == "done" || text == "save")
        return {CalibrationCommand::Save, 0.0};
    if (text == "취소" || text == "cancel" || text == "quit")
        return {CalibrationCommand::Cancel, 0.0};
    if (text == "?" || text == "help" || text == "도움말")
        return {CalibrationCommand::Help, 0.0};
    std::smatch match;
    double delta = 0.0;
    if (std::regex_match(text, match, std::regex(R"(^([+-]?[0-9]+)\s*(mm)?$)")))
        delta = std::stod(match[1].str());
    else if (std::regex_match(text, match, std::regex(R"(^(위로|위|up|아래로|아래|down)\s*([0-9]+)\s*(mm)?$)")))
    {
        delta = std::stod(match[2].str());
        if (match[1] == "아래로" || match[1] == "아래" || match[1] == "down") delta = -delta;
    }
    else
        throw std::invalid_argument("Use whole millimeters: +10, -1, up 10mm, down 1mm, done, cancel.");
    if (!std::isfinite(delta) || delta == 0.0 || std::abs(delta) > 300.0)
        throw std::invalid_argument("Each adjustment must be 1..300 whole millimeters, with a direction.");
    return {CalibrationCommand::Move, delta};
}

inline double parseExperimentStartDistanceMm(const std::string &text)
{
    double distance = 0.0;
    size_t parsed = 0;
    try
    {
        distance = std::stod(text, &parsed);
    }
    catch (const std::exception &)
    {
        throw std::invalid_argument("Enter a positive distance in mm, such as 200 or 125.5.");
    }
    if (!std::isfinite(distance) || distance <= 0.0 ||
        text.find_first_not_of(" \t\r\n", parsed) != std::string::npos)
        throw std::invalid_argument("Enter a positive distance in mm, such as 200 or 125.5.");
    return distance;
}

inline std::string calibrationJsonString(const std::string &value)
{
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value)
    {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
        else out << c;
    }
    out << '"';
    return out.str();
}

inline void saveSensorZero(const std::string &path, const std::string &robot_name,
                           const VectorXd &base_pose, const VectorXd &zero_pose,
                           const StartupJoints &zero_joints)
{
    if (path.empty() || base_pose.size() != 6 || zero_pose.size() != 6 ||
        !base_pose.allFinite() || !zero_pose.allFinite())
        throw std::invalid_argument("Invalid sensor zero data or path");
    for (double q : zero_joints)
        if (!std::isfinite(q) || std::abs(q) > 2 * PI)
            throw std::invalid_argument("Invalid sensor zero joint angle");
    const std::filesystem::path destination(path);
    if (!destination.parent_path().empty())
        std::filesystem::create_directories(destination.parent_path());
    const std::string temporary = path + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    if (!out) throw std::runtime_error("Cannot open sensor zero file: " + temporary);
    out << std::setprecision(17);
    auto array = [&](const auto &values) {
        out << '[';
        for (int i = 0; i < 6; ++i) out << (i ? ", " : "") << values[i];
        out << ']';
    };
    out << "{\n  \"version\": 1,\n  \"robot_name\": " << calibrationJsonString(robot_name)
        << ",\n  \"base_tcp_pose\": "; array(base_pose);
    out << ",\n  \"sensor_zero_tcp_pose\": "; array(zero_pose);
    out << ",\n  \"sensor_zero_joint_rad\": "; array(zero_joints);
    out << ",\n  \"z_offset_mm\": " << (zero_pose[2] - base_pose[2]) * 1000.0
        << ",\n  \"saved_at_unix_seconds\": "
        << std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
        << "\n}\n";
    out.flush();
    if (!out) throw std::runtime_error("Failed to write sensor zero file");
    out.close();
    std::filesystem::rename(temporary, destination);
}

inline void saveExperimentStart(const std::string &path, const std::string &robot_name,
                                const VectorXd &pose, const StartupJoints &joints,
                                bool orientation_aligned = false, double start_distance_mm = 300.0)
{
    if (path.empty() || pose.size() != 6 || !pose.allFinite())
        throw std::invalid_argument("Invalid experiment start pose or path");
    if (!std::isfinite(start_distance_mm) || start_distance_mm <= 0.0)
        throw std::invalid_argument("Start distance must be a positive number in mm");
    for (double q : joints)
        if (!std::isfinite(q) || std::abs(q) > 2 * PI)
            throw std::invalid_argument("Invalid experiment start joint angle");
    const std::filesystem::path destination(path);
    if (!destination.parent_path().empty())
        std::filesystem::create_directories(destination.parent_path());
    const std::string temporary = path + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    if (!out) throw std::runtime_error("Cannot open experiment start file: " + temporary);
    out << std::setprecision(17);
    auto array = [&](const auto &values) {
        out << '[';
        for (int i = 0; i < 6; ++i) out << (i ? ", " : "") << values[i];
        out << ']';
    };
    out << "{\n  \"version\": 2,\n  \"start_distance_mm\": " << start_distance_mm
        << ",\n  \"robot_name\": " << calibrationJsonString(robot_name)
        << ",\n  \"tcp_pose\": "; array(pose);
    out << ",\n  \"joint_rad\": "; array(joints);
    out << ",\n  \"orientation_aligned\": " << (orientation_aligned ? "true" : "false");
    out << ",\n  \"saved_at_unix_seconds\": "
        << std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
        << "\n}\n";
    out.flush();
    if (!out) throw std::runtime_error("Failed to write experiment start file");
    out.close();
    std::filesystem::rename(temporary, destination);
}
