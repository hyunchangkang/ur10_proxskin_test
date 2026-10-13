#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>

// The lower endpoint is zero; the saved upper endpoint is travel_mm.
// Preserve measured overshoot rather than clamping it to the nominal range.
inline double experimentDistanceMm(double actual_z_m, double start_z_m, double travel_mm)
{
    if (!std::isfinite(actual_z_m) || !std::isfinite(start_z_m) ||
        !std::isfinite(travel_mm) || travel_mm <= 0.0)
        throw std::invalid_argument("Invalid experiment distance reference");
    return travel_mm + (actual_z_m - start_z_m) * 1000.0;
}

inline std::string experimentCsvFilename(const std::string &mode)
{
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&seconds, &local);
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count() % 1000000;
    std::ostringstream name;
    name << "experiment_" << mode << '_' << std::put_time(&local, "%Y-%m-%d_%H-%M-%S")
         << '_' << std::setw(6) << std::setfill('0') << micros << ".csv";
    return name.str();
}

inline void writeExperimentCsvHeader(std::ostream &out)
{
    out << "time_s,robot_distance_mm,sensor_value\n";
}

inline void writeExperimentCsvRow(std::ostream &out, double elapsed_s, double distance_mm,
                                 std::optional<uint32_t> sensor = std::nullopt)
{
    // Avoid displaying negative zero after rounding a tiny measured error.
    if (std::abs(distance_mm) < 0.005) distance_mm = 0.0;
    out << std::fixed << std::setprecision(6) << elapsed_s << ','
        << std::setprecision(2) << distance_mm << ',';
    if (sensor) out << *sensor;
    out << '\n';
}
