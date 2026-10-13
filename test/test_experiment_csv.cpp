#include <iostream>
#include <sstream>
#include <stdexcept>
#include "ur10_proxskin_test/experiment_csv.hpp"

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    const double start_z = 0.5945;
    require(std::abs(experimentDistanceMm(start_z, start_z, 300) - 300) < 1e-10,
            "30 cm start must be recorded as 300 mm");
    require(std::abs(experimentDistanceMm(start_z - 0.3, start_z, 300)) < 1e-10,
            "30 cm downward endpoint must be recorded as zero");
    require(std::abs(experimentDistanceMm(start_z - 0.123456, start_z, 300) - 176.544) < 1e-10,
            "measured displacement must be converted from meters to millimeters");
    require(experimentDistanceMm(start_z - 0.301, start_z, 300) < 0,
            "measured overshoot must not be hidden by clamping");
    require(std::abs(experimentDistanceMm(start_z - 0.02, start_z, 20)) < 1e-10,
            "selected travel must define the distance reference");
    std::ostringstream csv;
    writeExperimentCsvHeader(csv);
    writeExperimentCsvRow(csv, 0.0, 300.0);
    writeExperimentCsvRow(csv, 0.008123, 176.544);
    writeExperimentCsvRow(csv, 6.0, -0.000001);
    writeExperimentCsvRow(csv, 6.008, 0.05, 0);
    writeExperimentCsvRow(csv, 7.0, 12.345, 123456);
    require(csv.str() == "time_s,robot_distance_mm,sensor_value\n"
                         "0.000000,300.00,\n"
                         "0.008123,176.54,\n"
                         "6.000000,0.00,\n"
                         "6.008000,0.05,0\n"
                         "7.000000,12.35,123456\n",
            "CSV must have exactly three columns, two decimal millimeters and distinct missing/zero sensor values");
    std::cout << "PASS: feedback distance reference, millimeters, timestamps and sensor placeholder\n";
}
