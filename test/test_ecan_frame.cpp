#include <array>
#include <sstream>
#include <stdexcept>
#include "ur10_proxskin_test/ecan_frame.hpp"
#include "ur10_proxskin_test/experiment_csv.hpp"

int main()
{
    // 40000001 cannot be represented exactly as float32. Preserve every raw bit.
    std::array<uint8_t, 14> bytes{0, 0, 0, 0, 0x41, 8, 0x02, 0x62, 0x5a, 0x01,
                               0xff, 0xff, 0xff, 0xff};
    const auto frame = decodeEcanFrame(bytes.data(), bytes.size());
    if (!frame || frame->can_id != 0x41 || frame->front != 40000001 || frame->back != UINT32_MAX)
        throw std::runtime_error("CAN 0x41 must decode front/back as exact big-endian uint32 values");
    std::ostringstream csv;
    writeExperimentCsvRow(csv, .01, 299.5, frame->front);
    if (csv.str() != "0.010000,299.50,40000001\n")
        throw std::runtime_error("The third CSV column must preserve the complete raw integer");
    bytes[5] = 7;
    if (decodeEcanFrame(bytes.data(), bytes.size()) || decodeEcanFrame(bytes.data(), 13))
        throw std::runtime_error("Reject short frames and DLC other than eight");
    bytes[5] = 8;
    for (size_t i = 6; i < bytes.size(); ++i) bytes[i] = 0;
    if (decodeEcanFrame(bytes.data(), bytes.size())->front != 0)
        throw std::runtime_error("Zero is valid sensor data");
}
