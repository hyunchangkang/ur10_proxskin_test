#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

// Same wire format and integer conversion as
// Prox_sensor_React/src/EthernetCommunication.cpp::parse_frame:
// 14 bytes, CAN ID at 4, DLC at 5, front at 6..9, back at 10..13.
struct EcanFrame
{
    uint32_t can_id;
    uint32_t front;
    uint32_t back;
};

inline uint32_t decodeEcanBigEndianU32(const uint8_t *data)
{
    return (static_cast<uint32_t>(data[0]) << 24u) |
           (static_cast<uint32_t>(data[1]) << 16u) |
           (static_cast<uint32_t>(data[2]) << 8u) | data[3];
}

inline std::optional<EcanFrame> decodeEcanFrame(const uint8_t *frame, size_t size)
{
    if (size != 14 || frame[5] != 8) return std::nullopt;
    return EcanFrame{frame[4], decodeEcanBigEndianU32(frame + 6), decodeEcanBigEndianU32(frame + 10)};
}
