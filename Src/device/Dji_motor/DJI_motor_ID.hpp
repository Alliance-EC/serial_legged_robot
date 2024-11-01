#pragma once

#include <cstdint>
#include <type_traits>
namespace device {
template <typename T>
std::enable_if_t<std::is_enum_v<T>, uint32_t> toU32(T enum_val) {
    return static_cast<uint32_t>(enum_val);
}
enum class M3508_ID : uint16_t {
    ID1 = 0x201,
    ID2,
    ID3,
    ID4,
    ID5,
    ID6,
    ID7,
    ID8,
};
enum class DM8009_ID : uint16_t {
    ID1 = 0x301,
    ID2,
    ID3,
    ID4,
    ID5,
    ID6,
    ID7,
    ID8,
};
enum class GM6020_ID : uint16_t {
    ID1 = 0x205,
    ID2,
    ID3,
    ID4,
    ID5,
    ID6,
    ID7,
    ID8,
};
enum class M3508_sendID : uint16_t {
    ID1 = 0x200,
    ID2 = 0x1FF,
};
enum class GM6020_sendID : uint16_t {
    ID1 = 0x1FF,
    ID2 = 0x2FF,
};
enum class DM8009_sendID : uint16_t {
    ID1 = 0x3FE,
    ID2 = 0x4FE,
};
enum class DjiMotorType : uint8_t {
    UNKNOWN        = 0,
    GM6020         = 1,
    GM6020_VOLTAGE = 2,
    M3508_15       = 3,
    M3508_19       = 4,
    M2006          = 5,
    DM8009         = 6,
};
} // namespace device