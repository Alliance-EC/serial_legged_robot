#pragma once

#include <Eigen/Dense>
#include <bit>
#include <cstdint>

namespace device {
struct __attribute__((packed)) RC_Keyboard {
    constexpr static inline RC_Keyboard zero() {
        constexpr uint16_t zero = 0;
        return std::bit_cast<RC_Keyboard>(zero);
    }
    bool w     : 1;
    bool s     : 1;
    bool a     : 1;
    bool d     : 1;
    bool shift : 1;
    bool ctrl  : 1;
    bool q     : 1;
    bool e     : 1;
    bool r     : 1;
    bool f     : 1;
    bool g     : 1;
    bool z     : 1;
    bool x     : 1;
    bool c     : 1;
    bool v     : 1;
    bool b     : 1;
};

struct __attribute__((packed)) RC_Mouse {
    constexpr static inline RC_Mouse zero() {
        constexpr uint8_t zero = 0;
        return std::bit_cast<RC_Mouse>(zero);
    }
    bool left  : 1;
    bool right : 1;
};
enum class RC_Switch : uint8_t { UNKNOWN = 0, UP = 1, DOWN = 2, MIDDLE = 3 };
struct __attribute__((packed)) RC_Feedback {
    uint16_t joystick_channel0 : 11;
    uint16_t joystick_channel1 : 11;
    uint16_t joystick_channel2 : 11;
    uint16_t joystick_channel3 : 11;

    RC_Switch switch_right : 2;
    RC_Switch switch_left  : 2;

    int16_t mouse_velocity_x;
    int16_t mouse_velocity_y;
    int16_t mouse_velocity_z;

    bool mouse_left;
    bool mouse_right;

    RC_Keyboard keyboard;

    uint16_t dial;
};
struct RC_status {
    Eigen::Vector2f joystick_right = Eigen::Vector2f::Zero();
    Eigen::Vector2f joystick_left  = Eigen::Vector2f::Zero();

    RC_Switch switch_right = RC_Switch::UNKNOWN;
    RC_Switch switch_left  = RC_Switch::UNKNOWN;
};
} // namespace device