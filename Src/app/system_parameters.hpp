#pragma once

#include <cstdint>

namespace app {
constexpr double m_b   = 4.4;  // 身体质量
constexpr double eta_l = 0; // 质心位置系数
constexpr double m_l   = 0.6;// 单腿质量
constexpr double R_l = 0.17; // 半轮距
constexpr double g   = 9.80665;
constexpr double Rw  = 0.06; // 轮子半径

constexpr double dt = 0.001; // 执行周期

enum class chassis_mode : uint8_t {
    stop = 0,
    follow,
    spin,
    balanceless,
    unknown,
};
enum class board : uint8_t {
    UNKNOWN           = 0,
    DM_MC02           = 1,
    Robomaster_Cboard = 2,
};
enum wheel : uint8_t {
    wheel_L = 5 - 1,
    wheel_R = 6 - 1,
};
enum leg : uint8_t {
    leg_LF = 1 - 1,
    leg_LB = 2 - 1,
    leg_RF = 3 - 1,
    leg_RB = 4 - 1,
};
constexpr board this_board = board::DM_MC02;
} // namespace app