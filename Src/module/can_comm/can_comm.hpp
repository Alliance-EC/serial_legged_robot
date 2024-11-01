#pragma once

#include "app/system_parameters.hpp"
#include "bsp/can/can.hpp"
#include <cstdint>

namespace module {
struct ChassisStates {
    double leg_length_L;
    double leg_length_R;
    app::chassis_mode mode;
    double chassis_relative_angle;
};
struct __attribute__((packed)) ChassisStatesPackage {
    uint16_t leg_length_L;           //*1000
    uint16_t leg_length_R;           //*1000
    app::chassis_mode mode;
    uint16_t chassis_relative_angle; //*100
    uint8_t unused = {};
};
struct CanComm_params {
    bsp::can_params can_params     = {};
    std::function<void()> callback = nullptr;
    CanComm_params() {
        can_params.rx_id = 0x501;
        can_params.tx_id = 0x500;
    }
    CanComm_params& set_can_instance(FDCAN_HandleTypeDef* can_handle) {
        return can_params.can_handle = can_handle, *this;
    }
    CanComm_params& set_rx_id(uint32_t id) { return can_params.rx_id = id, *this; }
    CanComm_params& set_tx_id(uint32_t id) { return can_params.tx_id = id, *this; }
};
class CanComm {
public:
    explicit CanComm(const CanComm_params& params)
        : can_(params.can_params)
        , callback_(params.callback) {
        // can_.SetCallback(std::bind(&CanComm::Decode, this));
    }
    ~CanComm() = default;
    void Decode() {}
    void Send(const ChassisStates& data) {
        auto normalize_angle_degrees = [](double angle) -> double {
            double normalized_angle = fmod(angle, 360.0); // 使用fmod进行模运算
            if (normalized_angle < 0) {
                normalized_angle += 360.0;                // 如果结果为负，加上360度
            }
            return normalized_angle;
        };
        data_.leg_length_L = static_cast<uint16_t>(data.leg_length_L * 1000);
        data_.leg_length_R = static_cast<uint16_t>(data.leg_length_R * 1000);
        data_.mode         = data.mode;
        data_.chassis_relative_angle =
            static_cast<uint16_t>(normalize_angle_degrees(data.chassis_relative_angle) * 100);
        can_.Transmit(reinterpret_cast<uint8_t*>(&data_), sizeof(data_));
    }

private:
    bsp::can can_;
    std::function<void()> callback_ = nullptr;
    ChassisStatesPackage data_;
};
} // namespace module