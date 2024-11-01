#pragma once

#include "bsp/can/can.hpp"
#include <cstdint>

namespace device {
struct __attribute__((packed)) SuperCapFeedback {
    uint16_t chassis_power;
    uint16_t supercap_voltage;
    uint16_t chassis_voltage;
    uint8_t enabled;
    uint8_t unused;
};
struct __attribute__((packed)) SuperCapControl {
    uint8_t placeholder[6] = {};
    uint8_t power_limit;
    bool enable;
};
struct SuperCapInfo {
    double chassis_power_;
    double chassis_voltage_;
    double supercap_voltage_;
    bool enabled_;
};
struct SuperCap_params {
    bsp::can_params can_params     = {};
    std::function<void()> callback = nullptr;
    SuperCap_params() {
        can_params.rx_id = 0x300;
        can_params.tx_id = 0x427;
    }
    SuperCap_params& set_can_instance(FDCAN_HandleTypeDef* can_handle) {
        return can_params.can_handle = can_handle, *this;
    }
    SuperCap_params& set_rx_id(uint32_t id) { return can_params.rx_id = id, *this; }
    SuperCap_params& set_tx_id(uint32_t id) { return can_params.tx_id = id, *this; }
};
class SuperCap {
public:
    explicit SuperCap(const SuperCap_params& params)
        : can_(params.can_params)
        , callback_(params.callback) {
        can_.SetCallback(std::bind(&SuperCap::Decode, this));
    }
    ~SuperCap() = default;
    void Decode() {
        SuperCapFeedback data;
        can_.GetRxData(reinterpret_cast<uint8_t*>(&data));
        Info.chassis_power_    = uint_to_double(data.chassis_power, 0.0, 500.0);
        Info.chassis_voltage_  = uint_to_double(data.chassis_voltage, 0.0, 50.0);
        Info.supercap_voltage_ = uint_to_double(data.supercap_voltage, 0.0, 50.0);
        Info.enabled_          = data.enabled;
        can_.Transmit(reinterpret_cast<uint8_t*>(&data_)); // 接收到数据后立即发送数据
    }
    void write_data(bool enabled, uint8_t power_limit) {
        data_.enable      = enabled;
        data_.power_limit = power_limit;
    }
    void write_data(const SuperCapControl& data) { data_ = data; }

    SuperCapInfo Info = {};

private:
    static constexpr double
        uint_to_double(std::unsigned_integral auto value, double min, double max) {
        double span   = max - min;
        double offset = min;
        return (double)value / (double)decltype(value)(-1) * span + offset;
    }
    bsp::can can_;
    std::function<void()> callback_ = nullptr;
    SuperCapControl data_;
};
} // namespace device