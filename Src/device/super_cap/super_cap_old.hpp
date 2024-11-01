#pragma once

#include "bsp/can/can.hpp"
#include <cstdint>

namespace device {
enum class watchdog_state : uint8_t {
    on = 1,
    off,
    feed_dog,
};
struct __attribute__((packed)) SuperCapFeedback {          // 50Hz
    float voltage;
    bool status;
    bool CtrlBoard_status;
    uint8_t unused[2];
};
struct __attribute__((packed)) SuperCapControl {
    bool control_ON = false;
    uint8_t power_level;                                   // 0-10
    uint8_t power_remain           = 1;                    // 暂时用不到
    watchdog_state watchdog_states = watchdog_state::off;
    uint8_t unused[4]              = {};
};
struct SuperCapInfo {
    float voltage = 0;
    bool status   = false;
    bool CtrlBoard_status;
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
        Info.voltage          = data.voltage;
        Info.status           = !data.status;              // 协议中取反
        Info.CtrlBoard_status = data.CtrlBoard_status;
        can_.Transmit(reinterpret_cast<uint8_t*>(&data_)); // 接收到数据后立即发送数据
    }
    void write_data(bool control_ON, uint8_t voltage_level, uint8_t power_remain) {
        data_.control_ON   = control_ON;
        data_.power_level  = voltage_level;
        data_.power_remain = power_remain;
    }
    void write_data(const SuperCapControl& data) { data_ = data; }

    SuperCapInfo Info = {};

private:
    bsp::can can_;
    std::function<void()> callback_ = nullptr;
    SuperCapControl data_;
};
} // namespace device