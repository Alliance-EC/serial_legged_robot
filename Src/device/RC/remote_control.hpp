#pragma once

#include "app/system_parameters.hpp"
#include "bsp/uart/uart.hpp"
#include "remote_control_data.hpp"
#include "tool/daemon/daemon.hpp"
#include <cstdint>
#include <cstring>
#include <functional>

namespace device {
struct RC_params {
    bsp::uart_params uart_params   = {};
    std::function<void()> callback = nullptr;
    explicit RC_params(app::board board) {
        switch (board) {
        case app::board::UNKNOWN:
        case app::board::DM_MC02:
            uart_params = {
                .uart_handle = &huart5,
            };
            break;
        default: break;
        }
        this->uart_params.recv_buff_size = sizeof(RC_Feedback);
    }
};
class remote_control {
public:
    explicit remote_control(const RC_params& params)
        : rc_uart_(params.uart_params)
        , daemon_(0.5, std::bind(&remote_control::RemoteControlLostCallback, this)) {
        rc_uart_.SetCallback(std::bind(&remote_control::RemoteControlRxCallback, this));
    }
    ~remote_control() = default;

    void SetCallback(const std::function<void()>& callback) {
        if (callback)
            callback_ = callback;
    }
    RC_status data;

private:
    bsp::uart rc_uart_;
    tool::daemon daemon_;
    std::function<void()> callback_;

    void RemoteControlRxCallback() {
        RC_Feedback feedback;
        auto data_ptr = rc_uart_.GetRxBuffer();
        std::copy(data_ptr, data_ptr + sizeof(RC_Feedback), reinterpret_cast<uint8_t*>(&feedback));

        auto channel_to_float = [](uint16_t value) {
            return static_cast<float>((static_cast<int32_t>(value) - 1024) / 660.0);
        };
        data.joystick_right.y() = -channel_to_float(feedback.joystick_channel0);
        data.joystick_right.x() = channel_to_float(feedback.joystick_channel1);
        data.joystick_left.y()  = -channel_to_float(feedback.joystick_channel2);
        data.joystick_left.x()  = channel_to_float(feedback.joystick_channel3);

        data.switch_right = feedback.switch_right;
        data.switch_left  = feedback.switch_left;

        daemon_.reload();
    }
    void RemoteControlLostCallback() {
        data.joystick_right.setZero();
        data.joystick_left.setZero();
        data.switch_right = RC_Switch::DOWN;
        data.switch_left  = RC_Switch::DOWN;
    }
};
} // namespace device