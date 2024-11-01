#pragma once

#include "DJI_motor_ID.hpp"
#include "bsp/can/can.hpp"
#include "tool/endian_promise.hpp"
#include <cstdint>
#include <cstring>
#include <numbers>
#include <sys/types.h>

namespace device {
using namespace tool;
struct DjiMotorConfig {
    DjiMotorType motor_type;
    int encoder_zero_point;
    double reduction_ratio;
    bool reversed;
    bool multi_turn_angle_enabled;

    explicit DjiMotorConfig(DjiMotorType motor_type) {
        this->encoder_zero_point = 0.0;
        this->motor_type         = motor_type;
        switch (motor_type) {
        case DjiMotorType::UNKNOWN:
        case DjiMotorType::GM6020:
        case DjiMotorType::GM6020_VOLTAGE: reduction_ratio = 1.0; break;
        case DjiMotorType::M3508_15: reduction_ratio = 268.0 / 17.0; break;
        case DjiMotorType::M3508_19: reduction_ratio = 3591.0 / 187.0; break;
        case DjiMotorType::M2006: reduction_ratio = 36.0; break;
        case DjiMotorType::DM8009: reduction_ratio = 1.0; break; // 数据是输出轴的
        }
        this->reversed                 = false;
        this->multi_turn_angle_enabled = true;
    }

    DjiMotorConfig& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
    DjiMotorConfig& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
    DjiMotorConfig& reverse() { return reversed = true, *this; }
    DjiMotorConfig& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
};
struct __attribute__((packed)) DjiMotorFeedback {
    be_int16_t angle;
    be_int16_t velocity;
    be_int16_t current;
    uint8_t temperature;
    uint8_t unused;
};
struct __attribute__((packed)) DjiMotorControl {
    be_int16_t current[4] = {};
};
struct DjiMotor_params {
    bsp::can_params can_params     = {};
    std::function<void()> callback = nullptr;
    virtual DjiMotor_params& set_can_instance(FDCAN_HandleTypeDef* can_handle) {
        return can_params.can_handle = can_handle, *this;
    }
    virtual DjiMotor_params& set_rx_id(uint32_t id) { return can_params.rx_id = id, *this; }
    virtual DjiMotor_params& set_tx_id(uint32_t id) { return can_params.tx_id = id, *this; }
};
class DjiMotor {
public:
    explicit DjiMotor(const DjiMotor_params& params)
        : can_(params.can_params)
        , callback_(params.callback) {
        can_.SetCallback(std::bind(&DjiMotor::Decode, this));
        encoder_zero_point_       = 0;
        last_raw_angle_           = 0;
        multi_turn_angle_enabled_ = true;

        raw_angle_to_angle_coefficient_ = angle_to_raw_angle_coefficient_ = 0.0;
        raw_current_to_torque_coefficient_ = torque_to_raw_current_coefficient_ = 0.0;
    }
    DjiMotor(const DjiMotor&)            = delete;
    DjiMotor& operator=(const DjiMotor&) = delete;

    void configure(const DjiMotorConfig& config) {
        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += raw_angle_max_;

        double sign = config.reversed ? -1 : 1;

        raw_angle_to_angle_coefficient_ = static_cast<double>(
            sign / config.reduction_ratio / raw_angle_max_ * 2.0 * std::numbers::pi);
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            static_cast<double>(sign / config.reduction_ratio / 60 * 2 * std::numbers::pi);
        if (config.motor_type == DjiMotorType::DM8009)
            raw_velocity_to_velocity_coefficient_ *= 0.01;       // 原始数据被放大了100倍
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;

        double torque_constant = 0.0, raw_current_max = 0.0, current_max = 0.0;
        switch (config.motor_type) {
        case DjiMotorType::GM6020:
            torque_constant = 0.741;
            raw_current_max = 16384.0;
            current_max     = 3.0;
            break;
        case DjiMotorType::GM6020_VOLTAGE:
            torque_constant = 0.741;
            raw_current_max = 25000.0;
            current_max     = 3.0;
            break;
        case DjiMotorType::M3508_15:
        case DjiMotorType::M3508_19:
            torque_constant = 0.3 * 187.0 / 3591.0;
            raw_current_max = 16384.0;
            current_max     = 20.0;
            break;
        case DjiMotorType::M2006:
            torque_constant = 0.18 * 1.0 / 36.0;
            raw_current_max = 16384.0;
            current_max     = 10.0;
            break;
        case DjiMotorType::DM8009:                               // 假装是DJI的电机
            torque_constant = 1.261575;
            raw_current_max = 16384.0;
            current_max     = 32.835820896;
            break;
        default: break;
        }

        raw_current_to_torque_coefficient_ =
            sign * config.reduction_ratio * torque_constant / raw_current_max * current_max;
        torque_to_raw_current_coefficient_ = 1 / raw_current_to_torque_coefficient_;
        if (config.motor_type == DjiMotorType::DM8009)
            raw_current_to_torque_coefficient_ = sign * torque_constant / 1000; // feedback unit:mA

        reduction_ratio_ = config.reduction_ratio;
        max_torque_      = 1 * config.reduction_ratio * torque_constant * current_max;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_         = 0;
        motor_type_               = config.motor_type;
    }

    void Decode() {
        can_.GetRxData(reinterpret_cast<uint8_t*>(&raw_data_));
        int raw_angle = raw_data_.angle;
        if (!raw_angle_offset_set) {
            encoder_zero_point_ += raw_angle;
            raw_angle_offset_set = true;
        }

        // Angle unit: rad
        int angle = raw_angle - encoder_zero_point_;
        if (angle < 0)
            angle += raw_angle_max_;
        if (!multi_turn_angle_enabled_) {
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle);
        } else {
            auto diff = (angle - angle_multi_turn_) % raw_angle_max_;
            if (diff <= -raw_angle_max_ / 2)
                diff += raw_angle_max_;
            else if (diff > raw_angle_max_ / 2)
                diff -= raw_angle_max_;
            angle_multi_turn_ += diff;
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }

        // Velocity unit: rad/s
        velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(raw_data_.velocity);

        // Torque unit: N*m
        torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(raw_data_.current);

        last_raw_angle_ = raw_angle;
    }
    void SetOfflineCallback(const std::function<void()>& callback) {
        can_.SetOfflineCallback(callback);
    }
    int calibrate_zero_point() {
        angle_multi_turn_   = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    void reset_angle() { angle_multi_turn_ = 0; }

    [[nodiscard]] virtual double get_angle() const { return angle_; }
    [[nodiscard]] double get_velocity() const { return velocity_; }
    [[nodiscard]] double get_torque() const { return torque_; }
    [[nodiscard]] double get_max_torque() const { return max_torque_; }

    [[nodiscard]] double get_raw_current() const { return static_cast<double>(raw_data_.current); }

    [[nodiscard]] uint8_t get_index() const {
        switch (motor_type_) {
        case DjiMotorType::M3508_15: return can_.GetRxId() - static_cast<uint16_t>(M3508_ID::ID1);
        case DjiMotorType::M3508_19: return can_.GetRxId() - static_cast<uint16_t>(M3508_ID::ID1);
        case DjiMotorType::DM8009: return can_.GetRxId() - static_cast<uint16_t>(DM8009_ID::ID1);
        default: return 0;
        }
    }

private:
    bsp::can can_;
    std::function<void()> callback_;
    static constexpr int raw_angle_max_ = 8192;
    int encoder_zero_point_, last_raw_angle_;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;
    bool raw_angle_offset_set = false; // 上电时编码器初始值

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    double reduction_ratio_;
    double max_torque_;

    double angle_ = 0.0;
    double velocity_;
    double torque_;

    DjiMotorType motor_type_;
    DjiMotorFeedback raw_data_;

    friend class DjiMotor_sender;
};
class DjiMotor_sender {
public:
    explicit DjiMotor_sender(const DjiMotor_params& params)
        : can_(params.can_params) {}
    void write_command(DjiMotor* motor, double torque) {
        if (std::isnan(torque)) {
            data_.current[motor->get_index()] = 0;
            return;
        }
        double max_torque = motor->get_max_torque();
        torque            = std::clamp(torque, -max_torque, max_torque);

        double current = std::round(motor->torque_to_raw_current_coefficient_ * torque);
        data_.current[motor->get_index()] = static_cast<short>(current);
    }
    void send() {
        // can_.SetDLC(sizeof(DjiMotorControl));
        can_.Transmit(reinterpret_cast<uint8_t*>(&data_));
        data_ = DjiMotorControl();
    }
    void send_DM8009() {
        struct __attribute__((packed)) {
            le_int16_t current[4] = {};
        } DMdata_ = {};
        for (int i = 0; i < 4; i++) {
            DMdata_.current[i] = static_cast<le_uint16_t>(data_.current[i]);
        }
        can_.Transmit(reinterpret_cast<uint8_t*>(&DMdata_));
        data_ = DjiMotorControl();
    }

private:
    bsp::can can_;
    DjiMotorControl data_ = {};
};
} // namespace device