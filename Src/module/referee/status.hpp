#pragma once
#include "bsp/uart/uart.hpp"
#include "dji_crc.hpp"
#include "field.hpp"
#include "frame.hpp"
#include "tool/daemon/daemon.hpp"
#include "usart.h"

namespace module::referee {
class Status {
public:
    static Status* GetInstance() {
        static auto instance = new Status();
        return instance;
    }

    void update() {
        const auto data_ptr = referee_uart_.GetRxBuffer();
        auto data_size      = referee_uart_.GetTrueRxSize();
        uint16_t ptr_offset = 0;
        while (ptr_offset < data_size) {
            const auto now_ptr = data_ptr + ptr_offset;
            if (ptr_offset + sizeof(frame_.header) > data_size) // 防止越界访问
                break;
            std::copy(
                now_ptr, now_ptr + sizeof(frame_.header),
                reinterpret_cast<uint8_t*>(&frame_.header));
            auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id)
                            + frame_.header.data_length + sizeof(uint16_t);
            if (ptr_offset + frame_size > data_size)
                break;

            std::copy(now_ptr, now_ptr + frame_size, reinterpret_cast<uint8_t*>(&frame_));
            if (dji_crc::verify_crc8(frame_.header)) {
                if (dji_crc::verify_crc16(&frame_, frame_size)) {
                    process_frame();
                    ptr_offset += frame_size;
                    continue;
                }
            }
            ptr_offset++;
        }
    }

    double chassis_power_limit_ = safe_chassis_power_limit;
    uint8_t chassis_power_level = 0;
    bool chassis_power_status   = false;
    bool gimbal_power_status    = false;
    double chassis_power_       = 0;
    double buffer_energy_       = 60;

private:
    Status()                                      = default;    // 禁止外部构造
    ~Status()                                     = default;    // 禁止外部析构
    Status(const Status& Status)                  = delete;     // 禁止外部拷贝构造
    const Status& operator=(const Status& Status) = delete;     // 禁止外部赋值操作

    Frame frame_;

    tool::daemon robot_status_daemon_ =
        tool::daemon(30, std::bind(&Status::robot_status_receive_error_callback, this));
    tool::daemon power_heat_daemon_ =
        tool::daemon(3, std::bind(&Status::power_heat_receive_error_callback, this));
    ;
    bsp::uart referee_uart_ = bsp::uart({&huart7, sizeof(Frame), std::bind(&Status::update, this)});

    // When referee system loses connection unexpectedly,
    // use these indicators make sure the robot safe.
    // Chassis: Health priority with level 1
    static constexpr double safe_chassis_power_limit = 45;

    void process_frame() {
        auto command_id = frame_.body.command_id;
        if (command_id == 0x0001)
            update_game_status();
        if (command_id == 0x0003)
            update_game_robot_hp();
        else if (command_id == 0x0201)
            update_robot_status();
        else if (command_id == 0x0202)
            update_power_heat_data();
        else if (command_id == 0x0203)
            update_robot_position();
        else if (command_id == 0x0206)
            update_hurt_data();
        else if (command_id == 0x0207)
            update_shoot_data();
        else if (command_id == 0x0208)
            update_bullet_allowance();
        else if (command_id == 0x020B)
            update_game_robot_position();
    }

    void update_game_status() {}

    void update_game_robot_hp() {}

    void update_robot_status() {
        // 30s check
        robot_status_daemon_.reload();

        auto& data           = reinterpret_cast<RobotStatus&>(frame_.body.data);
        chassis_power_limit_ = static_cast<double>(data.chassis_power_limit);
        chassis_power_level  = data.robot_level;
        chassis_power_status = data.power_management_chassis_output;
        gimbal_power_status  = data.power_management_gimbal_output;
    }

    void update_power_heat_data() {
        // 3s check
        power_heat_daemon_.reload();

        auto& data     = reinterpret_cast<PowerHeatData&>(frame_.body.data);
        chassis_power_ = data.chassis_power;
        buffer_energy_ = static_cast<double>(data.buffer_energy);
    }

    void update_robot_position() {}

    void update_hurt_data() {}

    void update_shoot_data() {}

    void update_bullet_allowance() {}

    void update_game_robot_position() {}

    void robot_status_receive_error_callback() { chassis_power_limit_ = safe_chassis_power_limit; }
    void power_heat_receive_error_callback() {
        chassis_power_ = 0.0;
        buffer_energy_ = 60.0;
    }
};

} // namespace module::referee