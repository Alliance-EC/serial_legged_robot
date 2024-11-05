#pragma once
#include "app/system_parameters.hpp"
#include "controller.hpp"
#include "device/Dji_motor/DJI_motor.hpp"
#include "module/M3508/M3508.hpp"
#include "tool/PID/PID.hpp"
namespace app::controller {
class SendProcess {
public:
    static SendProcess* GetInstance() {
        static auto instance = new SendProcess();
        return instance;
    }
    void Send() {
        switch (*mode_) {
            // motor_fixed();//原双下收腿
            // break;
        case chassis_mode::balanceless:
            motor_fixed(torque_->wheel_L, torque_->wheel_R); // in this case,torque means velocity
            M3508_sender_1_->write_command(M3508_[leg_LF], torque_->leg_LF);
            M3508_sender_1_->write_command(M3508_[leg_LB], torque_->leg_LB);
            M3508_sender_1_->write_command(M3508_[leg_RF], torque_->leg_RF);
            M3508_sender_1_->write_command(M3508_[leg_RB], torque_->leg_RB);
            break;
        case chassis_mode::stop:
        case chassis_mode::unknown:
        default:
            M3508_sender_1_->write_command(M3508_[leg_LF], torque_->leg_LF);
            M3508_sender_1_->write_command(M3508_[leg_LB], torque_->leg_LB);
            M3508_sender_1_->write_command(M3508_[leg_RF], torque_->leg_RF);
            M3508_sender_1_->write_command(M3508_[leg_RB], torque_->leg_RB);
            // M3508_sender_2_->write_command(M3508_[wheel_L-4], torque_->wheel_L);
            // M3508_sender_2_->write_command(M3508_[wheel_R-4], torque_->wheel_R);
            break;
        };
        M3508_sender_1_->send();
        // M3508_sender_2_->send();
    }
    void Init(
        std::array<module::M3508*, 6> M3508, device::DjiMotor_sender* m3508_sender_1,
        device::DjiMotor_sender* m3508_sender_2, chassis_mode* mode) {
        M3508_        = M3508;
        M3508_sender_1_ = m3508_sender_1;
        M3508_sender_2_ = m3508_sender_2;
        mode_         = mode;
    }
    // double get_torque(int motor_id) {
    //     switch (motor_id){
    //         case leg_LF:return torque_->leg_LF;
    //         case leg_LB:return torque_->leg_LB;
    //         case leg_RF: return torque_->leg_RF;
    //         case leg_RB: return torque_->leg_RB;
    //         case wheel_L: return torque_->wheel_L;
    //         case wheel_R: return torque_->wheel_R;
    //     }
    // }

private:
    SendProcess()                               = default;   // 禁止外部构造
    ~SendProcess()                              = default;   // 禁止外部析构
    SendProcess(const SendProcess& SendProcess) = delete;    // 禁止外部拷贝构造
    const SendProcess& operator=(const SendProcess& SendProcess) = delete; // 禁止外部赋值操作

    device::DjiMotor_sender* M3508_sender_1_ = nullptr;
    device::DjiMotor_sender* M3508_sender_2_ = nullptr;
    std::array<module::M3508*, 6> M3508_   = {};

    const chassis_mode* mode_     = nullptr;
    const control_torque* torque_ = &Controller::GetInstance()->control_torque_;
    tool::PID wheel_L_PID_        = tool::PID({0.6, 0.0, 0.0, 4.0, 0.0, 0.0, dt});
    tool::PID wheel_R_PID_        = tool::PID({0.6, 0.0, 0.0, 4.0, 0.0, 0.0, dt});

    inline void motor_fixed(double wheel_L_speed = 0, double wheel_R_speed = 0) {
        M3508_sender_2_->write_command(
            M3508_[wheel_L-4], wheel_L_PID_.update(wheel_L_speed, M3508_[wheel_L]->get_velocity()));
        M3508_sender_2_->write_command(
            M3508_[wheel_R-4], wheel_R_PID_.update(wheel_R_speed, M3508_[wheel_R]->get_velocity()));
    }
};
} // namespace app::controller