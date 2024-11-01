#pragma once
#include "app/observer/observer.hpp"
#include "app/system_parameters.hpp"
#include "bsp/dwt/dwt.h"
#include "device/Dji_motor/DJI_motor.hpp"
#include "device/RC/remote_control_data.hpp"
#include "module/IMU/IMU.hpp"
#include "module/referee/status.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <numbers>

namespace app::controller {
struct desire {
    Eigen::Matrix<double, 10, 1> xd = Eigen::Matrix<double, 10, 1>::Zero();
    double roll                     = 0;
    double leg_angle                =0;
    double leg_length               = 0.12;
};
class DesireSet {
public:
    static DesireSet* GetInstance() {
        static auto instance = new DesireSet();
        return instance;
    }
    static constexpr double spinning_velocity = -5.0;
    static constexpr double x_velocity_scale  = 2.5;
    void update() {
        using namespace device;

        do {
            // 双下/未知状态/
            if ((RC_->switch_left == RC_Switch::UNKNOWN || RC_->switch_right == RC_Switch::UNKNOWN)
                || ((RC_->switch_left == RC_Switch::DOWN)
                    && (RC_->switch_right == RC_Switch::DOWN))) {
                reset_all_controls();
                *mode_ = chassis_mode::stop;
                break;
            }

            if (((RC_->switch_left == RC_Switch::MIDDLE) && (RC_->switch_right == RC_Switch::DOWN))
                || balanceless_mode_) {
                if (last_switch_right != RC_Switch::DOWN) {
                    reset_all_controls();
                }

                *mode_ = chassis_mode::balanceless;
            } else if (RC_->switch_right == RC_Switch::MIDDLE) {
                if (last_switch_right != RC_Switch::MIDDLE) {
                    *mode_ = chassis_mode::follow;
                }
            } else if (RC_->switch_right == RC_Switch::UP) {
                *mode_ = chassis_mode::spin;
            } else {
                reset_all_controls();
            }

            switch (*mode_) {
            case chassis_mode::follow:

                set_states_desire(
                    RC_->joystick_right.x() * x_velocity_scale,
                    RC_->joystick_right.y() * spinning_velocity);
                set_length_desire(RC_->joystick_left.x());

                break;
            case chassis_mode::spin: set_states_desire(0, spinning_velocity); break;
            case chassis_mode::balanceless:
                set_states_desire(RC_->joystick_right.x(), RC_->joystick_right.y());
                set_length_desire(RC_->joystick_left.x());
                set_angle_desire(RC_->joystick_left.y());
                break;
            default: break;
            }

            // if (RC_->keyboard.f) {
            //     desires.leg_length = 0.18;
            // } else if (RC_->keyboard.g) {
            //     desires.leg_length = 0.25;
            // }
        } while (false);
        last_switch_right = RC_->switch_right;
        if (observer_->status_levitate_)
            reset_persistent_data();
    }
    void Init(module::IMU_output_vector* IMU_output, device::RC_status* RC, chassis_mode* mode) {
        IMU_data_ = IMU_output;
        RC_       = RC;
        mode_     = mode;
    }
    /*检测到有电机can信号丢失回调这个*/
    void CanLost() {                                                  /* reset_all_controls();*/
    }

    desire desires = {};

private:
    DesireSet()                                            = default; // 禁止外部构造
    ~DesireSet()                                           = default; // 禁止外部析构
    DesireSet(const DesireSet& DesireSet)                  = delete;  // 禁止外部拷贝构造
    const DesireSet& operator=(const DesireSet& DesireSet) = delete;  // 禁止外部赋值操作

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    // rclcpp::TimerBase::SharedPtr rc_watchdog_timer_;

    const device::RC_status* RC_               = nullptr;
    const module::IMU_output_vector* IMU_data_ = nullptr;
    chassis_mode* mode_                        = nullptr;
    observer::observer* observer_              = observer::observer::GetInstance();

    device::RC_Switch last_switch_right = {};
    bool balanceless_mode_              = false;
    /*传入前进速度和YAW旋转速度，单位m/s,rad/s*/
    void set_states_desire(double x_velocity, double rotation_velocity = 0.0) {
        auto x_d_ref              = x_velocity;
        x_d_ref                   = std::clamp(x_d_ref, -x_velocity_scale, x_velocity_scale);
        static uint32_t last_time = 0;
        auto dt                   = DWT_GetDeltaT64_Expect(&last_time, app::dt);

        desires.xd(0, 0) += x_d_ref * dt; // distance
        if (*mode_ == chassis_mode::balanceless)
            desires.xd(1, 0) = x_d_ref;
        else
            desires.xd(1, 0) = 0;         // velocity

        switch (*mode_) {                 // yaw
        case chassis_mode::follow:
        case chassis_mode::balanceless:
            desires.xd(2, 0) = IMU_data_->Yaw_multi_turn - std::numbers::pi;
            break;
        case chassis_mode::spin: desires.xd(2, 0) += rotation_velocity * dt; break;
        default: break;
        }
        for (uint8_t i = 3; i < 10; i++) {
            desires.xd(i, 0) = 0;
        }
    }
    void set_length_desire(double length_speed) {
        constexpr double length_speed_scale = 0.0003;
        desires.leg_length += length_speed * length_speed_scale;
        desires.leg_length = std::clamp(desires.leg_length, 0.12, 0.28);
    }
    void set_angle_desire(double angle_speed) {
        constexpr double angle_speed_scale = 0.0003;
        desires.leg_angle += angle_speed * angle_speed_scale;
        // if (desires.leg_angle>PI){
        //     desires.leg_angle-=2*PI;
        // } else if (desires.leg_angle<-PI){
        //     desires.leg_angle += 2 * PI;
        // }
    }
    void set_roll_desire() {
        // constexpr double roll_scale = 0.5;
        // auto roll_control           = *thumb_wheel_ > 0.05 ? *thumb_wheel_ : 0.0;
        // *roldesire.leg_lentgh               = roll_control * roll_scale;
        desires.roll = 0.0;
    }
    void reset_all_controls() {
        desires.xd.setZero();
        desires.roll       = 0;
        desires.leg_length = 0.12;
        *mode_             = chassis_mode::stop;
    }
    void reset_persistent_data() { desires.xd(0, 0) = 0; }
};

} // namespace app::controller
