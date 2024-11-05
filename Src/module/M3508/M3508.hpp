#pragma once
#include "app/system_parameters.hpp"
#include "device/Dji_motor/DJI_motor.hpp"
#include "tool/pid/pid.hpp"
#include <numbers>
using namespace device;
namespace module {
enum class M3508_mode : uint8_t { angle, velocity, torque };
struct M3508_params {
    tool::PID_params angle_pid_params = {};
    tool::PID_params speed_pid_params = {};
    DjiMotor_params Dji_common;
    M3508_params() {
        this->angle_pid_params = {
            .Kp            = 20.0f,
            .Ki            = 0.0f,
            .Kd            = 0.0f,
            .MaxOut        = 40.0f,
            .IntegralLimit = 0.0f,
            .Expect_dt     = app::dt};
        this->speed_pid_params = {
            .Kp            = 5.0f,
            .Ki            = 0.0f,
            .Kd            = 0.0f,
            .MaxOut        = 40.0f,
            .IntegralLimit = 0.0f,
            .Expect_dt     = app::dt};
    }
};  
class M3508 : public DjiMotor {
public:
    explicit M3508(const M3508_params& params)
        : DjiMotor(params.Dji_common)
        , angle_pid_(params.angle_pid_params)
        , speed_pid_(params.speed_pid_params) {}
    ~M3508() = default;
    void configure(bool reversed = false, DjiMotorType m3508_type = DjiMotorType::M3508_15) {
        auto config = DjiMotorConfig(m3508_type).enable_multi_turn_angle();
        if (reversed) {
            config.reverse();
        }
        DjiMotor::configure(config);
    }
    void SetAngle(double angle) {
        angle_target_ = angle;
        mode_         = M3508_mode::angle;
    }
    void SetSpeed(double speed) {
        velocity_target_ = speed;
        mode_            = M3508_mode::velocity;
    }
    void update() {
        switch (mode_) {
        case M3508_mode::angle: {
            calculated_torque_ = speed_pid_.update(0, get_velocity())
                               + angle_pid_.update(angle_target_, get_angle());
            break;
        }
        case M3508_mode::velocity:
            calculated_torque_ = speed_pid_.update(velocity_target_, get_velocity());
            break;
        default: break;
        }
    }
    double get_calculated_torque() {
        update();
        return calculated_torque_;
    }
    void set_mode(M3508_mode mode) { mode_ = mode; } 

    // static constexpr double angle_offset = 0.3; // 上机械限位角度
    [[nodiscard]] double get_angle() const override {
        /*对于单编码器关节电机，角度偏移包括上电启动时*/
        auto angle = DjiMotor::get_angle();
        switch (DjiMotor::get_index() + 1) {
        case 1: angle = std::numbers::pi / 2 - angle;break;
        case 2: angle = 4.06 - angle;break;
        case 3: angle += std::numbers::pi / 2; break;
        case 4: angle += 4.06; break;
        default: break;
        }
        return angle;
    }

private:
    double angle_target_      = 0.0f;
    double velocity_target_   = 0.0f;
    double calculated_torque_ = 0.0f;
    M3508_mode mode_          = M3508_mode::torque;
    tool::PID angle_pid_;
    tool::PID speed_pid_;
};
} // namespace module