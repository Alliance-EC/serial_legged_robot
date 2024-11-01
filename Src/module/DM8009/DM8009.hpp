#pragma once
#include "app/system_parameters.hpp"
#include "device/Dji_motor/DJI_motor.hpp"
#include "tool/pid/pid.hpp"
#include <numbers>

using namespace device;
namespace module {
enum class DM8009_mode : uint8_t { angle, velocity, torque };
struct DM8009_params {
    tool::PID_params angle_pid_params = {};
    tool::PID_params speed_pid_params = {};
    DjiMotor_params Dji_common;
    DM8009_params() {
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
class DM8009 : public DjiMotor {
public:
    explicit DM8009(const DM8009_params& params)
        : DjiMotor(params.Dji_common)
        , angle_pid_(params.angle_pid_params)
        , speed_pid_(params.speed_pid_params) {}
    ~DM8009() = default;
    void configure(int encoder_zero_point = 0, bool reversed = false) {
        auto config = DjiMotorConfig(DjiMotorType::DM8009)
                          .set_encoder_zero_point(encoder_zero_point)
                          .enable_multi_turn_angle();
        if (reversed) {
            config.reverse();
        }
        DjiMotor::configure(config);
    }
    void SetAngle(double angle) {
        angle_target_ = angle;
        mode_         = DM8009_mode::angle;
    }
    void SetSpeed(double speed) {
        velocity_target_ = speed;
        mode_            = DM8009_mode::velocity;
    }
    void update() {
        switch (mode_) {
        case DM8009_mode::angle: {
            calculated_torque_ = speed_pid_.update(0, get_velocity())
                               + angle_pid_.update(angle_target_, get_angle());
            break;
        }
        case DM8009_mode::velocity:
            calculated_torque_ = speed_pid_.update(velocity_target_, get_velocity());
            break;
        default: break;
        }
    }
    double get_calculated_torque() {
        update();
        return calculated_torque_;
    }
    void set_mode(DM8009_mode mode) { mode_ = mode; }

    static constexpr double angle_offset = 0.35020031441266222960973855269724; // 上机械限位角度
    [[nodiscard]] double get_angle() const override {

        auto angle = DjiMotor::get_angle();
        switch (DjiMotor::get_index() + 1) {
        case 2:
        case 4: angle -= angle_offset; break;

        case 1:
        case 3: angle += angle_offset + std::numbers::pi; break;
        default: break;
        }
        return angle;
    }

private:
    double angle_target_      = 0.0f;
    double velocity_target_   = 0.0f;
    double calculated_torque_ = 0.0f;
    DM8009_mode mode_         = DM8009_mode::torque;
    tool::PID angle_pid_;
    tool::PID speed_pid_;
};
} // namespace module