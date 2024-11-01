#pragma once
#include "app/system_parameters.hpp"
#include "bsp/dwt/dwt.h"
#include "leg_conv_reverse.hpp"
#include "leg_pos.hpp"
#include "module/IMU/IMU.hpp"
#include "module/M3508/M3508.hpp"
#include "tool/daemon/daemon.hpp"
#include "tool/filter//low_pass_filter.hpp"
#include "tool/filter/OLS.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>

double watch_data[6];
double increase_flag_ = 0;
extern double reverse;
namespace app::observer {
struct leg_length {
    double L;
    double Ld;
    double R;
    double Rd;
};
struct support_force {
    double L;
    double R;
};
class observer {
public:
    static observer* GetInstance() {
        static auto instance = new observer();
        return instance;
    }
    void update() {
        dt_ = DWT_GetDeltaT64_Expect(&dwt_count_, app::dt);
        IMU_update();
        leg_update();
        wheel_update();
        support_force_update();
        levitate_detect();

        if ((*chassis_mode_ == chassis_mode::stop)
            || (*chassis_mode_ == chassis_mode::balanceless)) {
            reset_persistent_data();
            status_levitate_ = false;
        }
        auto phi = IMU_->output_vector.Yaw_multi_turn;
        x_states_ << distance, // 1 x
            velocity_,         // 2 x_d
            phi,               // 3 phi
            imu_gyro_->z(),    // 4 phid
            angle_L_,          // 5 theta_ll
            angle_Ld_,         // 6 theta_lld
            angle_R_,          // 7 theta_lr
            angle_Rd_,         // 8 theta_lrd
            imu_euler_->y(),   // 9 theta_b
            imu_gyro_->y();    // 10 theta_bd
    }
    void Init(module::IMU* IMU, std::array<module::M3508*, 6> M3508, chassis_mode* chassis_mode) {
        IMU_          = IMU;
        M3508_        = M3508;
        chassis_mode_ = chassis_mode;
        imu_accel_    = &IMU_->output_vector.accel;
        imu_gyro_     = &IMU_->output_vector.gyro;
        imu_euler_    = &IMU_->output_vector.euler_angle;
        // y轴反向
    }
    // output variables
    Eigen::Matrix<double, 10, 1> x_states_;
    leg_length leg_length_;
    support_force support_force_;
    bool status_levitate_;
    double roll_, leg_length_avg_;

private:
    observer()                                          = default; // 禁止外部构造
    ~observer()                                         = default; // 禁止外部析构
    observer(const observer& observer)                  = delete;  // 禁止外部拷贝构造
    const observer& operator=(const observer& observer) = delete;  // 禁止外部赋值操作

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double angle_L_ = 0, angle_Ld_ = 0;
    double angle_R_ = 0, angle_Rd_ = 0;
    double velocity_ = 0.0, distance = 0.0;

    module::IMU* IMU_                    = nullptr;
    std::array<module::M3508*, 6> M3508_ = {};
    const chassis_mode* chassis_mode_    = nullptr; // 设为指针以便在外部修改
    tool::daemon levitate_allow_timer_ =
        tool::daemon(1, std::bind(&observer::reset_levitate_allowance, this));

    uint32_t dwt_count_               = 0;
    double dt_                        = 0.0;
    bool allow_levitate_              = true;
    const Eigen::Vector3f *imu_accel_ = nullptr, *imu_gyro_ = nullptr, *imu_euler_ = nullptr;

    tool::filter::LowPassFilter angle_L_LPF_  = tool::filter::LowPassFilter(100);
    tool::filter::LowPassFilter angle_R_LPF_  = tool::filter::LowPassFilter(100);
    tool::filter::LowPassFilter length_L_LPF_ = tool::filter::LowPassFilter(50);
    tool::filter::LowPassFilter length_R_LPF_ = tool::filter::LowPassFilter(50);
    tool::filter::OLS velocity_OLS_           = tool::filter::OLS(50);

    void IMU_update() {
        IMU_->Acquire();
        roll_ = imu_euler_->x();
    }
    void leg_update() {
        static double last_angle_L  = 0.0;
        static double last_angle_R  = 0.0;
        static double last_length_L = 0.0;
        static double last_length_R = 0.0;
        double data[2];
        /*左腿虚拟角度和长度*/
        leg_pos(M3508_[leg_LF]->get_angle(), M3508_[leg_LB]->get_angle(), data);
        watch_data[2]=data[0];
        watch_data[3] = data[1];

        leg_length_.L = data[0];                                // ll
        leg_length_.L = length_L_LPF_.update(leg_length_.L);

        angle_L_ = -data[1] + imu_euler_->y();                  // 5 theta_ll

        watch_data[4] = angle_L_;
        watch_data[5] = last_angle_L;
        if (angle_L_ < -3.13 && last_angle_L > 3.13) {
            increase_flag_ = 1;
        } else if (angle_L_ > 3.13 && last_angle_L < -3.13) {
            increase_flag_ = -1;
        }

        angle_L_ = angle_L_LPF_.update(angle_L_);
        angle_Ld_ = (angle_L_ - last_angle_L) / dt_;            // 6 theta_lld
        last_angle_L = angle_L_;

        leg_length_.Ld = (leg_length_.L - last_length_L) / dt_; // lld
        last_length_L  = leg_length_.L;

        watch_data[0] = leg_length_.L;
        /*右腿虚拟腿角度和长度*/
        leg_pos(M3508_[leg_RB]->get_angle(), M3508_[leg_RF]->get_angle(), data);

        leg_length_.R = data[0];                                // lr
        leg_length_.R = length_R_LPF_.update(leg_length_.R);

        angle_R_ = -data[1] + imu_euler_->y();                  // 7 theta_lr
        angle_R_ = angle_R_LPF_.update(angle_R_);

        angle_Rd_    = (angle_R_ - last_angle_R) / dt_;         // 8 theta_lrd
        last_angle_R = angle_R_;

        leg_length_.Rd = (leg_length_.R - last_length_R) / dt_; // lrd
        last_length_R  = leg_length_.R;

        leg_length_avg_ = (leg_length_.L + leg_length_.R) / 2.0f;

        watch_data[1] = angle_L_;
    }

    void wheel_update() {
        velocity_ = (M3508_[wheel_L]->get_velocity() + M3508_[wheel_R]->get_velocity()) * Rw / 2.0f;
        velocity_ = velocity_OLS_.Smooth(dt_, velocity_);
        distance  = (M3508_[wheel_L]->get_angle() + M3508_[wheel_R]->get_angle()) * Rw / 2.0f;
    }
    void support_force_update() {
        double data[2];
        leg_conv_reverse(
            M3508_[leg_LF]->get_torque(), M3508_[leg_LB]->get_torque(), M3508_[leg_LF]->get_angle(),
            M3508_[leg_LB]->get_angle(), data);
        auto F_l         = data[0];
        auto T_l         = data[1];
        auto P_l         = F_l * std::cos(angle_L_) + T_l * std::sin(angle_L_) / leg_length_.L;
        support_force_.L = P_l;
        leg_conv_reverse(
            M3508_[leg_RF]->get_torque(), M3508_[leg_RB]->get_torque(), M3508_[leg_RF]->get_angle(),
            M3508_[leg_RB]->get_angle(), data);
        auto F_r         = data[0];
        auto T_r         = data[1];
        auto P_r         = F_r * std::cos(angle_R_) + T_r * std::sin(angle_R_) / leg_length_.R;
        support_force_.R = P_r;
    }
    void levitate_detect() {
        constexpr double force_levitate = 20.0;
        constexpr double force_normal   = 50.0;
        auto support_force_avg          = (support_force_.L + support_force_.R) / 2.0;
        if ((support_force_avg < force_levitate) && allow_levitate_
            && (*chassis_mode_ != chassis_mode::stop)
            && (*chassis_mode_ != chassis_mode::balanceless) && (leg_length_avg_ > 0.25)) {
            status_levitate_ = true;
        } else if (
            (support_force_avg > force_normal) || *chassis_mode_ != chassis_mode::stop
            || *chassis_mode_ != chassis_mode::balanceless) {
            if (status_levitate_ == true) {
                allow_levitate_ = false;
                levitate_allow_timer_.reload();
            }
            status_levitate_ = false;
        }

        if (status_levitate_)
            reset_persistent_data();
    }
    /*轮电机多圈里程清零*/
    void reset_persistent_data() {
        distance = 0;
        M3508_[wheel_L]->reset_angle();
        M3508_[wheel_R]->reset_angle();
    }
    void reset_levitate_allowance() { allow_levitate_ = true; }
};

} // namespace app::observer
