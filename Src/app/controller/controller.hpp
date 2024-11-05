#pragma once
#include "LQR_k.hpp"
#include "LegConv.hpp"
#include "app/observer/observer.hpp"
#include "app/system_parameters.hpp"
#include "desire_set.hpp"
#include "device/super_cap/super_cap.hpp"
#include "module/referee/status.hpp"
#include "speed_hat.hpp"
#include "tool/pid/pid.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

double watch_data_c[15];
double watch_data_desire[15];
double over[2]={};
namespace app::controller {

using namespace tool;
struct control_torque {
    double wheel_L; // velocity target when in balanceless mode
    double wheel_R; // velocity target when in balanceless mode
    double leg_LF;
    double leg_LB;
    double leg_RF;
    double leg_RB;
};
class Controller {
public:
    static Controller* GetInstance() {
        static auto instance = new Controller();
        return instance;
    }

    void update() {
        do {
            if (*mode_ == chassis_mode::balanceless) {
                velocity_control((*xd_)(1, 0) * 0.5, (*xd_)(2, 0) - (*x_states_)(2, 0));
                leg_controller_balanceless();
                break;
            } else if (*mode_ == chassis_mode::stop) {
                stop_all_control();
                break;
            }
            kinematic_controller();
            leg_controller();
            // wheel_model_hat();
            // leg_coordinator();
            torque_process();
        } while (false);
    }
    void Init(module::IMU* IMU, std::array<module::M3508*, 6> M3508, chassis_mode* chassis_mode) {
        IMU_      = IMU;
        M3508_    = M3508;
        mode_     = chassis_mode;
        imu_euler = &IMU_->output_vector.euler_angle;
        imu_gyro  = &IMU_->output_vector.gyro;
    }

    control_torque control_torque_ = {};

private:
    Controller()                                              = default; // 禁止外部构造
    ~Controller()                                             = default; // 禁止外部析构
    Controller(const Controller& Controller)                  = delete;  // 禁止外部拷贝构造
    const Controller& operator=(const Controller& Controller) = delete;  // 禁止外部赋值操作

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double F_l = 0, F_r = 0;
    double T_lwl_ = 0, T_lwr_ = 0, T_bll_ = 0, T_blr_ = 0;
    double T_lwl_compensate_ = 0, T_lwr_compensate_ = 0;
    double T_bll_compensate_ = 0, T_blr_compensate_ = 0;

    double wheel_compensate_kp_ = 0.0;
    PID pid_roll_               = PID({50, 0.0, 0, 50, 0.0, 0.0, dt});
    PID pid_length_             = PID({300, 0, 0, 50, 100.0, 0.0, dt});
    /*非平衡状态腿控参数*/
    PID pid_length_l_ = PID({300, 0, 1, 50, 40.0, 0.0, dt});

    PID pid_length_r_ = PID({300, 0, 1, 50, 40.0, 0.0, dt});

    PID pid_angle_l_  = PID({15, 0, 0, 10, 5.0, 0.0, dt});

    PID pid_angle_r_  = PID({15, 0, 0, 10, 5.0, 0.0, dt});

    observer::observer* observer_        = observer::observer::GetInstance();
    DesireSet* desire_                   = controller::DesireSet::GetInstance();
    std::array<module::M3508*, 6> M3508_ = {};
    module::IMU* IMU_                    = nullptr;

    const Eigen::Matrix<double, 10, 1>* xd_       = &desire_->desires.xd;
    const Eigen::Matrix<double, 10, 1>* x_states_ = &observer_->x_states_;
    const observer::leg_length* leg_length_ =&observer_->leg_length_;
    const double* length_desire_                  = &desire_->desires.leg_length;
    const double* angle_desire_                   = &desire_->desires.leg_angle;
    const double* roll_desire_ = &desire_->desires.roll;
    //
    // const double* leg_angle_desire_                    = &desire_->desires.roll;
    const Eigen::Vector3f *imu_euler = nullptr, *imu_gyro = nullptr;
    const chassis_mode* mode_ = nullptr;

    void kinematic_controller() {
        double lqr_k[40];
        LQR_k(leg_length_->L, leg_length_->R, lqr_k);
        Eigen::Map<Eigen::Matrix<double, 4, 10, Eigen::ColMajor>> LQR_gain(lqr_k);

        Eigen::Matrix<double, 10, 1> e_mat = -1.0 * (*xd_ - *x_states_);
        if (observer_->status_levitate_) { // 腾空状态仅保持腿部竖直
            e_mat(0, 0) = 0;
            e_mat(1, 0) = 0;
            e_mat(2, 0) = 0;
            e_mat(3, 0) = 0;

            e_mat(8, 0) = 0;
            e_mat(9, 0) = 0;
        }
        auto u_mat = LQR_gain * e_mat;
        T_lwl_     = u_mat(0, 0);
        T_lwr_     = u_mat(1, 0);
        T_bll_     = u_mat(2, 0);
        T_blr_     = u_mat(3, 0);
    }

    void leg_controller() {
        auto length = (leg_length_->L + leg_length_->R) / 2.0;
        /*重力前馈和侧向力前馈*/
        auto gravity_ff  = [=]() { return (m_b / 2.0 + eta_l * m_l) * g; };
        auto inertial_ff = [=, this]() {
        auto coefficient = length / (2 * R_l) * (*x_states_)(3, 0) * (*x_states_)(1, 0);
            // return (m_b / 2.0 + eta_l * m_l) * coefficient;
            return 0;
        };

        auto length_desire = observer_->status_levitate_ ? 0.3 : *length_desire_;
        auto angle_desire = observer_->status_levitate_? 0.0 : *angle_desire_;
        auto roll_desire   = observer_->status_levitate_ ? 0.0 : *roll_desire_;

        /*roll和腿长PID运算*/
        auto F_length = pid_length_.update(length_desire, length);
        auto F_roll   = pid_roll_.update(roll_desire, imu_euler->x(), imu_gyro->x());

        F_l = F_length + F_roll + gravity_ff() - inertial_ff();
        F_r = F_length - F_roll + gravity_ff() + inertial_ff();
        watch_data_desire[0] = F_length;
        watch_data_desire[1] = F_roll;
        watch_data_desire[2] = F_l;
        watch_data_desire[3] = F_r;
        watch_data_desire[4] = length_desire;
        watch_data_desire[5] = roll_desire;
    }
    void leg_controller_balanceless() {
        constexpr double LEG_MOTOR_T_MAX = 10.0f;
        F_l          = pid_length_l_.update(*length_desire_, leg_length_->L);
        F_r          = pid_length_r_.update(*length_desire_, leg_length_->R);
        T_bll_       = pid_angle_l_.update(*angle_desire_,(*x_states_)(4, 0)+over[0]);
        T_blr_       = pid_angle_r_.update(*angle_desire_,(*x_states_)(6, 0)+over[1]);

        // // watch_data_c[0] = *length_desire_;
        // // watch_data_c[1] = over[1];

        // watch_data_c[2] = (*angle_desire_);
        // watch_data_c[3] = (*x_states_)(6, 0) + over[1];
        // watch_data_c[4] = (*x_states_)(6, 0);
        /*VMC解算到关节电机扭矩*/
        static double leg_T[2] = {};
        leg_conv(-F_l, -T_bll_, M3508_[leg_LB]->get_angle(), M3508_[leg_LF]->get_angle(), leg_T);
        control_torque_.leg_LB = std::clamp(leg_T[0], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        control_torque_.leg_LF = std::clamp(leg_T[1], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        watch_data_c[9]        = leg_T[0];
        watch_data_c[10]       = leg_T[1];
        leg_conv(F_r, T_blr_, M3508_[leg_RB]->get_angle(), M3508_[leg_RF]->get_angle(), leg_T);
        control_torque_.leg_RB = std::clamp(leg_T[0], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        control_torque_.leg_RF = std::clamp(leg_T[1], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        watch_data_c[5] = F_l;
        watch_data_c[6] = F_r;
        watch_data_c[7] = T_bll_;
        watch_data_c[8] = T_blr_;
        watch_data_c[11]        = leg_T[0];
        watch_data_c[12]       = leg_T[1];
        //过圈计算
        for (int i=0;i<2;i++){
            if (observer_->get_increase_flag(i) == 1) {
                over[i] +=2*PI;
            } else if (observer_->get_increase_flag(i) == -1) {
                over[i] -= 2*PI;
            }
        }
        observer_->set_increase_flag(0,0);
        observer_->set_increase_flag(1,0);
    }
    void wheel_model_hat() {
        double wheel_speed_hat[2];
        constexpr double predict_dt = 0.001; // 预测之后多少时间的值
        speed_hat(
            T_bll_, T_blr_, T_lwl_, T_lwr_, predict_dt, leg_length_->L, leg_length_->R,
            (*x_states_)(3, 0), (*x_states_)(1, 0), (*x_states_)(4, 0), (*x_states_)(6, 0),
            wheel_speed_hat);

        T_lwl_compensate_ =
            wheel_compensate_kp_ * (wheel_speed_hat[0] - M3508_[wheel_L]->get_velocity());
        T_lwr_compensate_ =
            wheel_compensate_kp_ * (wheel_speed_hat[1] - M3508_[wheel_R]->get_velocity());
    }
    void torque_process() {
        constexpr double LEG_MOTOR_T_MAX = 6.0f;
        constexpr double LEG_T_MAX       = 3.0f;

        // T_lwl_ += T_lwl_compensate_;
        // T_lwr_ += T_lwr_compensate_;
        // T_bll_ += T_bll_compensate_;
        // T_blr_ += T_blr_compensate_;

        constexpr double max_torque_wheel = 1.0f;
        control_torque_.wheel_L           = std::clamp(T_lwl_, -max_torque_wheel, max_torque_wheel);
        control_torque_.wheel_R           = std::clamp(T_lwr_, -max_torque_wheel, max_torque_wheel);
        if (*mode_ != chassis_mode::balanceless) {
            control_torque_.wheel_L *= -1;
            control_torque_.wheel_R *= -1;
        }
        /*虚拟腿扭矩限幅，防止受大冲击时大幅度震荡*/
        watch_data_c[2]=T_bll_;
        watch_data_c[3] =T_blr_;
        T_bll_ = std::clamp(T_bll_, -LEG_T_MAX, LEG_T_MAX);
        T_blr_ = std::clamp(T_blr_, -LEG_T_MAX, LEG_T_MAX);
        /*VMC解算到关节电机扭矩*/
        double leg_T[2];
        leg_conv(-F_l, -T_bll_, M3508_[leg_LF]->get_angle(), M3508_[leg_LB]->get_angle(), leg_T);
        control_torque_.leg_LF = std::clamp(leg_T[0], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        control_torque_.leg_LB = std::clamp(leg_T[1], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        leg_conv(F_r, T_blr_, M3508_[leg_RF]->get_angle(), M3508_[leg_RB]->get_angle(), leg_T);
        control_torque_.leg_RF = std::clamp(leg_T[0], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        control_torque_.leg_RB = std::clamp(leg_T[1], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
    }

    void stop_all_control() {
        control_torque_.wheel_L = nan;
        control_torque_.wheel_R = nan;
        control_torque_.leg_LF  = nan;
        control_torque_.leg_LB  = nan;
        control_torque_.leg_RF  = nan;
        control_torque_.leg_RB  = nan;
    }
    void velocity_control(double x_speed, double angle_error = 0.0) {
        control_torque_.leg_LF = nan;
        control_torque_.leg_LB = nan;
        control_torque_.leg_RF = nan;
        control_torque_.leg_RB = nan;

        double rotation_velocity = std::clamp(10 * angle_error, -3.0, 3.0);
        control_torque_.wheel_L  = static_cast<float>((x_speed - rotation_velocity * R_l) / Rw);
        control_torque_.wheel_R  = static_cast<float>((x_speed + rotation_velocity * R_l) / Rw);
    }
};
} // namespace app::controller