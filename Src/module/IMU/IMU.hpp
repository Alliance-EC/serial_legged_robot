#pragma once
#include "AHRS.h"
#include "app/system_parameters.hpp"
#include "bsp/dwt/dwt.h"
#include "device/BMI088/BMI088.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <utility>

namespace module {
struct IMU_output_vector {
    Eigen::Vector3f gyro        = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f accel       = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f euler_angle = {0.0f, 0.0f, 0.0f};
    double Yaw_multi_turn       = 0.0;
};
struct IMU_params {
    device::BMI088_params bmi088_params = {};
    Eigen::Vector3f angle_offset        = {0.0f, 0.0f, 0.0f};
    std::function<void()> callback      = nullptr;

    explicit IMU_params(app::board board) {
        this->bmi088_params = {
            .work_mode = device::BMI088_Work_Mode::BLOCK_PERIODIC_MODE, // 用任务定时读取比较稳定
            .cali_mode = device::BMI088_Calibrate_Mode::LOAD_PRE_CALI_MODE,
        };
        switch (board) {
        case app::board::UNKNOWN:
        case app::board::DM_MC02:
            this->bmi088_params.spi_gyro_params = {
                .spi_handle = &hspi2,
                .GPIOx      = GPIOC,
                .cs_pin     = GPIO_PIN_3,
            };
            this->bmi088_params.spi_acc_params = {
                .spi_handle = &hspi2,
                .GPIOx      = GPIOC,
                .cs_pin     = GPIO_PIN_0,
            };
            this->bmi088_params.gyro_int_params = {.GPIOx = GPIOE, .GPIO_Pin = GPIO_PIN_12};
            this->bmi088_params.acc_int_params  = {.GPIOx = GPIOE, .GPIO_Pin = GPIO_PIN_10};
            this->bmi088_params.heat_pwm_params = {
                .htim      = &htim3,
                .channel   = HAL_TIM_ACTIVE_CHANNEL_4,
                .period    = 0.001f,
                .dutyratio = 0,
            };
            break;
        default: break;
        }
        this->bmi088_params.heat_pid_params = {
            .Kp            = 0.32f,
            .Ki            = 0.0004f,
            .Kd            = 0,
            .MaxOut        = 0.95f,
            .IntegralLimit = 0.90f,
        };
    }
    IMU_params& set_work_mode(device::BMI088_Work_Mode mode) {
        return bmi088_params.work_mode = mode, *this;
    }
    IMU_params& set_cali_mode(device::BMI088_Calibrate_Mode mode) {
        return bmi088_params.cali_mode = mode, *this;
    }
    IMU_params& set_angle_offset(Eigen::Vector3f offset) {
        return this->angle_offset = std::move(offset), *this;
    }
};
struct IMU_data {
    float gyro[3]  = {};
    float accel[3] = {};
    float mag[3]   = {};
    float quat[4]  = {};
};
struct IMU_output {
    float euler_angle[3]  = {};                                         // 弧度制欧拉角
    float Yaw_total_angle = 0;                                          // 云台总偏转角度
};
class IMU {
public:
    explicit IMU(const IMU_params& params)
        : bmi088_(params.bmi088_params)
        , callback_(params.callback)
        , angle_offset(params.angle_offset) {
        bmi088_.SetCallback(std::bind(&IMU::update, this));
        gyro_offset = bmi088_.GetGyroOffset();

        device::BMI088_Data raw_data;
        bmi088_.Acquire(&raw_data);
        cali_slove(&raw_data);
        AHRS_init(INS_data_.quat, INS_data_.accel, INS_data_.mag);

        accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_data_.accel[0];
        accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_data_.accel[1];
        accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_data_.accel[2];
    }
    bool Acquire(IMU_output* output = nullptr) {
        if (bmi088_.GetWorkMode() == device::BMI088_Work_Mode::BLOCK_PERIODIC_MODE) {
            update();
        }
        if (output != nullptr && AHRS_ready_) {
            for (uint8_t i = 0; i < 3; i++)
                output->euler_angle[i] = INS_output_.euler_angle[i];
            output->Yaw_total_angle = INS_output_.Yaw_total_angle;
        }
        return AHRS_ready_;
    }
    [[nodiscard]] Eigen::Vector3f get_gyro() const {
        return {INS_data_.gyro[0], INS_data_.gyro[1], INS_data_.gyro[2]};
    }
    [[nodiscard]] Eigen::Vector3f get_accel() const {
        return {INS_data_.accel[0], INS_data_.accel[1], INS_data_.accel[2]};
    }
    [[nodiscard]] Eigen::Vector3f get_euler_angle() const {
        return {INS_output_.euler_angle[0], INS_output_.euler_angle[1], INS_output_.euler_angle[2]};
    }
    [[nodiscard]] float get_yaw() const { return INS_output_.euler_angle[INS_YAW_ADDRESS_OFFSET]; }
    [[nodiscard]] float get_pitch() const {
        return INS_output_.euler_angle[INS_PITCH_ADDRESS_OFFSET];
    }
    [[nodiscard]] float get_roll() const {
        return INS_output_.euler_angle[INS_ROLL_ADDRESS_OFFSET];
    }
    [[nodiscard]] float get_yaw_total() const { return INS_output_.Yaw_total_angle; }

    IMU_output_vector output_vector;

private:
    float timing_time; // 任务运行的时间 单位 s
    float accel_fliter_1[3]              = {0.0f, 0.0f, 0.0f};
    float accel_fliter_2[3]              = {0.0f, 0.0f, 0.0f};
    float accel_fliter_3[3]              = {0.0f, 0.0f, 0.0f};
    static constexpr float fliter_num[3] = {
        1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

    Eigen::Vector3f gyro_offset  = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f accel_offset = {0.0f, 0.0f, 0.0f};

// Fucking shit ，waiting C++ Matrix
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, { 0.0f, 0.0f, 1.0f }

    float gyro_scale_factor[3][3]  = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
    float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};

    static constexpr uint8_t INS_YAW_ADDRESS_OFFSET = 2; // 陀螺仪数据相较于云台的yaw的方向
    static constexpr uint8_t INS_PITCH_ADDRESS_OFFSET = 1; // 陀螺仪数据相较于云台的pitch的方向
    static constexpr uint8_t INS_ROLL_ADDRESS_OFFSET = 0; // 陀螺仪数据相较于云台的roll的方向

    IMU_data INS_data_ = {};
    device::bmi088 bmi088_;
    std::function<void()> callback_ = nullptr;
    Eigen::Vector3f angle_offset    = {0.0f, 0.0f, 0.0f};
    IMU_output INS_output_;
    bool AHRS_ready_ = false;

    //@brief 旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
    void cali_slove(device::BMI088_Data* data) {
        for (uint8_t i = 0; i < 3; i++) {
            INS_data_.gyro[i] = data->gyro[0] * gyro_scale_factor[i][0]
                              + data->gyro[1] * gyro_scale_factor[i][1]
                              + data->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
            INS_data_.accel[i] = data->acc[0] * accel_scale_factor[i][0]
                               + data->acc[1] * accel_scale_factor[i][1]
                               + data->acc[2] * accel_scale_factor[i][2] + accel_offset[i];
            INS_data_.mag[i] = 0; // 不使用磁力计
        }
    }
    void update() {
        device::BMI088_Data raw_data;
        if (bmi088_.Acquire(&raw_data)) {
            cali_slove(&raw_data);
            // 加速度计低通滤波
            accel_fliter_1[0] = accel_fliter_2[0];
            accel_fliter_2[0] = accel_fliter_3[0];

            accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0]
                              + accel_fliter_1[0] * fliter_num[1]
                              + INS_data_.accel[0] * fliter_num[2];

            accel_fliter_1[1] = accel_fliter_2[1];
            accel_fliter_2[1] = accel_fliter_3[1];

            accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0]
                              + accel_fliter_1[1] * fliter_num[1]
                              + INS_data_.accel[1] * fliter_num[2];

            accel_fliter_1[2] = accel_fliter_2[2];
            accel_fliter_2[2] = accel_fliter_3[2];

            accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0]
                              + accel_fliter_1[2] * fliter_num[1]
                              + INS_data_.accel[2] * fliter_num[2];

            timing_time = DWT_GetDeltaT(&bmi088_.bias_dwt_cnt);
            AHRS_update(INS_data_.quat, timing_time, INS_data_.gyro, accel_fliter_3, INS_data_.mag);
            get_angle(
                INS_data_.quat, &INS_output_.euler_angle[INS_YAW_ADDRESS_OFFSET],
                &INS_output_.euler_angle[INS_PITCH_ADDRESS_OFFSET],
                &INS_output_.euler_angle[INS_ROLL_ADDRESS_OFFSET]);
            for (uint8_t i = 0; i < 3; i++) {
                INS_output_.euler_angle[i] += angle_offset[i];
            }

            // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
            static float last_yaw_angle    = 0; // 上一次的yaw角度
            static int16_t yaw_round_count = 0; // yaw转过的圈数
            if (INS_output_.euler_angle[INS_YAW_ADDRESS_OFFSET] - last_yaw_angle
                > std::numbers::pi) {
                yaw_round_count--;
            } else if (
                INS_output_.euler_angle[INS_YAW_ADDRESS_OFFSET] - last_yaw_angle
                < -std::numbers::pi) {
                yaw_round_count++;
            }
            INS_output_.Yaw_total_angle =
                INS_output_.euler_angle[INS_YAW_ADDRESS_OFFSET]
                + static_cast<float>(yaw_round_count) * 2.0f * std::numbers::pi_v<float>;
            last_yaw_angle               = INS_output_.euler_angle[INS_YAW_ADDRESS_OFFSET];
            output_vector.gyro           = get_gyro();
            output_vector.accel          = get_accel();
            output_vector.euler_angle    = get_euler_angle();
            output_vector.Yaw_multi_turn = INS_output_.Yaw_total_angle;
            AHRS_ready_                  = true;
        }
    }
};
} // namespace module