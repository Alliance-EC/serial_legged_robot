//@note 当前版本是c快速移植版本
#pragma once
#define USE_IMU_EKF
#include "QuaternionEKF.h"
#include "app/system_parameters.hpp"
#include "bsp/dwt/dwt.h"
#include "device/BMI088/BMI088.hpp"
#include "user_lib.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>

namespace module {
struct IMU_output_vector {
    Eigen::Vector3f gyro        = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f accel       = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f euler_angle = {0.0f, 0.0f, 0.0f};
    double Yaw_multi_turn       = 0.0;
};

struct IMU_params {
    device::BMI088_params bmi088_params             = {};
    std::function<void()> callback                  = nullptr;
    Eigen::Vector3f angle_offset                    = {0.0f, 0.0f, 0.0f};
    Eigen::Matrix<float, 3, 3> install_spin_matrix_ = Eigen::Matrix<float, 3, 3>::Identity();
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
        return angle_offset = std::move(offset), *this;
    }
    IMU_params& set_install_spin(Eigen::Matrix<float, 3, 3> install_spin_matrix) {
        return install_spin_matrix_ = std::move(install_spin_matrix), *this;
    }
};
// 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)
struct IMU_attitude {
    float Gyro[3]  = {}; // 角速度
    float Accel[3] = {}; // 加速度
    // 还需要增加角速度数据
    float Roll          = 0;
    float Pitch         = 0;
    float Yaw           = 0;
    float YawTotalAngle = 0;
};
class IMU {
public:
    explicit IMU(const IMU_params& params)
        : angle_offset_(params.angle_offset)
        , install_spin_matrix_(params.install_spin_matrix_)
        , bmi088_(params.bmi088_params)
        , callback_(params.callback) {
        bmi088_.SetCallback(std::bind(&IMU::update, this));
        gyro_offset_ = bmi088_.GetGyroOffset();

        float init_quaternion[4] = {0};
        InitQuaternion(init_quaternion);
        IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
        // noise of accel is relatively big and of high freq,thus lpf is used
        IMU_data_.AccelLPF = 0.0085;
    }
    bool Acquire(IMU_attitude* output = nullptr) {
        if (bmi088_.GetWorkMode() == device::BMI088_Work_Mode::BLOCK_PERIODIC_MODE) {
            update();
        }
        if (output != nullptr && AHRS_ready_) {
            for (uint8_t i = 0; i < 3; ++i) {
                output->Gyro[i]  = IMU_data_.Gyro[i];
                output->Accel[i] = IMU_data_.Accel[i];
            }
            output->Yaw           = IMU_data_.Yaw;
            output->Pitch         = IMU_data_.Pitch;
            output->Roll          = IMU_data_.Roll;
            output->YawTotalAngle = IMU_data_.YawTotalAngle;
        }
        return AHRS_ready_;
    }
    [[nodiscard]] Eigen::Vector3f get_gyro() const {
        return {IMU_data_.Gyro[0], IMU_data_.Gyro[1], IMU_data_.Gyro[2]};
    }
    [[nodiscard]] Eigen::Vector3f get_accel() const {
        return {IMU_data_.Accel[0], IMU_data_.Accel[1], IMU_data_.Accel[2]};
    }
    [[nodiscard]] Eigen::Vector3f get_euler_angle() const { // 四元数转换顺序为PRY
        return {IMU_data_.Pitch, IMU_data_.Roll, IMU_data_.Yaw};
    }
    [[nodiscard]] float get_yaw() const { return IMU_data_.Yaw; }
    [[nodiscard]] float get_pitch() const { return IMU_data_.Pitch; }
    [[nodiscard]] float get_roll() const { return IMU_data_.Roll; }
    [[nodiscard]] float get_yaw_total() const { return IMU_data_.YawTotalAngle; }

    IMU_output_vector output_vector;

private:
    struct IMU_data {
        float q[4] = {1, 0, 0, 0};                          // 四元数估计值

        float MotionAccel_b[3] = {};                        // 机体坐标加速度
        float MotionAccel_n[3] = {};                        // 绝对系加速度

        float AccelLPF = 0;                                 // 加速度低通滤波系数

        // bodyframe在绝对系的向量表示
        float xn[3] = {};
        float yn[3] = {};
        float zn[3] = {};

        // 加速度在机体系和XY两轴的夹角
        // float atanxz;
        // float atanyz;

        // IMU量测值
        float Gyro[3]  = {}; // 角速度
        float Accel[3] = {}; // 加速度
        // 位姿
        float Roll          = 0;
        float Pitch         = 0;
        float Yaw           = 0;
        float YawTotalAngle = 0;
    };
    struct IMU_correction {
        uint8_t flag   = 1;
        float scale[3] = {1, 1, 1};

        float Yaw   = 0;
        float Pitch = 0;
        float Roll  = 0;
    };

    float dt; // 任务运行的时间 单位 s

    static constexpr float xb[3] = {1, 0, 0};
    static constexpr float yb[3] = {0, 1, 0};
    static constexpr float zb[3] = {0, 0, 1};

    static constexpr uint8_t X = 0;
    static constexpr uint8_t Y = 1;
    static constexpr uint8_t Z = 2;

    static constexpr float gravity[3] = {0, 0, app::gravity};

    static constexpr float angle2rad = std::numbers::pi_v<float> / 180.0f;
    static constexpr float rad2angle = 180.0f / std::numbers::pi_v<float>;

    Eigen::Vector3f gyro_offset_  = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f accel_offset_ = {0.0f, 0.0f, 0.0f};

    Eigen::Vector3f angle_offset_                   = {0.0f, 0.0f, 0.0f};
    Eigen::Matrix<float, 3, 3> install_spin_matrix_ = Eigen::Matrix<float, 3, 3>::Identity();

    IMU_data IMU_data_ = {};
    device::bmi088 bmi088_;
    std::function<void()> callback_ = nullptr;
    IMU_attitude IMU_attitude_;
    IMU_correction IMU_correction_ = {};
    bool AHRS_ready_               = false;

    void update() {
        device::BMI088_Data raw_data;
        if (bmi088_.Acquire(&raw_data)) {
            dt = DWT_GetDeltaT(&bmi088_.bias_dwt_cnt);
            spin_drift(&raw_data);

            IMU_data_.Accel[X] = raw_data.acc[X];
            IMU_data_.Accel[Y] = raw_data.acc[Y];
            IMU_data_.Accel[Z] = raw_data.acc[Z];
            IMU_data_.Gyro[X]  = raw_data.gyro[X];
            IMU_data_.Gyro[Y]  = raw_data.gyro[Y];
            IMU_data_.Gyro[Z]  = raw_data.gyro[Z];

            // demo function,用于修正安装误差,可以不管,本demo暂时没用
            IMU_Param_Correction(&IMU_correction_, IMU_data_.Gyro, IMU_data_.Accel);

            // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
            // IMU_data_.atanxz = -atan2f(IMU_data_.Accel[X], IMU_data_.Accel[Z]) ;
            // IMU_data_.atanyz = atan2f(IMU_data_.Accel[Y], IMU_data_.Accel[Z]) ;

            // 核心函数,EKF更新四元数
            IMU_QuaternionEKF_Update(
                IMU_data_.Gyro[X], IMU_data_.Gyro[Y], IMU_data_.Gyro[Z], IMU_data_.Accel[X],
                IMU_data_.Accel[Y], IMU_data_.Accel[Z], dt);

            memcpy(IMU_data_.q, QEKF_INS.q, sizeof(QEKF_INS.q));

            // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
            BodyFrameToEarthFrame(xb, IMU_data_.xn, IMU_data_.q);
            BodyFrameToEarthFrame(yb, IMU_data_.yn, IMU_data_.q);
            BodyFrameToEarthFrame(zb, IMU_data_.zn, IMU_data_.q);

            // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
            float gravity_b[3];
            EarthFrameToBodyFrame(gravity, gravity_b, IMU_data_.q);
            for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
            {
                IMU_data_.MotionAccel_b[i] =
                    (IMU_data_.Accel[i] - gravity_b[i]) * dt / (IMU_data_.AccelLPF + dt)
                    + IMU_data_.MotionAccel_b[i] * IMU_data_.AccelLPF / (IMU_data_.AccelLPF + dt);
            }
            // 转换回导航系n
            BodyFrameToEarthFrame(IMU_data_.MotionAccel_b, IMU_data_.MotionAccel_n, IMU_data_.q);
            // PRY
            IMU_data_.Yaw           = QEKF_INS.Yaw + angle_offset_[2];
            IMU_data_.Pitch         = QEKF_INS.Pitch + angle_offset_[0];
            IMU_data_.Roll          = QEKF_INS.Roll + angle_offset_[1];
            IMU_data_.YawTotalAngle = QEKF_INS.YawTotalAngle + angle_offset_[2];

            output_vector.gyro           = get_gyro();
            output_vector.accel          = get_accel();
            output_vector.euler_angle    = get_euler_angle();
            output_vector.Yaw_multi_turn = IMU_data_.YawTotalAngle;
            AHRS_ready_                  = true;
        }
    }
    // 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
    void InitQuaternion(float* init_q4) {
        float acc_init[3]     = {0};
        float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
        float axis_rot[3]     = {0};       // 旋转轴
        device::BMI088_Data raw_data;
        // 读取100次加速度计数据,取平均值作为初始值
        for (uint8_t i = 0; i < 100; ++i) {
            bmi088_.Acquire(&raw_data);
            spin_drift(&raw_data);
            acc_init[X] += raw_data.acc[X];
            acc_init[Y] += raw_data.acc[Y];
            acc_init[Z] += raw_data.acc[Z];
            DWT_Delay(0.001);
        }
        for (auto& i : acc_init)
            i /= 100;
        Norm3d(acc_init);
        // 计算原始加速度矢量和导航系重力加速度矢量的夹角
        float angle = acosf(Dot3d(acc_init, gravity_norm));
        Cross3d(acc_init, gravity_norm, axis_rot);
        Norm3d(axis_rot);
        init_q4[0] = cosf(angle / 2.0f);
        for (uint8_t i = 0; i < 2; ++i)
            init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
    }
    //@brief 旋转陀螺仪和加速度计,并加上零漂,因为设备有不同安装方式
    void spin_drift(device::BMI088_Data* data) {
        Eigen::Map<Eigen::Vector3f> gyro(data->gyro);
        Eigen::Map<Eigen::Vector3f> acc(data->acc);
        gyro = install_spin_matrix_ * gyro + gyro_offset_;
        acc  = install_spin_matrix_ * acc + accel_offset_;
    }
    /**
     * @brief          Transform 3dvector from BodyFrame to EarthFrame
     * @param[1]       vector in BodyFrame
     * @param[2]       vector in EarthFrame
     * @param[3]       quaternion
     */
    static void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, const float* q) {
        vecEF[0] =
            2.0f
            * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0]
               + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

        vecEF[1] = 2.0f
                 * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0]
                    + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1]
                    + (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

        vecEF[2] = 2.0f
                 * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1]
                    + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
    }
    /**
     * @brief          Transform 3dvector from EarthFrame to BodyFrame
     * @param[1]       vector in EarthFrame
     * @param[2]       vector in BodyFrame
     * @param[3]       quaternion
     */
    static void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, const float* q) {
        vecBF[0] =
            2.0f
            * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0]
               + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

        vecBF[1] = 2.0f
                 * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0]
                    + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1]
                    + (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

        vecBF[2] = 2.0f
                 * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1]
                    + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
    }
    /**
     * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
     *
     *
     * @param param IMU参数
     * @param gyro  角速度
     * @param accel 加速度
     */
    static void IMU_Param_Correction(IMU_correction* param, float gyro[3], float accel[3]) {
        static float lastYawOffset, lastPitchOffset, lastRollOffset;
        static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
        float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

        if (fabsf(param->Yaw - lastYawOffset) > 0.001f
            || fabsf(param->Pitch - lastPitchOffset) > 0.001f
            || fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
            cosYaw   = arm_cos_f32(param->Yaw);
            cosPitch = arm_cos_f32(param->Pitch);
            cosRoll  = arm_cos_f32(param->Roll);
            sinYaw   = arm_sin_f32(param->Yaw);
            sinPitch = arm_sin_f32(param->Pitch);
            sinRoll  = arm_sin_f32(param->Roll);

            // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
            c_11        = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
            c_12        = cosPitch * sinYaw;
            c_13        = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
            c_21        = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
            c_22        = cosYaw * cosPitch;
            c_23        = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
            c_31        = -cosPitch * sinRoll;
            c_32        = sinPitch;
            c_33        = cosPitch * cosRoll;
            param->flag = 0;
        }
        float gyro_temp[3];
        for (uint8_t i = 0; i < 3; ++i)
            gyro_temp[i] = gyro[i] * param->scale[i];

        gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
        gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
        gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

        float accel_temp[3];
        for (uint8_t i = 0; i < 3; ++i)
            accel_temp[i] = accel[i];

        accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
        accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
        accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

        lastYawOffset   = param->Yaw;
        lastPitchOffset = param->Pitch;
        lastRollOffset  = param->Roll;
    }
    /**
     * @brief        Update quaternion
     */
    static void QuaternionUpdate(float* q, float gx, float gy, float gz, float dt) {
        float qa, qb, qc;

        gx *= 0.5f * dt;
        gy *= 0.5f * dt;
        gz *= 0.5f * dt;
        qa = q[0];
        qb = q[1];
        qc = q[2];
        q[0] += (-qb * gx - qc * gy - q[3] * gz);
        q[1] += (qa * gx + qc * gz - q[3] * gy);
        q[2] += (qa * gy - qb * gz + q[3] * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);
    }
    /**
     * @brief        Convert quaternion to eular angle
     */
    static void QuaternionToEularAngle(const float* q, float* Yaw, float* Pitch, float* Roll) {
        *Yaw =
            atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
        *Pitch =
            atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
        *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
    }
    /**
     * @brief        Convert eular angle to quaternion
     */
    static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float* q) {
        float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
        cosPitch = arm_cos_f32(Pitch / 2);
        cosYaw   = arm_cos_f32(Yaw / 2);
        cosRoll  = arm_cos_f32(Roll / 2);
        sinPitch = arm_sin_f32(Pitch / 2);
        sinYaw   = arm_sin_f32(Yaw / 2);
        sinRoll  = arm_sin_f32(Roll / 2);
        q[0]     = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
        q[1]     = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
        q[2]     = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
        q[3]     = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
    }
};
} // namespace module