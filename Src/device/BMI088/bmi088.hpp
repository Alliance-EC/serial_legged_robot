#pragma once

#include "bmi088_reg.hpp"
#include "bsp/gpio/gpio.hpp"
#include "bsp/pwm/pwm.hpp"
#include "bsp/spi/spi.hpp"
#include "tool/pid/pid.hpp"
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <functional>
#include <utility>

namespace device {
// bmi088工作模式枚举
enum class BMI088_Work_Mode : uint8_t { //@note 阻塞是指解算是阻塞的
    BLOCK_PERIODIC_MODE = 0,            // 阻塞模式,周期性读取
    BLOCK_TRIGGER_MODE,                 // 阻塞模式,触发读取(中断)
};
// bmi088标定方式枚举,若使用预设标定参数,注意修改预设参数
enum class BMI088_Calibrate_Mode : uint8_t {
    CALIBRATE_ONLINE_MODE = 0, // 初始化时进行标定
    LOAD_PRE_CALI_MODE,        // 使用预设标定参数,
};
/* BMI088数据*/
struct BMI088_Data {
    float gyro[3]     = {}; // 陀螺仪数据, xyz
    float acc[3]      = {}; // 加速度计数据, xyz
    float temperature = 0;  // 温度
};
struct BMI088_params {
    BMI088_Work_Mode work_mode       = BMI088_Work_Mode::BLOCK_TRIGGER_MODE;
    BMI088_Calibrate_Mode cali_mode  = {};
    bsp::spi_params spi_gyro_params  = {};
    bsp::spi_params spi_acc_params   = {};
    bsp::gpio_params gyro_int_params = {};
    bsp::gpio_params acc_int_params  = {};
    tool::PID_params heat_pid_params = {};
    bsp::pwm_params heat_pwm_params  = {};
    std::function<void()> callback   = nullptr;
};
class bmi088 {
public:
    explicit bmi088(const BMI088_params& params)
        : work_mode_(params.work_mode)
        , cali_mode_(params.cali_mode)
        , spi_gyro_(params.spi_gyro_params)
        , spi_acc_(params.spi_acc_params)
        , gyro_int_(params.gyro_int_params)
        , acc_int_(params.acc_int_params)
        , heat_pid_(params.heat_pid_params)
        , heat_pwm_(params.heat_pwm_params)
        , callback_(params.callback) {
        DWT_GetDeltaT(&bias_dwt_cnt);
        // 初始化时使用阻塞模式
        SetMode(BMI088_Work_Mode::BLOCK_PERIODIC_MODE);
        // 初始化加速度计和陀螺仪
        bmi088_reg::ErrorCode error = AccelInit();
        if (error != bmi088_reg::ErrorCode::NO_ERROR) {
            while (true)
                ;
        }
        error = GyroInit();
        if (error != bmi088_reg::ErrorCode::NO_ERROR) {
            while (true)
                ;
        }
        // 初始化中断
        CalibrateIMU();
        SetMode(params.work_mode); // 恢复工作模式
        if (work_mode_ == BMI088_Work_Mode::BLOCK_TRIGGER_MODE) {
            spi_acc_.SetCallback(std::bind(&bmi088::AccSPIFinishCallback, this));
            spi_gyro_.SetCallback(std::bind(&bmi088::GyroSPIFinishCallback, this));
            gyro_int_.SetCallback(std::bind(&bmi088::GyroINTCallback, this));
            acc_int_.SetCallback(std::bind(&bmi088::AccINTCallback, this));
        }
    }
    ~bmi088() = default;
    uint8_t Acquire(BMI088_Data* data_store) {
        // 如果是blocking模式,则主动触发一次读取并返回数据
        if (work_mode_ == BMI088_Work_Mode::BLOCK_PERIODIC_MODE) {
            static uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
            static size_t invoke_times;
            // 读取accel的x轴数据首地址,bmi088内部自增读取地址 // 3* sizeof(int16_t)
            AccelRead(bmi088_reg::ACCEL_XOUT_L, buf, 6);
            for (uint8_t i = 0; i < 3; i++)
                data_store->acc[i] =
                    ACCEL_SEN_ * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]);
            GyroRead(bmi088_reg::GYRO_X_L, buf, 6); // 连续读取3个(3*2=6)轴的角速度
            for (uint8_t i = 0; i < 3; i++)
                data_store->gyro[i] =
                    GYRO_SEN_ * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]);
            AccelRead(bmi088_reg::TEMP_M, buf, 2);  // 读温度,温度传感器在accel上
            data_store->temperature =
                (float)(int16_t)((buf[0] << 3) | (buf[1] >> 5)) * bmi088_reg::TEMP_FACTOR
                + bmi088_reg::TEMP_OFFSET;
            // 更新BMI088自身结构体数据
            for (uint8_t i = 0; i < 3; i++) {
                bmi088_data_.acc[i]  = data_store->acc[i];
                bmi088_data_.gyro[i] = data_store->gyro[i];
            }
            bmi088_data_.temperature = data_store->temperature;
            if (invoke_times >= 100) {
                temperature_control();
                invoke_times = 0;
            }
            invoke_times++;
            return 1;
        }
        // 如果是IT模式,则检查标志位.当传感器数据准备好会触发外部中断,中断服务函数会将标志位置1
        if (work_mode_ == BMI088_Work_Mode::BLOCK_TRIGGER_MODE && update_flag_.imu_ready == 1) {
            for (uint8_t i = 0; i < 3; i++) {
                data_store->acc[i]  = bmi088_data_.acc[i];
                data_store->gyro[i] = bmi088_data_.gyro[i];
            }
            data_store->temperature = bmi088_data_.temperature;
            update_flag_.imu_ready  = 0;
            return 1;
        }
        // 如果数据还没准备好,则返回空数据
        if (update_flag_.imu_ready == 0) {
            data_store = nullptr;
            return 0;
        }
        return 0;
    }

    //@return 1 数据准备完毕 0 没有数据
    uint8_t Get_IT_Status() {
        // 只有中断才能读取标志位
        if (work_mode_ == BMI088_Work_Mode::BLOCK_TRIGGER_MODE && update_flag_.imu_ready == 1) {
            update_flag_.imu_ready = 0;
            return 1;
        } else
            return 0;
    }
    BMI088_Work_Mode GetWorkMode() { return work_mode_; }

    void SetCallback(std::function<void()> callback) { callback_ = std::move(callback); }
    Eigen::Vector3f GetGyroOffset() { return {gyro_offset_[0], gyro_offset_[1], gyro_offset_[2]}; }
    // 用于计算两次采样的时间间隔
    uint32_t bias_dwt_cnt = 0;

private:
    // 传输模式和工作模式控制
    BMI088_Work_Mode work_mode_;
    BMI088_Calibrate_Mode cali_mode_;
    // SPI接口
    bsp::spi spi_gyro_; // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    bsp::spi spi_acc_;  // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    // EXTI GPIO,如果BMI088工作在中断模式,则需要配置中断引脚(有数据产生时触发解算)
    bsp::gpio gyro_int_;
    bsp::gpio acc_int_;
    // 温度控制
    tool::PID heat_pid_; // 恒温PID
    bsp::pwm heat_pwm_;  // 加热PWM
    // RAW数据
    uint8_t gyro_raw_[6];
    uint8_t acc_raw_[6];
    uint8_t temp_raw_[2];
    // IMU数据
    BMI088_Data bmi088_data_;
    // 标定数据
    static constexpr std::array<float, 3> pre_cali_offset = {
        -0.00505202683, 0.00223287498, -0.00349383592};
    float gyro_offset_[3] = {}; // 陀螺仪零偏
    // float acc_offset_[3]                                  = {}; // 加速度计零偏
    //  传感器灵敏度,用于计算实际值(regNdef.h中定义)
    float ACCEL_SEN_ = 0;
    float GYRO_SEN_  = 0;

    // 数据更新标志位
    struct                        // 位域,节省空间提高可读性
    {
        uint8_t gyro         : 1; // 1:有新数据,0:无新数据
        uint8_t acc          : 1;
        uint8_t temp         : 1;
        uint8_t gyro_overrun : 1; // 1:数据溢出,0:无溢出
        uint8_t acc_overrun  : 1;
        uint8_t temp_overrun : 1;
        uint8_t imu_ready    : 1; // 1:IMU数据准备好,0:IMU数据未准备好(gyro+acc)
        uint8_t reserved : 1; // 后续可添加其他标志位,不够用可以扩充16or32,太多可以删
    } update_flag_ = {0};
    std::function<void()> callback_;

    // --------------------以下私有函数,用于读写BMI088寄存器封装,blocking----------------------//
    void AccelRead(uint8_t reg, uint8_t* dataptr, uint8_t len) {
        if (len > 6)
            while (true)
                ;
        // 一次读取最多6个字节,加上两个dummy data
        // 第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
        static uint8_t
            tx[8]; // 读取,第一个字节为0x80|reg ,第二个是dummy data,后面的没用都是dummy write
        static uint8_t rx[8]; // 前两个字节是dummy data,第三个开始是真正的数据
        tx[0] = 0x80 | reg; // 静态变量每次进来还是上次的值,所以要每次都要给tx[0]赋值0x80
        spi_acc_.TransmitReceive(rx, tx, len + 2);
        memcpy(dataptr, rx + 2, len);
    }
    void GyroRead(uint8_t reg, uint8_t* dataptr, uint8_t len) {
        if (len > 6)
            while (true)
                ;
        // 一次读取最多6个字节,加上一个dummy data
        // ,第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
        static uint8_t tx[7] = {0x80}; // 读取,第一个字节为0x80 | reg ,之后是dummy data
        static uint8_t rx[7];          // 第一个是dummy data,第三个开始是真正的数据
        tx[0] = 0x80 | reg;
        spi_gyro_.TransmitReceive(rx, tx, len + 1);
        memcpy(dataptr, rx + 1, len);
    }
    void AccelWriteSingleReg(uint8_t reg, uint8_t data) {
        uint8_t tx[2] = {reg, data};
        spi_acc_.Transmit(tx, 2);
    }
    void GyroWriteSingleReg(uint8_t reg, uint8_t data) {
        uint8_t tx[2] = {reg, data};
        spi_gyro_.Transmit(tx, 2);
    }
    // ----------------以下为私有函数,用于初始化BMI088acc和gyro的硬件和配置-------------------//
    static constexpr uint8_t BMI088REG   = 0;
    static constexpr uint8_t BMI088DATA  = 1;
    static constexpr uint8_t BMI088ERROR = 2;
    // BMI088初始化配置数组for accel,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
    // clang-format off
    constexpr static uint8_t BMI088_Accel_Init_Table[bmi088_reg::WRITE_ACCEL_REG_NUM][3] = {
    {bmi088_reg::ACC_PWR_CTRL, bmi088_reg::ACC_ENABLE_ACC_ON                                                                , bmi088_reg::ErrorCode::ACC_PWR_CTRL_ERROR},
    {bmi088_reg::ACC_PWR_CONF, bmi088_reg::ACC_PWR_ACTIVE_MODE                                                              , bmi088_reg::ErrorCode::ACC_PWR_CONF_ERROR},
    {bmi088_reg::ACC_CONF    , bmi088_reg::ACC_NORMAL | bmi088_reg::ACC_1600_HZ | bmi088_reg::ACC_CONF_MUST_Set             , bmi088_reg::ErrorCode::ACC_CONF_ERROR},
    {bmi088_reg::ACC_RANGE   , bmi088_reg::ACC_RANGE_6G                                                                     , bmi088_reg::ErrorCode::ACC_RANGE_ERROR},
    {bmi088_reg::INT1_IO_CTRL, bmi088_reg::ACC_INT1_IO_ENABLE | bmi088_reg::ACC_INT1_GPIO_PP | bmi088_reg::ACC_INT1_GPIO_LOW, bmi088_reg::ErrorCode::INT1_IO_CTRL_ERROR},
    {bmi088_reg::INT_MAP_DATA, bmi088_reg::ACC_INT1_DRDY_INTERRUPT                                                          , bmi088_reg::ErrorCode::INT_MAP_DATA_ERROR}
    };
    // BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
    constexpr static uint8_t BMI088_Gyro_Init_Table[bmi088_reg::WRITE_GYRO_REG_NUM][3] = {
    {bmi088_reg::GYRO_RANGE            , bmi088_reg::GYRO_2000                                             , bmi088_reg::ErrorCode::GYRO_RANGE_ERROR},
    {bmi088_reg::GYRO_BANDWIDTH        , bmi088_reg::GYRO_1000_116_HZ | bmi088_reg::GYRO_BANDWIDTH_MUST_Set, bmi088_reg::ErrorCode::GYRO_BANDWIDTH_ERROR},
    {bmi088_reg::GYRO_LPM1             , bmi088_reg::GYRO_NORMAL_MODE                                      , bmi088_reg::ErrorCode::GYRO_LPM1_ERROR},
    {bmi088_reg::GYRO_CTRL             , bmi088_reg::DRDY_ON                                               , bmi088_reg::ErrorCode::GYRO_CTRL_ERROR},
    {bmi088_reg::GYRO_INT3_INT4_IO_CONF, bmi088_reg::GYRO_INT3_GPIO_PP | bmi088_reg::GYRO_INT3_GPIO_LOW    , bmi088_reg::ErrorCode::GYRO_INT3_INT4_IO_CONF_ERROR},
    {bmi088_reg::GYRO_INT3_INT4_IO_MAP , bmi088_reg::GYRO_DRDY_IO_INT3                                     , bmi088_reg::ErrorCode::GYRO_INT3_INT4_IO_MAP_ERROR}
    };
    // clang-format on
    // @attention : 以上两个数组配合各自的初始化函数使用. 若要修改请参照BMI088 datasheet

    bmi088_reg::ErrorCode AccelInit() {
        uint8_t WhoAmI_check = 0;

        // 加速度计以I2C模式启动,需要一次上升沿来切换到SPI模式,因此进行一次fake write
        AccelRead(bmi088_reg::ACC_CHIP_ID, &WhoAmI_check, 1);
        DWT_Delay(0.001f);

        AccelWriteSingleReg(bmi088_reg::ACC_SOFTRESET, bmi088_reg::ACC_SOFTRESET_VALUE); // 软复位
        DWT_Delay(bmi088_reg::COM_WAIT_SENSOR_TIME / 1000.0f);

        AccelRead(bmi088_reg::ACC_CHIP_ID, &WhoAmI_check, 1);
        // if (WhoAmI_check != bmi088_reg::ACC_CHIP_ID_VALUE)
        //     return bmi088_reg::ErrorCode::NO_SENSOR;
        DWT_Delay(0.001f);

        uint8_t reg = 0, data = 0;
        bmi088_reg::ErrorCode error = bmi088_reg::ErrorCode::NO_ERROR;

        for (auto i : BMI088_Accel_Init_Table) {
            reg  = i[BMI088REG];
            data = i[BMI088DATA];
            AccelWriteSingleReg(reg, data); // 写入寄存器
            DWT_Delay(0.01f);
            AccelRead(reg, &data, 1);       // 写完之后立刻读回检查
            DWT_Delay(0.01f);
            if (data != i[BMI088DATA])
                error = static_cast<bmi088_reg::ErrorCode>(i[BMI088ERROR]);
        }

        switch (BMI088_Accel_Init_Table[3][1]) {
        case bmi088_reg::ACC_RANGE_3G: ACCEL_SEN_ = bmi088_reg::ACCEL_3G_SEN; break;
        case bmi088_reg::ACC_RANGE_6G: ACCEL_SEN_ = bmi088_reg::ACCEL_6G_SEN; break;
        case bmi088_reg::ACC_RANGE_12G: ACCEL_SEN_ = bmi088_reg::ACCEL_12G_SEN; break;
        case bmi088_reg::ACC_RANGE_24G: ACCEL_SEN_ = bmi088_reg::ACCEL_24G_SEN; break;
        default: break;
        }
        return error;
    }

    bmi088_reg::ErrorCode GyroInit() {
        GyroWriteSingleReg(bmi088_reg::GYRO_SOFTRESET, bmi088_reg::GYRO_SOFTRESET_VALUE); // 软复位
        DWT_Delay(0.08f);

        uint8_t WhoAmI_check = 0;
        GyroRead(bmi088_reg::GYRO_CHIP_ID, &WhoAmI_check, 1);
        if (WhoAmI_check != bmi088_reg::GYRO_CHIP_ID_VALUE)
            return bmi088_reg::ErrorCode::NO_SENSOR;
        DWT_Delay(0.001f);

        uint8_t reg = 0, data = 0;
        bmi088_reg::ErrorCode error = bmi088_reg::ErrorCode::NO_ERROR;
        for (auto i : BMI088_Gyro_Init_Table) {
            reg  = i[BMI088REG];
            data = i[BMI088DATA];
            GyroWriteSingleReg(reg, data); // 写入寄存器
            DWT_Delay(0.001f);
            GyroRead(reg, &data, 1); // 写完之后立刻读回对应寄存器检查是否写入成功
            DWT_Delay(0.001f);
            if (data != i[BMI088DATA])
                error = static_cast<bmi088_reg::ErrorCode>(i[BMI088ERROR]);
        }
        // 设置灵敏度
        switch (BMI088_Gyro_Init_Table[0][1]) {
        case bmi088_reg::GYRO_2000: GYRO_SEN_ = bmi088_reg::GYRO_2000_SEN; break;
        case bmi088_reg::GYRO_1000: GYRO_SEN_ = bmi088_reg::GYRO_1000_SEN; break;
        case bmi088_reg::GYRO_500: GYRO_SEN_ = bmi088_reg::GYRO_500_SEN; break;
        case bmi088_reg::GYRO_250: GYRO_SEN_ = bmi088_reg::GYRO_250_SEN; break;
        case bmi088_reg::GYRO_125: GYRO_SEN_ = bmi088_reg::GYRO_125_SEN; break;
        default: break;
        }
        return error;
    }
    //----------------以下为私有函数,private用于IT模式下的中断处理----------------------//
    void AccSPIFinishCallback() {

        //  如果是加速度计的中断,则启动加速度计数据读取,并转换为实际值
        if (update_flag_.acc == 1) {
            for (uint8_t i = 0; i < 3; i++)
                bmi088_data_.acc[i] =
                    ACCEL_SEN_ * (float)(int16_t)(((acc_raw_[2 * i + 1]) << 8) | acc_raw_[2 * i]);
            update_flag_.acc = 0;
        }
        if (update_flag_.temp == 1) {
            bmi088_data_.temperature = (float)(int16_t)((temp_raw_[0] << 3) | (temp_raw_[1] >> 5))
                                         * bmi088_reg::TEMP_FACTOR
                                     + bmi088_reg::TEMP_OFFSET;
            temperature_control();
            update_flag_.temp = 0;
        }
    }

    void GyroSPIFinishCallback() {
        for (uint8_t i = 0; i < 3; i++)
            bmi088_data_.gyro[i] =
                GYRO_SEN_ * (float)(int16_t)(((gyro_raw_[2 * i + 1]) << 8) | gyro_raw_[2 * i]);
        update_flag_.gyro      = 0;
        update_flag_.imu_ready = 1;
        // 然后进行解算回调
        if (callback_) {
            callback_();
        }
    }

    void AccINTCallback() {
        static uint16_t callback_time = 0;
        AccelRead(bmi088_reg::ACCEL_XOUT_L, acc_raw_, 6);
        update_flag_.acc = 1;
        if (callback_time >= 100) {
            AccelRead(bmi088_reg::TEMP_M, temp_raw_, 2);
            update_flag_.temp = 1;
            callback_time     = 0;
        }
        callback_time++;
        // 读取完毕会调用AccSPIFinishCallback
    }

    void GyroINTCallback() {
        // 启动陀螺仪数据读取,并转换为实际值
        GyroRead(bmi088_reg::GYRO_X_L, gyro_raw_, 6);
        update_flag_.gyro = 1;
        // 读取完毕会调用GyroSPIFinishCallback
    }
    void SetMode(BMI088_Work_Mode mode) {
        work_mode_ = mode;
        if (mode == BMI088_Work_Mode::BLOCK_PERIODIC_MODE) {
            spi_acc_.SetMode(bsp::SPI_TXRX_MODE::POLLING);
            spi_gyro_.SetMode(bsp::SPI_TXRX_MODE::POLLING);
        } else if (mode == BMI088_Work_Mode::BLOCK_TRIGGER_MODE) {
            spi_acc_.SetMode(bsp::SPI_TXRX_MODE::DMA);
            spi_gyro_.SetMode(bsp::SPI_TXRX_MODE::DMA);
        }
    }
    void temperature_control() {
        static uint8_t temperature_constant_time = 0;
        static uint8_t first_temperature         = 0;  // 第一次达到设定温度
        static float target_temperature          = 48; // 推荐比环境温度高10度
        target_temperature                       = std::min(target_temperature, 48.0f);

        if (first_temperature) {
            auto pid_out =
                static_cast<float>(heat_pid_.update(target_temperature, bmi088_data_.temperature));
            pid_out = std::clamp(pid_out, 0.0f, 1.0f);
            heat_pid_.IntegralNoNagtive();
            heat_pwm_.SetDutyRatio(pid_out);
        } else {
            // 在没有达到设置的温度-4，一直最大功率加热
            heat_pwm_.SetDutyRatio(0.95f);
            if (bmi088_data_.temperature > target_temperature - 4) {
                temperature_constant_time++;
                if (temperature_constant_time > 200) {
                    // 达到设置温度，设置积分项，加速收敛
                    first_temperature = 1;
                    heat_pid_.SetIntegral(0.05f);
                }
            }
        }
    }
    static constexpr uint16_t GYRO_CALIBRATE_TIME = 20000; // 20s
    // 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定
    void CalibrateIMU() {
        if (cali_mode_ == BMI088_Calibrate_Mode::CALIBRATE_ONLINE_MODE) {
            static uint16_t cali_time_count = 0;
            while (cali_time_count < GYRO_CALIBRATE_TIME) {
                BMI088_Data bmi088_data = {};
                Acquire(&bmi088_data);
                for (uint8_t i = 0; i < 3; i++) {
                    bmi088_data.gyro[i] += gyro_offset_[i];
                    gyro_offset_[i] -= 0.0003f * bmi088_data.gyro[i];
                }
                cali_time_count++;
                DWT_Delay(0.001f);
            }
        }
        // 导入数据
        else if (cali_mode_ == BMI088_Calibrate_Mode::LOAD_PRE_CALI_MODE) {
            gyro_offset_[0] = pre_cali_offset[0];
            gyro_offset_[1] = pre_cali_offset[1];
            gyro_offset_[2] = pre_cali_offset[2];
        }
    }
};
} // namespace device