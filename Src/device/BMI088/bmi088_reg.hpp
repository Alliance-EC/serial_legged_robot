#pragma once

#include <cstdint>
namespace device::bmi088_reg {

// Register addresses and their content
constexpr uint8_t ACC_CHIP_ID       = 0x00;
constexpr uint8_t ACC_CHIP_ID_VALUE = 0x1E;

constexpr uint8_t ACC_ERR_REG               = 0x02;
constexpr uint8_t ACCEL_CONGIF_ERROR_SHIFTS = 0x2;
constexpr uint8_t ACCEL_CONGIF_ERROR        = (1 << ACCEL_CONGIF_ERROR_SHIFTS);
constexpr uint8_t FATAL_ERROR_SHIFTS        = 0x0;
constexpr uint8_t FATAL_ERROR               = (1 << FATAL_ERROR_SHIFTS);

constexpr uint8_t ACC_STATUS        = 0x03;
constexpr uint8_t ACCEL_DRDY_SHIFTS = 0x7;
constexpr uint8_t ACCEL_DRDY        = (1 << ACCEL_DRDY_SHIFTS);

constexpr uint8_t ACCEL_XOUT_L = 0x12;
constexpr uint8_t ACCEL_XOUT_M = 0x13;
constexpr uint8_t ACCEL_YOUT_L = 0x14;
constexpr uint8_t ACCEL_YOUT_M = 0x15;
constexpr uint8_t ACCEL_ZOUT_L = 0x16;
constexpr uint8_t ACCEL_ZOUT_M = 0x17;

constexpr uint8_t SENSORTIME_DATA_L = 0x18;
constexpr uint8_t SENSORTIME_DATA_M = 0x19;
constexpr uint8_t SENSORTIME_DATA_H = 0x1A;

constexpr uint8_t ACC_INT_STAT_1              = 0x1D;
constexpr uint8_t ACCEL_DRDY_INTERRUPT_SHIFTS = 0x7;
constexpr uint8_t ACCEL_DRDY_INTERRUPT        = (1 << ACCEL_DRDY_INTERRUPT_SHIFTS);

constexpr uint8_t TEMP_M = 0x22;
constexpr uint8_t TEMP_L = 0x23;

constexpr uint8_t ACC_CONF          = 0x40;
constexpr uint8_t ACC_CONF_MUST_Set = 0x80;
constexpr uint8_t ACC_BWP_SHIFTS    = 0x4;
constexpr uint8_t ACC_OSR4          = (0x0 << ACC_BWP_SHIFTS);
constexpr uint8_t ACC_OSR2          = (0x1 << ACC_BWP_SHIFTS);
constexpr uint8_t ACC_NORMAL        = (0x2 << ACC_BWP_SHIFTS);

constexpr uint8_t ACC_ODR_SHIFTS = 0x0;
constexpr uint8_t ACC_12_5_HZ    = (0x5 << ACC_ODR_SHIFTS);
constexpr uint8_t ACC_25_HZ      = (0x6 << ACC_ODR_SHIFTS);
constexpr uint8_t ACC_50_HZ      = (0x7 << ACC_ODR_SHIFTS);
constexpr uint8_t ACC_100_HZ     = (0x8 << ACC_ODR_SHIFTS);
constexpr uint8_t ACC_200_HZ     = (0x9 << ACC_ODR_SHIFTS);
constexpr uint8_t ACC_400_HZ     = (0xA << ACC_ODR_SHIFTS);
constexpr uint8_t ACC_800_HZ     = (0xB << ACC_ODR_SHIFTS);
constexpr uint8_t ACC_1600_HZ    = (0xC << ACC_ODR_SHIFTS);

constexpr uint8_t ACC_RANGE = 0x41;

constexpr uint8_t ACC_RANGE_SHIFTS = 0x0;
constexpr uint8_t ACC_RANGE_3G     = (0x0 << ACC_RANGE_SHIFTS);
constexpr uint8_t ACC_RANGE_6G     = (0x1 << ACC_RANGE_SHIFTS);
constexpr uint8_t ACC_RANGE_12G    = (0x2 << ACC_RANGE_SHIFTS);
constexpr uint8_t ACC_RANGE_24G    = (0x3 << ACC_RANGE_SHIFTS);

constexpr uint8_t INT1_IO_CTRL              = 0x53;
constexpr uint8_t ACC_INT1_IO_ENABLE_SHIFTS = 0x3;
constexpr uint8_t ACC_INT1_IO_ENABLE        = (0x1 << ACC_INT1_IO_ENABLE_SHIFTS);
constexpr uint8_t ACC_INT1_GPIO_MODE_SHIFTS = 0x2;
constexpr uint8_t ACC_INT1_GPIO_PP          = (0x0 << ACC_INT1_GPIO_MODE_SHIFTS);
constexpr uint8_t ACC_INT1_GPIO_OD          = (0x1 << ACC_INT1_GPIO_MODE_SHIFTS);
constexpr uint8_t ACC_INT1_GPIO_LVL_SHIFTS  = 0x1;
constexpr uint8_t ACC_INT1_GPIO_LOW         = (0x0 << ACC_INT1_GPIO_LVL_SHIFTS);
constexpr uint8_t ACC_INT1_GPIO_HIGH        = (0x1 << ACC_INT1_GPIO_LVL_SHIFTS);

constexpr uint8_t INT2_IO_CTRL              = 0x54;
constexpr uint8_t ACC_INT2_IO_ENABLE_SHIFTS = 0x3;
constexpr uint8_t ACC_INT2_IO_ENABLE        = (0x1 << ACC_INT2_IO_ENABLE_SHIFTS);
constexpr uint8_t ACC_INT2_GPIO_MODE_SHIFTS = 0x2;
constexpr uint8_t ACC_INT2_GPIO_PP          = (0x0 << ACC_INT2_GPIO_MODE_SHIFTS);
constexpr uint8_t ACC_INT2_GPIO_OD          = (0x1 << ACC_INT2_GPIO_MODE_SHIFTS);
constexpr uint8_t ACC_INT2_GPIO_LVL_SHIFTS  = 0x1;
constexpr uint8_t ACC_INT2_GPIO_LOW         = (0x0 << ACC_INT2_GPIO_LVL_SHIFTS);
constexpr uint8_t ACC_INT2_GPIO_HIGH        = (0x1 << ACC_INT2_GPIO_LVL_SHIFTS);

constexpr uint8_t INT_MAP_DATA                   = 0x58;
constexpr uint8_t ACC_INT2_DRDY_INTERRUPT_SHIFTS = 0x6;
constexpr uint8_t ACC_INT2_DRDY_INTERRUPT        = (0x1 << ACC_INT2_DRDY_INTERRUPT_SHIFTS);
constexpr uint8_t ACC_INT1_DRDY_INTERRUPT_SHIFTS = 0x2;
constexpr uint8_t ACC_INT1_DRDY_INTERRUPT        = (0x1 << ACC_INT1_DRDY_INTERRUPT_SHIFTS);

constexpr uint8_t ACC_SELF_TEST                 = 0x6D;
constexpr uint8_t ACC_SELF_TEST_OFF             = 0x00;
constexpr uint8_t ACC_SELF_TEST_POSITIVE_SIGNAL = 0x0D;
constexpr uint8_t ACC_SELF_TEST_NEGATIVE_SIGNAL = 0x09;

constexpr uint8_t ACC_PWR_CONF         = 0x7C;
constexpr uint8_t ACC_PWR_SUSPEND_MODE = 0x03;
constexpr uint8_t ACC_PWR_ACTIVE_MODE  = 0x00;

constexpr uint8_t ACC_PWR_CTRL       = 0x7D;
constexpr uint8_t ACC_ENABLE_ACC_OFF = 0x00;
constexpr uint8_t ACC_ENABLE_ACC_ON  = 0x04;

constexpr uint8_t ACC_SOFTRESET       = 0x7E;
constexpr uint8_t ACC_SOFTRESET_VALUE = 0xB6;

constexpr uint8_t GYRO_CHIP_ID       = 0x00;
constexpr uint8_t GYRO_CHIP_ID_VALUE = 0x0F;

constexpr uint8_t GYRO_X_L = 0x02;
constexpr uint8_t GYRO_X_H = 0x03;
constexpr uint8_t GYRO_Y_L = 0x04;
constexpr uint8_t GYRO_Y_H = 0x05;
constexpr uint8_t GYRO_Z_L = 0x06;
constexpr uint8_t GYRO_Z_H = 0x07;

constexpr uint8_t GYRO_INT_STAT_1  = 0x0A;
constexpr uint8_t GYRO_DYDR_SHIFTS = 0x7;
constexpr uint8_t GYRO_DYDR        = (0x1 << GYRO_DYDR_SHIFTS);

constexpr uint8_t GYRO_RANGE        = 0x0F;
constexpr uint8_t GYRO_RANGE_SHIFTS = 0x0;
constexpr uint8_t GYRO_2000         = (0x0 << GYRO_RANGE_SHIFTS);
constexpr uint8_t GYRO_1000         = (0x1 << GYRO_RANGE_SHIFTS);
constexpr uint8_t GYRO_500          = (0x2 << GYRO_RANGE_SHIFTS);
constexpr uint8_t GYRO_250          = (0x3 << GYRO_RANGE_SHIFTS);
constexpr uint8_t GYRO_125          = (0x4 << GYRO_RANGE_SHIFTS);

constexpr uint8_t GYRO_BANDWIDTH          = 0x10;
constexpr uint8_t GYRO_BANDWIDTH_MUST_Set = 0x80;
constexpr uint8_t GYRO_2000_532_HZ        = 0x00;
constexpr uint8_t GYRO_2000_230_HZ        = 0x01;
constexpr uint8_t GYRO_1000_116_HZ        = 0x02;
constexpr uint8_t GYRO_400_47_HZ          = 0x03;
constexpr uint8_t GYRO_200_23_HZ          = 0x04;
constexpr uint8_t GYRO_100_12_HZ          = 0x05;
constexpr uint8_t GYRO_200_64_HZ          = 0x06;
constexpr uint8_t GYRO_100_32_HZ          = 0x07;

constexpr uint8_t GYRO_LPM1              = 0x11;
constexpr uint8_t GYRO_NORMAL_MODE       = 0x00;
constexpr uint8_t GYRO_SUSPEND_MODE      = 0x80;
constexpr uint8_t GYRO_DEEP_SUSPEND_MODE = 0x20;

constexpr uint8_t GYRO_SOFTRESET       = 0x14;
constexpr uint8_t GYRO_SOFTRESET_VALUE = 0xB6;

constexpr uint8_t GYRO_CTRL = 0x15;
constexpr uint8_t DRDY_OFF  = 0x00;
constexpr uint8_t DRDY_ON   = 0x80;

constexpr uint8_t GYRO_INT3_INT4_IO_CONF     = 0x16;
constexpr uint8_t GYRO_INT4_GPIO_MODE_SHIFTS = 0x3;
constexpr uint8_t GYRO_INT4_GPIO_PP          = (0x0 << GYRO_INT4_GPIO_MODE_SHIFTS);
constexpr uint8_t GYRO_INT4_GPIO_OD          = (0x1 << GYRO_INT4_GPIO_MODE_SHIFTS);
constexpr uint8_t GYRO_INT4_GPIO_LVL_SHIFTS  = 0x2;
constexpr uint8_t GYRO_INT4_GPIO_LOW         = (0x0 << GYRO_INT4_GPIO_LVL_SHIFTS);
constexpr uint8_t GYRO_INT4_GPIO_HIGH        = (0x1 << GYRO_INT4_GPIO_LVL_SHIFTS);
constexpr uint8_t GYRO_INT3_GPIO_MODE_SHIFTS = 0x1;
constexpr uint8_t GYRO_INT3_GPIO_PP          = (0x0 << GYRO_INT3_GPIO_MODE_SHIFTS);
constexpr uint8_t GYRO_INT3_GPIO_OD          = (0x1 << GYRO_INT3_GPIO_MODE_SHIFTS);
constexpr uint8_t GYRO_INT3_GPIO_LVL_SHIFTS  = 0x0;
constexpr uint8_t GYRO_INT3_GPIO_LOW         = (0x0 << GYRO_INT3_GPIO_LVL_SHIFTS);
constexpr uint8_t GYRO_INT3_GPIO_HIGH        = (0x1 << GYRO_INT3_GPIO_LVL_SHIFTS);

constexpr uint8_t GYRO_INT3_INT4_IO_MAP = 0x18;

constexpr uint8_t GYRO_DRDY_IO_OFF  = 0x00;
constexpr uint8_t GYRO_DRDY_IO_INT3 = 0x01;
constexpr uint8_t GYRO_DRDY_IO_INT4 = 0x80;
constexpr uint8_t GYRO_DRDY_IO_BOTH = (GYRO_DRDY_IO_INT3 | GYRO_DRDY_IO_INT4);

constexpr uint8_t GYRO_SELF_TEST        = 0x3C;
constexpr uint8_t GYRO_RATE_OK_SHIFTS   = 0x4;
constexpr uint8_t GYRO_RATE_OK          = (0x1 << GYRO_RATE_OK_SHIFTS);
constexpr uint8_t GYRO_BIST_FAIL_SHIFTS = 0x2;
constexpr uint8_t GYRO_BIST_FAIL        = (0x1 << GYRO_BIST_FAIL_SHIFTS);
constexpr uint8_t GYRO_BIST_RDY_SHIFTS  = 0x1;
constexpr uint8_t GYRO_BIST_RDY         = (0x1 << GYRO_BIST_RDY_SHIFTS);
constexpr uint8_t GYRO_TRIG_BIST_SHIFTS = 0x0;
constexpr uint8_t GYRO_TRIG_BIST        = (0x1 << GYRO_TRIG_BIST_SHIFTS);

// Configuration and sensor sensitivity conversion factors
constexpr float TEMP_FACTOR = 0.125f;
constexpr float TEMP_OFFSET = 23.0f;

constexpr uint8_t WRITE_ACCEL_REG_NUM = 6;
constexpr uint8_t WRITE_GYRO_REG_NUM  = 6;

constexpr uint8_t GYRO_DATA_READY_BIT       = 0;
constexpr uint8_t ACCEL_DATA_READY_BIT      = 1;
constexpr uint8_t ACCEL_TEMP_DATA_READY_BIT = 2;

constexpr uint8_t LONG_DELAY_TIME      = 80;
constexpr uint16_t COM_WAIT_SENSOR_TIME = 150;

constexpr uint8_t ACCEL_IIC_ADDRESS = (0x18 << 1);
constexpr uint8_t GYRO_IIC_ADDRESS  = (0x68 << 1);

constexpr float ACCEL_3G_SEN  = 0.0008974358974f;
constexpr float ACCEL_6G_SEN  = 0.00179443359375f;
constexpr float ACCEL_12G_SEN = 0.0035888671875f;
constexpr float ACCEL_24G_SEN = 0.007177734375f;

constexpr float GYRO_2000_SEN = 0.00106526443603169529841533860381f;
constexpr float GYRO_1000_SEN = 0.00053263221801584764920766930190693f;
constexpr float GYRO_500_SEN  = 0.00026631610900792382460383465095346f;
constexpr float GYRO_250_SEN  = 0.00013315805450396191230191732547673f;
constexpr float GYRO_125_SEN  = 0.000066579027251980956150958662738366f;

// Error codes enum
enum ErrorCode : uint8_t {
    NO_ERROR                     = 0x00,
    ACC_PWR_CTRL_ERROR           = 0x01,
    ACC_PWR_CONF_ERROR           = 0x02,
    ACC_CONF_ERROR               = 0x03,
    ACC_SELF_TEST_ERROR          = 0x04,
    ACC_RANGE_ERROR              = 0x05,
    INT1_IO_CTRL_ERROR           = 0x06,
    INT_MAP_DATA_ERROR           = 0x07,
    GYRO_RANGE_ERROR             = 0x08,
    GYRO_BANDWIDTH_ERROR         = 0x09,
    GYRO_LPM1_ERROR              = 0x0A,
    GYRO_CTRL_ERROR              = 0x0B,
    GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    SELF_TEST_ACCEL_ERROR = 0x80,
    SELF_TEST_GYRO_ERROR  = 0x40,
    NO_SENSOR             = 0xFF
};

} // namespace device::bmi088_reg
