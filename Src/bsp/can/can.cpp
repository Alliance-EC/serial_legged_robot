#include "can.hpp"

namespace bsp {
// 定义静态成员变量
std::array<can*, can::MAX_CAN_INSTANCES> can::can_instances_ = {};
size_t can::can_instance_count_                              = 0;

void CANFIFOxCallback(FDCAN_HandleTypeDef* _hfdcan, uint32_t fifox) {
    static FDCAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (
        HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, can_rx_buff);
        can* instance = can::get_instance(_hfdcan, rxconf.Identifier);
        if (instance) {
            auto length = rxconf.DataLength;
            if (length > 0xff)
                length >>= 16;
            instance->OnRxFifoCallback(can_rx_buff, length);
            return;
        }
    }
}
} // namespace bsp
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    bsp::CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
    bsp::CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}