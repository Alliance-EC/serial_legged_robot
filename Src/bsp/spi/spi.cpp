#include "spi.hpp"

namespace bsp {
// 定义静态成员变量
std::array<spi*, spi::MAX_SPI_INSTANCES> spi::spi_instances_ = {};
} // namespace bsp
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    bsp::spi* instance = bsp::spi::get_instance(hspi);
    if (instance) {
        instance->onRxCpltCallback();
    }
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    HAL_SPI_RxCpltCallback(hspi); // 直接调用接收完成的回调函数
}
