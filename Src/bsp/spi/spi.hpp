#pragma once
#include "spi.h"
#include "stm32h7xx_hal_gpio.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace bsp {
enum class SPI_TXRX_MODE : uint8_t {
    POLLING = 0,
    IT      = 1,
    DMA     = 2,
};

struct spi_params {
    SPI_HandleTypeDef* spi_handle = nullptr;
    GPIO_TypeDef* GPIOx           = nullptr; // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin               = 0; // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
    SPI_TXRX_MODE spi_work_mode   = SPI_TXRX_MODE::POLLING;
    std::function<void()> callback = nullptr;
};

class spi {
public:
    explicit spi(const spi_params& params)
        : spi_handle_(params.spi_handle)
        , GPIOx_(params.GPIOx)
        , cs_pin_(params.cs_pin)
        , spi_work_mode_(params.spi_work_mode)
        , callback_(params.callback) {
        register_instance(this);
    }

    ~spi() { unregister_instance(this); }

    void SetMode(SPI_TXRX_MODE mode) {
        if (spi_work_mode_ != mode) {
            spi_work_mode_ = mode;
        }
    }

    void Transmit(uint8_t* tx_buffer, uint8_t tx_size) {
        HAL_GPIO_WritePin(GPIOx_, cs_pin_, GPIO_PIN_RESET);
        switch (spi_work_mode_) {
        case SPI_TXRX_MODE::POLLING:
            HAL_SPI_Transmit(spi_handle_, tx_buffer, tx_size, 1000);
            HAL_GPIO_WritePin(GPIOx_, cs_pin_, GPIO_PIN_SET);
            break;
        case SPI_TXRX_MODE::IT: HAL_SPI_Transmit_IT(spi_handle_, tx_buffer, tx_size); break;
        case SPI_TXRX_MODE::DMA:
            SCB_CleanInvalidateDCache_by_Addr(
                reinterpret_cast<uint32_t*>(tx_buffer), static_cast<int32_t>(tx_size));
            HAL_SPI_Transmit_DMA(spi_handle_, tx_buffer, tx_size);
            break;
        default:
            while (true)
                ;                      // error mode!
        }
    }

    void Receive(uint8_t* rx_buffer, uint8_t rx_size) {
        HAL_GPIO_WritePin(GPIOx_, cs_pin_, GPIO_PIN_RESET);
        switch (spi_work_mode_) {
        case SPI_TXRX_MODE::POLLING:
            HAL_SPI_Receive(spi_handle_, rx_buffer, rx_size, 1000);
            HAL_GPIO_WritePin(GPIOx_, cs_pin_, GPIO_PIN_SET);
            break;
        case SPI_TXRX_MODE::IT:
            this->rx_buffer_ = rx_buffer;
            this->rx_size_   = rx_size;
            HAL_SPI_Receive_IT(spi_handle_, rx_buffer, rx_size);
            break;
        case SPI_TXRX_MODE::DMA:
            this->rx_buffer_ = rx_buffer;
            this->rx_size_   = rx_size;
            SCB_CleanInvalidateDCache_by_Addr(
                reinterpret_cast<uint32_t*>(rx_buffer), static_cast<int32_t>(rx_size));
            HAL_SPI_Receive_DMA(spi_handle_, rx_buffer, rx_size);
            break;
        default:
            while (true)
                ;                      // error mode!
        }
    }

    void TransmitReceive(uint8_t* rx_buffer, uint8_t* tx_buffer, uint8_t size) {
        this->rx_buffer_ = rx_buffer;
        this->rx_size_   = size;
        HAL_GPIO_WritePin(GPIOx_, cs_pin_, GPIO_PIN_RESET);
        switch (spi_work_mode_) {
        case SPI_TXRX_MODE::POLLING:
            HAL_SPI_TransmitReceive(spi_handle_, tx_buffer, rx_buffer, size, 1000);
            HAL_GPIO_WritePin(GPIOx_, cs_pin_, GPIO_PIN_SET);
            break;
        case SPI_TXRX_MODE::IT:
            HAL_SPI_TransmitReceive_IT(spi_handle_, tx_buffer, rx_buffer, size);
            break;
        case SPI_TXRX_MODE::DMA:
            SCB_CleanInvalidateDCache_by_Addr(
                reinterpret_cast<uint32_t*>(tx_buffer), static_cast<int32_t>(size));
            SCB_CleanInvalidateDCache_by_Addr(
                reinterpret_cast<uint32_t*>(rx_buffer), static_cast<int32_t>(size));
            HAL_SPI_TransmitReceive_DMA(spi_handle_, tx_buffer, rx_buffer, size);
            break;
        default:
            while (true)
                ;                      // error mode!
        }
    }

    void onRxCpltCallback() {
        if (callback_) {
            callback_();
        }
    }
    void SetCallback(const std::function<void()>& callback) { callback_ = callback; }
    static spi* get_instance(SPI_HandleTypeDef* hspi) {
        for (auto* instance : spi_instances_) {
            if (instance && instance->spi_handle_ == hspi) {
                if (HAL_GPIO_ReadPin(instance->GPIOx_, instance->cs_pin_) == GPIO_PIN_RESET) {
                    HAL_GPIO_WritePin(instance->GPIOx_, instance->cs_pin_, GPIO_PIN_SET);
                    return instance;
                }
            }
        }
        return nullptr;
    }

private:
    SPI_HandleTypeDef* spi_handle_;    // SPI外设handle
    GPIO_TypeDef* GPIOx_;              // 片选信号对应的GPIO
    uint16_t cs_pin_;                  // 片选信号对应的引脚号
    SPI_TXRX_MODE spi_work_mode_;      // 传输工作模式
    uint8_t rx_size_    = 0;           // 本次接收的数据长度
    uint8_t* rx_buffer_ = nullptr;     // 本次接收的数据缓冲区
    std::function<void()> callback_;

    static constexpr size_t MAX_SPI_INSTANCES = 10;
    static std::array<spi*, MAX_SPI_INSTANCES> spi_instances_;

    static void register_instance(spi* instance) {
        auto it = std::find(spi_instances_.begin(), spi_instances_.end(), nullptr);
        if (it != spi_instances_.end()) {
            *it = instance;
        }
    }

    static void unregister_instance(spi* instance) {
        auto it = std::find(spi_instances_.begin(), spi_instances_.end(), instance);
        if (it != spi_instances_.end()) {
            *it = nullptr;
        }
    }

    friend void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi);
};

} // namespace bsp
