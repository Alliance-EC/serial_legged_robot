#pragma once

#include "bsp/dwt/dwt.h"
#include "usbd_cdc_if.h"
#include <functional>

namespace bsp {
class usb {
public:
    static usb* GetInstance() {
        static auto instance = new usb();
        return instance;
    }
    void RegisterTxcallback(const std::function<void()>& callback) { Txcallback_ = callback; }
    void RegisterRxcallback(const std::function<void()>& callback) { Rxcallback_ = callback; }
    static void Transmit(uint8_t* tx_buffer, uint16_t tx_size) {
        CDC_Transmit_HS(tx_buffer, tx_size);
    }
    void Txcallback() {
        if (Txcallback_ != nullptr) {
            Txcallback_();
        }
    }
    void Rxcallback() {
        if (Rxcallback_ != nullptr) {
            Rxcallback_();
        }
    }
    static void Refresh() {
        // 重新枚举usb设备
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
        DWT_Delay(0.1f);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
        DWT_Delay(0.1f);
    }
    void SetRxBuffer(uint8_t* rx_buffer, uint32_t rx_size) {
        this->rx_buffer = rx_buffer;
        this->rx_size   = rx_size;
    }

private:
    usb()                                = default; // 禁止外部构造
    ~usb()                               = default; // 禁止外部析构
    usb(const usb& usb)                  = delete;  // 禁止外部拷贝构造
    const usb& operator=(const usb& usb) = delete;  // 禁止外部赋值操作

    std::function<void()> Txcallback_ = nullptr;
    std::function<void()> Rxcallback_ = nullptr;
    uint8_t* rx_buffer                = nullptr;
    uint32_t rx_size                  = 0;
};
} // namespace bsp
