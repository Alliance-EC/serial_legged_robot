#pragma once

#include "usart.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace bsp {
enum class UART_TRANSFER_MODE : uint8_t {
    POLLING = 0,
    IT,
    DMA,
};

struct uart_params {
    UART_HandleTypeDef* uart_handle = nullptr; // 实例对应的usart_handle
    uint16_t recv_buff_size         = 0;       // 模块接收一包数据的大小
    std::function<void()> callback  = nullptr;
};

class uart {
public:
    explicit uart(const uart_params& params)
        : uart_handle_(params.uart_handle)
        , rx_size_(params.recv_buff_size)
        , callback_(params.callback) {
        rx_buffer_ = new uint8_t[rx_size_];
        register_instance(this);
        // 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收
        ReceiveDMAAuto();
    }

    ~uart() {
        delete[] rx_buffer_;
        unregister_instance(this);
    }
    bool IsReady() {
        if (uart_handle_->gState | HAL_UART_STATE_BUSY_TX)
            return false;
        else
            return true;
    }
    void Send(uint8_t* send_buf, uint16_t send_size, UART_TRANSFER_MODE mode) {
        switch (mode) {
        case UART_TRANSFER_MODE::POLLING:
            HAL_UART_Transmit(uart_handle_, send_buf, send_size, 100);
            break;
        case UART_TRANSFER_MODE::IT: HAL_UART_Transmit_IT(uart_handle_, send_buf, send_size); break;
        case UART_TRANSFER_MODE::DMA:
            SCB_CleanInvalidateDCache_by_Addr(
                reinterpret_cast<uint32_t*>(send_buf), static_cast<int32_t>(send_size));
            HAL_UART_Transmit_DMA(uart_handle_, send_buf, send_size);
            break;
        default:
            while (true)
                ; // illegal mode! check your code context!
                  // 检查定义instance的代码上下文,可能出现指针越界
            break;
        }
    }

    void OnRxCpltCallback() {
        if (callback_)
            callback_();
    }
    void SetCallback(const std::function<void()>& callback) { callback_ = callback; }

    void ReceiveDMAAuto() {
        SCB_CleanInvalidateDCache_by_Addr(
            reinterpret_cast<uint32_t*>(rx_buffer_), static_cast<int32_t>(rx_size_));
        HAL_UARTEx_ReceiveToIdle_DMA(uart_handle_, rx_buffer_, rx_size_);
        // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
        // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
        // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
        __HAL_DMA_DISABLE_IT(uart_handle_->hdmarx, DMA_IT_HT);
    }
    uint8_t* GetRxBuffer() { return rx_buffer_; }

    static uart* get_instance(UART_HandleTypeDef* usart_handle) {
        for (auto* instance : uart_instances_) {
            if (instance && instance->uart_handle_ == usart_handle) {
                return instance;
            }
        }
        return nullptr;
    }
    [[nodiscard]] uint16_t GetTrueRxSize() const { return rx_size_from_register; }
    void SetTrueRxSize(uint16_t size) { rx_size_from_register = size; }

private:
    UART_HandleTypeDef* uart_handle_;   // 实例对应的usart_handle
    uint8_t* rx_buffer_;                // 预先定义的最大buff大小
    uint16_t rx_size_;                  // 模块接收一包数据的大小
    uint16_t rx_size_from_register = 0; // 空闲中断返回接收的数据长度
    std::function<void()> callback_;

    static constexpr size_t MAX_UART_INSTANCES = 16;
    static std::array<uart*, MAX_UART_INSTANCES> uart_instances_;

    static void register_instance(uart* instance) {
        auto it = std::find(uart_instances_.begin(), uart_instances_.end(), nullptr);
        if (it != uart_instances_.end()) {
            *it = instance;
        }
    }
    static void unregister_instance(uart* instance) {
        auto it = std::find(uart_instances_.begin(), uart_instances_.end(), instance);
        if (it != uart_instances_.end()) {
            *it = nullptr;
        }
    }
};
} // namespace bsp