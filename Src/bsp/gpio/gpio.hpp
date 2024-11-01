#pragma once
#include "gpio.h"
#include "stm32h7xx_hal_gpio.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>

namespace bsp {
enum class GPIO_EXTI_MODE : uint8_t {
    RISING,
    FALLING,
    RISING_FALLING,
    NONE,
};
struct gpio_params {
    GPIO_TypeDef* GPIOx      = nullptr;              // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state  = GPIO_PIN_RESET;       // 引脚状态,Set,Reset not frequently used
    GPIO_EXTI_MODE exti_mode = GPIO_EXTI_MODE::NONE; // 外部中断模式 not frequently used
    uint16_t GPIO_Pin        = 0; // 引脚号,@note 这里的引脚号是GPIO_PIN_0,GPIO_PIN_1...
    std::function<void()> callback = nullptr;
};
class gpio {
public:
    explicit gpio(const gpio_params& params)
        : GPIOx_(params.GPIOx)
        , pin_state_(params.pin_state)
        , exti_mode_(params.exti_mode)
        , GPIO_Pin_(params.GPIO_Pin)
        , callback_(params.callback) {
        register_instance(this);
    }
    ~gpio() { unregister_instance(this); }

    void SetState(GPIO_PinState state) {
        if (pin_state_ != state) {
            pin_state_ = state;
            HAL_GPIO_WritePin(GPIOx_, GPIO_Pin_, pin_state_);
        }
    }
    void Toggle() { HAL_GPIO_TogglePin(GPIOx_, GPIO_Pin_); }

    GPIO_PinState GetState() { return HAL_GPIO_ReadPin(GPIOx_, GPIO_Pin_); }

    void OnEXTICallback() {
        if (callback_)
            callback_();
    }
    void SetCallback(const std::function<void()>& callback) { callback_ = callback; }
    static gpio* get_instance(uint16_t GPIO_Pin) {
        for (auto* instance : gpio_instances_) {
            if (instance && instance->GPIO_Pin_ == GPIO_Pin) {
                return instance;
            }
        }
        return nullptr;
    }

private:
    GPIO_TypeDef* GPIOx_;         // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state_;     // 引脚状态,Set,Reset;not frequently used
    GPIO_EXTI_MODE exti_mode_;    // 外部中断模式 not frequently used
    uint16_t GPIO_Pin_;           // 引脚号,
    std::function<void()> callback_;

    static constexpr size_t MAX_GPIO_INSTANCES = 16;
    static std::array<gpio*, MAX_GPIO_INSTANCES> gpio_instances_;

    static void register_instance(gpio* instance) {
        auto it = std::find(gpio_instances_.begin(), gpio_instances_.end(), nullptr);
        if (it != gpio_instances_.end()) {
            *it = instance;
        }
    }

    static void unregister_instance(gpio* instance) {
        auto it = std::find(gpio_instances_.begin(), gpio_instances_.end(), instance);
        if (it != gpio_instances_.end()) {
            *it = nullptr;
        }
    }

    friend void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
};
} // namespace bsp