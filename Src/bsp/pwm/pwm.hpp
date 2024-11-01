#pragma once
#include "tim.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>

namespace bsp {
struct pwm_params {
    TIM_HandleTypeDef* htim        = nullptr;                  // TIM句柄
    HAL_TIM_ActiveChannel channel  = HAL_TIM_ACTIVE_CHANNEL_1; // 通道
    float period                   = 0;                        // 周期
    float dutyratio                = 0;                        // 占空比
    std::function<void()> callback = nullptr;                  // DMA传输完成回调函数
};
class pwm {
public:
    explicit pwm(const pwm_params& params)
        : htim_(params.htim)
        , channel_(params.channel)
        , period_(params.period)
        , dutyratio_(params.dutyratio)
        , callback_(params.callback) {
        register_instance(this);
        tclk_ = SelectTclk(htim_);
        // 启动PWM
        HAL_TIM_PWM_Start(htim_, channel_);
        SetPeriod(period_);
        SetDutyRatio(dutyratio_);
    }
    ~pwm() { unregister_instance(this); }
    void Start() { HAL_TIM_PWM_Start(htim_, channel_); }
    void Stop() { HAL_TIM_PWM_Stop(htim_, channel_); }
    void SetPeriod(float period) {
        period_ = period;
        __HAL_TIM_SetAutoreload(
            htim_, static_cast<uint16_t>(period * ((tclk_) / (htim_->Init.Prescaler + 1.0f))));
    }
    void SetDutyRatio(float dutyratio) {
        dutyratio_ = dutyratio;
        __HAL_TIM_SetCompare(
            htim_, channel_, static_cast<uint16_t>(dutyratio_ * htim_->Instance->ARR));
    }
    void StartDMA(uint32_t* pData, uint16_t Length) {
        SCB_CleanInvalidateDCache_by_Addr(pData, static_cast<int32_t>(Length));
        HAL_TIM_PWM_Start_DMA(htim_, channel_, pData, Length);
    }

    void OnPulseFinishedCallback() {
        if (callback_)
            callback_();
    }
    void SetCallback(const std::function<void()>& callback) { callback_ = callback; }
    static pwm* get_instance(TIM_HandleTypeDef* htim, uint32_t channel) {
        for (auto* instance : pwm_instances_) {
            if (instance && instance->htim_ == htim && instance->channel_ == channel) {
                return instance;
            }
        }
        return nullptr;
    }

private:
    TIM_HandleTypeDef* htim_;        // TIM句柄
    HAL_TIM_ActiveChannel channel_;  // 通道
    uint32_t tclk_;                  // 时钟频率
    float period_;                   // 周期
    float dutyratio_;                // 占空比
    std::function<void()> callback_; // DMA传输完成回调函数

    static uint32_t SelectTclk(TIM_HandleTypeDef* htim) {
        uintptr_t tclk_temp = ((uintptr_t)(htim->Instance));
        if ((tclk_temp <= (APB1PERIPH_BASE + 0x2000UL))
            && (tclk_temp >= (APB1PERIPH_BASE + 0x0000UL))) {
            return (
                HAL_RCC_GetPCLK1Freq()
                * (D1CorePrescTable[(RCC->CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos] == 0
                       ? 1
                       : 2));
        } else if (
            ((tclk_temp <= (APB2PERIPH_BASE + 0x0400UL))
             && (tclk_temp >= (APB2PERIPH_BASE + 0x0000UL)))
            || ((tclk_temp <= (APB2PERIPH_BASE + 0x4800UL))
                && (tclk_temp >= (APB2PERIPH_BASE + 0x4000UL)))) {
            return (
                HAL_RCC_GetPCLK2Freq()
                * (D1CorePrescTable[(RCC->CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos] == 0
                       ? 1
                       : 2));
        }
        return 0;
    }

    static constexpr size_t MAX_PWM_INSTANCES = 16;
    static std::array<pwm*, MAX_PWM_INSTANCES> pwm_instances_;

    static void register_instance(pwm* instance) {
        auto it = std::find(pwm_instances_.begin(), pwm_instances_.end(), nullptr);
        if (it != pwm_instances_.end()) {
            *it = instance;
        }
    }
    static void unregister_instance(pwm* instance) {
        auto it = std::find(pwm_instances_.begin(), pwm_instances_.end(), instance);
        if (it != pwm_instances_.end()) {
            *it = nullptr;
        }
    }
    friend void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim);
};
} // namespace bsp