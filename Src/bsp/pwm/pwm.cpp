#include "pwm.hpp"

namespace bsp {
std::array<pwm*, pwm::MAX_PWM_INSTANCES> pwm::pwm_instances_;
}
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    bsp::pwm* instance = bsp::pwm::get_instance(htim, htim->Channel);
    if (instance) {
        instance->OnPulseFinishedCallback();
    }
}