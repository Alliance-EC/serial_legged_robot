#include "cmsis_os2.h"
#include "controller/desire_set.hpp"
#include "controller/send_process.hpp"
#include "device/Dji_motor/DJI_motor.hpp"
#include "device/RC/remote_control.hpp"
#include "device/super_cap/super_cap.hpp"
#include "module/DM8009/DM8009.hpp"
#include "module/IMU/IMU.hpp"
#include "module/M3508/M3508.hpp"
#include "module/referee/status.hpp"
#include "observer/observer.hpp"
#include "system_parameters.hpp"
#include "tool/time_counter.hpp"
#include <numbers>

using namespace device;
using namespace module;
namespace app {
static module::IMU* IMU_instance;
static device::remote_control* RC_instance;

static std::array<module::M3508*, 6> M3508_instance;

static device::DjiMotor_sender* m3508_sender_instance_1;
static device::DjiMotor_sender* m3508_sender_instance_2;
static auto observer_instance   = observer::observer::GetInstance();
static auto desire_instance     = controller::DesireSet::GetInstance();
static auto controller_instance = controller::Controller::GetInstance();
static auto sender_instance     = controller::SendProcess::GetInstance();

static chassis_mode chassis_mode_;

static volatile double x_states_watch[10];

void Init() {
    __disable_irq();
    IMU_instance = new module::IMU(IMU_params(this_board).set_angle_offset({0.0369768739, 0, 0}));
    RC_instance  = new device::remote_control(RC_params(this_board));

    for (uint8_t i = 0; i < 6; ++i) {
        auto params = module::M3508_params();
        params.Dji_common.set_can_instance(&hfdcan1).set_rx_id(toU32(M3508_ID::ID1) + i);
        M3508_instance[i] = new module::M3508(params);
        /*电机掉线保护相关*/
        M3508_instance[i]->SetOfflineCallback(
            std::bind(&controller::DesireSet::CanLost, desire_instance));
    }
    m3508_sender_instance_1 = new device::DjiMotor_sender(
        DjiMotor_params().set_can_instance(&hfdcan1).set_tx_id(toU32(M3508_sendID::ID1)));

    m3508_sender_instance_2 = new device::DjiMotor_sender(
        DjiMotor_params().set_can_instance(&hfdcan1).set_tx_id(toU32(M3508_sendID::ID2)));
    /*设定电机减速比，初始角度*/

    M3508_instance[wheel_L]->configure(false, DjiMotorType::M3508_15);
    M3508_instance[wheel_R]->configure(false, DjiMotorType::M3508_15);

    M3508_instance[leg_LF]->configure(true, DjiMotorType::M3508_19);
    M3508_instance[leg_LB]->configure(true, DjiMotorType::M3508_19);
    M3508_instance[leg_RF]->configure(true, DjiMotorType::M3508_19);
    M3508_instance[leg_RB]->configure(true, DjiMotorType::M3508_19);

    observer_instance->Init(IMU_instance, M3508_instance, &chassis_mode_);
    desire_instance->Init(&IMU_instance->output_vector, &RC_instance->data, &chassis_mode_);
    controller_instance->Init(IMU_instance, M3508_instance, &chassis_mode_);
    sender_instance->Init(
        M3508_instance, m3508_sender_instance_1, m3508_sender_instance_2, &chassis_mode_);

    __enable_irq();
}
static double dt_watch;
extern "C" void main_task_func(void* argument) {
    Init();
    while (true) {
        uint32_t wakeUpTime = osKernelGetTickCount();
        TimeElapse(dt_watch, [&]() { // 在release中删除
            desire_instance->update();
            observer_instance->update();
            controller_instance->update();
            sender_instance->Send();
        });

        for (uint8_t i = 0; i < 10; ++i) {
            x_states_watch[i] = observer_instance->x_states_[i];
        }
        // watch_data = M3508_instance[0]->get_angle();
        osDelayUntil(wakeUpTime + 1);
    }
}
} // namespace app