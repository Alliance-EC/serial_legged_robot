#include "daemon.hpp"
#include "bsp/dwt/dwt.h"
namespace tool {
std::array<daemon*, daemon::MAX_DAEMON_INSTANCES> daemon::daemon_instances_ = {};
}
static float watch_dt;
// 此函数在tim17中断中调用 every 10ms
extern "C" void daemon_tim_callback() {
    static uint32_t cnt_last = 0;
    watch_dt                 = DWT_GetDeltaT(&cnt_last);
    tool::daemon::check_all();
}