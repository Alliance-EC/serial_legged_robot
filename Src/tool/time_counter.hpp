#pragma once
#include "bsp/dwt/dwt.h"

namespace tool {
// usage: TimeElapse(dt, [&](){
//            code;
//        });
template <typename Func>
void TimeElapse(auto& dt, Func&& code) {
    auto start = DWT_GetTimeline_s();
    code();
    dt = DWT_GetTimeline_s() - start;
}

} // namespace tool