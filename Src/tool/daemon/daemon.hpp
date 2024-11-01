#pragma once
#include "bsp/dwt/dwt.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
namespace tool {
class daemon {
public:
    explicit daemon(float dt, const std::function<void()>& callback = nullptr) {
        register_instance(this);
        if (dt < 0.01) {
            while (true)
                ;
            // error dt too small
        }
        dt_       = dt;
        callback_ = callback;
    }
    ~daemon() { unregister_instance(this); }
    void set_dt(float dt) { dt_ = dt; }
    void set_callback(const std::function<void()>& callback) { callback_ = callback; }

    void reload() { last_reload_time_ = DWT_GetTimeline_s(); }

    static void check_all() {
        for (auto& i : daemon_instances_) {
            if (i) {
                if (DWT_GetTimeline_s() - i->last_reload_time_ >= i->dt_) {
                    if (i->callback_)
                        i->callback_();
                    i->last_reload_time_ = DWT_GetTimeline_s();
                }
            }
        }
    }

private:
    float dt_                       = 0;
    float last_reload_time_         = 0;
    std::function<void()> callback_ = nullptr;

    static void register_instance(daemon* instance) {
        auto it = std::find(daemon_instances_.begin(), daemon_instances_.end(), nullptr);
        if (it != daemon_instances_.end()) {
            *it = instance;
        } else {
            while (true)
                ;
            // error
        }
    }
    static void unregister_instance(daemon* instance) {
        auto it = std::find(daemon_instances_.begin(), daemon_instances_.end(), instance);
        if (it != daemon_instances_.end()) {
            *it = nullptr;
        }
    }
    static constexpr uint16_t MAX_DAEMON_INSTANCES = 25;
    static std::array<daemon*, MAX_DAEMON_INSTANCES> daemon_instances_;
};
} // namespace tool