#pragma once

#include <algorithm>
#include <bsp/dwt/dwt.h>
#include <cfloat>
#include <cmath>
#include <cstdint>

namespace tool {
struct PID_params {
    double Kp            = 0;
    double Ki            = 0;
    double Kd            = 0;
    double MaxOut        = 0;
    double IntegralLimit = 0;
    double DeadBand      = 0;
    double Expect_dt     = -1;
};
struct PID_adjust_ptr {
    double* Kp_ptr;
    double* Ki_ptr;
    double* Kd_ptr;
};
class PID {
public:
    explicit PID(PID_params params)
        : Kp_(params.Kp)
        , Ki_(params.Ki)
        , Kd_(params.Kd)
        , MaxOut_(params.MaxOut)
        , IntegralLimit_(params.IntegralLimit)
        , DeadBand_(params.DeadBand)
        , Expect_dt_(params.Expect_dt) {
        DWT_GetDeltaT(&DWT_CNT_);
    }
    ~PID() = default;
    double update(double setpoint, double feedback, double outside_differential = 0) {
        auto isNotZero = [](double value) { return std::abs(value) > DBL_EPSILON; };
        if (std::isnan(setpoint) || std::isnan(feedback))
            return 0;
        // 获取两次pid计算的时间间隔,用于积分和微分
        if (Expect_dt_ > 0)
            dt_ = DWT_GetDeltaT64_Expect(&DWT_CNT_, Expect_dt_);
        else
            dt_ = DWT_GetDeltaT64(&DWT_CNT_);

        double error = setpoint - feedback;
        double output;
        if (std::abs(error) > DeadBand_) {
            double pout = Kp_ * error;
            double dout = 0;
            if (isNotZero(outside_differential))
                dout = Kd_ * outside_differential;
            else
                dout = Kd_ * (error - last_error_) / dt_;

            integral_ += Ki_ * error * dt_;
            integral_   = std::clamp(integral_, -IntegralLimit_, IntegralLimit_);
            double iout = integral_;
            output      = pout + iout + dout;
            output      = std::clamp(output, -MaxOut_, MaxOut_);
        } else {
            output    = 0;
            integral_ = 0;
        }
        last_error_ = error;
        return output;
    }
    void IntegralNoNagtive() { integral_ = std::max(integral_, 0.0); }
    void SetIntegral(double integral) { this->integral_ = integral; }
    void ChangeParams(PID_params params) {
        Kp_            = params.Kp;
        Ki_            = params.Ki;
        Kd_            = params.Kd;
        MaxOut_        = params.MaxOut;
        IntegralLimit_ = params.IntegralLimit;
        DeadBand_      = params.DeadBand;
    }
    void SetKp(double Kp) { Kp_ = Kp; }
    void SetKi(double Ki) { Ki_ = Ki; }
    void SetKd(double Kd) { Kd_ = Kd; }

private:
    double Kp_            = 0;
    double Ki_            = 0;
    double Kd_            = 0;
    double MaxOut_        = 0;
    double IntegralLimit_ = 0;
    double DeadBand_      = 0;

    double last_error_ = 0;
    double integral_   = 0;

    uint32_t DWT_CNT_ = 0;
    double dt_        = 0;
    double Expect_dt_ = -1;
};

} // namespace tool