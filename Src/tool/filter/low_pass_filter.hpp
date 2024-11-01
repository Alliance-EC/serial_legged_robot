#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace tool::filter {
// Type Definitions
struct dsp_BiquadFilter_0 {
    double W0_FILT_STATES[2];
    int W1_PreviousNumChannels;
    double P0_ICRTP;
    double P1_RTP1COEFF[3];
    double P2_RTP2COEFF[2];
    double P3_RTP3COEFF[2];
    bool P4_RTP_COEFF3_BOOL[2];
};

class LowPassFilter {
public:
    explicit LowPassFilter(uint32_t filter_freq) {
        switch (filter_freq) {
        case 50: LowPassFilter_init_50Hz_2order(); break;
        case 100: LowPassFilter_init_100Hz_2order(); break;
        default: // printf("Invalid filter frequency");
            break;
        }
    }

    double update(double x) {
        if (std::isnan(x)) {
            // printf("Warning: Input is NaN\n");
            return x;
        }
        double denAccum;
        double tmpState;
        double y;
        // System object Outputs function: dsp.BiquadFilter
        denAccum = Hd_.P3_RTP3COEFF[0] * x;
        denAccum -= Hd_.P2_RTP2COEFF[0] * Hd_.W0_FILT_STATES[0];
        tmpState = denAccum - Hd_.P2_RTP2COEFF[1] * Hd_.W0_FILT_STATES[1];
        denAccum = Hd_.P1_RTP1COEFF[0] * tmpState;
        denAccum += Hd_.W0_FILT_STATES[0] * Hd_.P1_RTP1COEFF[1];
        y = denAccum + Hd_.W0_FILT_STATES[1] * Hd_.P1_RTP1COEFF[2];

        Hd_.W0_FILT_STATES[1] = Hd_.W0_FILT_STATES[0];
        Hd_.W0_FILT_STATES[0] = tmpState;

        return y;
    }

private:
    void LowPassFilter_init_100Hz_2order() {
        //  设计滤波器系数时使用了以下代码:
        //
        //  N    = 2;     % Order
        //  F3dB = 100;   % 3-dB Frequency
        //  Fs   = 1000;  % Sampling Frequency
        //
        //  h = fdesign.lowpass('n,f3db', N, F3dB, Fs);
        //
        //  Hd = design(h, 'butter', ...
        //      'SystemObject', true,...
        //       UseLegacyBiquadFilter=true);
        // System object Constructor function: dsp.BiquadFilter
        Hd_.P0_ICRTP        = 0.0;
        Hd_.P1_RTP1COEFF[0] = 1.0;
        Hd_.P1_RTP1COEFF[1] = 2.0;
        Hd_.P1_RTP1COEFF[2] = 1.0;
        // System object Initialization function: dsp.BiquadFilter
        Hd_.P2_RTP2COEFF[0]       = -1.1429805025399;
        Hd_.P3_RTP3COEFF[0]       = 0.0674552738890719;
        Hd_.P4_RTP_COEFF3_BOOL[0] = true;
        Hd_.W0_FILT_STATES[0]     = Hd_.P0_ICRTP;
        Hd_.P2_RTP2COEFF[1]       = 0.412801598096189;
        Hd_.P3_RTP3COEFF[1]       = 0.0;
        Hd_.P4_RTP_COEFF3_BOOL[1] = false;
        Hd_.W0_FILT_STATES[1]     = Hd_.P0_ICRTP;
    }
    void LowPassFilter_init_50Hz_2order() {
        //  设计滤波器系数时使用了以下代码:
        //
        //  N    = 2;     % Order
        //  F3dB = 50;    % 3-dB Frequency
        //  Fs   = 1000;  % Sampling Frequency
        //
        //  h = fdesign.lowpass('n,f3db', N, F3dB, Fs);
        //
        //  Hd = design(h, 'butter', ...
        //      'SystemObject', true,...
        //       UseLegacyBiquadFilter=true);
        // System object Constructor function: dsp.BiquadFilter
        Hd_.P0_ICRTP        = 0.0;
        Hd_.P1_RTP1COEFF[0] = 1.0;
        Hd_.P1_RTP1COEFF[1] = 2.0;
        Hd_.P1_RTP1COEFF[2] = 1.0;
        // System object Initialization function: dsp.BiquadFilter
        Hd_.P2_RTP2COEFF[0]       = -1.56101807580072;
        Hd_.P3_RTP3COEFF[0]       = 0.0200833655642112;
        Hd_.P4_RTP_COEFF3_BOOL[0] = true;
        Hd_.W0_FILT_STATES[0]     = Hd_.P0_ICRTP;
        Hd_.P2_RTP2COEFF[1]       = 0.641351538057563;
        Hd_.P3_RTP3COEFF[1]       = 0.0;
        Hd_.P4_RTP_COEFF3_BOOL[1] = false;
        Hd_.W0_FILT_STATES[1]     = Hd_.P0_ICRTP;
    }

    dsp_BiquadFilter_0 Hd_;
};
} // namespace tool::filter