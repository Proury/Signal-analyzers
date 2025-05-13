#ifndef __FFT_UTILS_H__
#define __FFT_UTILS_H__

#include "arm_math.h"
#include <math.h>
              // ???2^n(256/512/1024...)
//#define SAMPLING_RATE 20000         // ADC???,??Hz

typedef struct {
    float dc_offset;    // ????
    float main_freq;    // ??
    float second_freq;  // ??
    float main_amp;     // ????
    float second_amp;   // ????
} SignalInfo_t;



#endif