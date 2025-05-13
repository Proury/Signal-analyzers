#include "fft.h"




//float fft_outputbuf[FFT_LENGTH * 2];  // ????:???????


// ??????
//SignalInfo_t capture_and_FFT(uint16_t len, uint16_t *adc_buf, uint32_t sample_rate)
//{
//    SignalInfo_t info = {0};
//    
//    // Step 1:ADC???????? [0,1] ??
//    for (int i = 0; i < len; i++) {
//        fft_inputbuf[i] = (float)adc_buf[i] / 4095.0f;
//    }

//    // Step 2:??FFT
//    arm_rfft_fast_f32(&fft_handler, fft_inputbuf, fft_outputbuf, 0);

//    // Step 3:????(???)
//    float max_value = 0.0f;
//    uint32_t max_index = 0;
//    for (int i = 1; i < len / 2; i++)  // DC????,?1??
//    {
//        float real = fft_outputbuf[2 * i];
//        float imag = fft_outputbuf[2 * i + 1];
//        float magnitude = sqrtf(real * real + imag * imag);

//        if (magnitude > max_value) {
//            max_value = magnitude;
//            max_index = i;
//        }
//    }

//    // Step 4:????
//    info.main_freq = (float)max_index * sample_rate / len;
//    info.max_magnitude = max_value;

//    return info;
//}
