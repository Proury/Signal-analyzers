/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "lcd.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  WAVE_SINE,
  WAVE_SQUARE,
  WAVE_TRIANGLE
} WaveType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LENGTH        4096
#define CURRENT_SAMPLING_FREQ 400000
#define WAVE_TABLE_LEN 100
#define DEBOUNCE_TIME_MS 50
#define SCREEN_TOP    146
#define SCREEN_BOTTOM 310
#define SCREEN_CENTER 228
#define MIN_FREQ_HZ 100
#define MAX_FREQ_HZ 10000
#define FREQ_STEP_HZ 100
#define FILTER_ALPHA 0.22566f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
arm_cfft_radix4_instance_f32 scfft;
float FFT_InputBuf[FFT_LENGTH*2];
float FFT_OutputBuf[FFT_LENGTH];
uint16_t ADC_1_Value_DMA[FFT_LENGTH] = {0};
float main_freq;
float main_vpp;
WaveType wave_type;
WaveType current_wave = WAVE_SINE;
uint16_t sin_table[WAVE_TABLE_LEN];
uint16_t square_table[WAVE_TABLE_LEN];
uint16_t triangle_table[WAVE_TABLE_LEN];
volatile uint8_t key0_pressed = 0; // KEY0 (PE4) - Waveform switch
volatile uint8_t key1_pressed = 0; // KEY1 (PE2) - Frequency decrease
volatile uint8_t key3_pressed = 0; // KEY3 (PE3) - Frequency increase
volatile uint32_t last_press_time = 0;
u16 wave_data[210];
uint32_t current_freq_hz = MIN_FREQ_HZ;
volatile uint8_t data_ready = 0;
uint16_t filtered_data[FFT_LENGTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Generate_Sin_Table(void);
void Generate_Square_Table(void);
void Generate_Triangle_Table(void);
void Set_DAC_Output_Frequency(uint32_t target_freq_hz);
void Oscilloscope_Display_Init(void);
void Oscilloscope_ClearWaveform(void);
void Oscilloscope_DrawWaveform(u16 *wave_data);
void Oscilloscope_GenerateSineWave(u16 *wave_data, u16 length, float amplitude, float frequency);
void Oscilloscope_UpdateWaveform(u16 *wave_data);
void Oscilloscope_Test(void);
void Low_Pass_Filter(uint16_t *input, uint16_t *output, uint16_t length, float alpha);
void Convert_ADC_Data_To_Waveform(uint16_t *adc_data, u16 *filtered_data, u16 *wave_data, uint16_t adc_length, uint16_t wave_length);
void CalculateFFT(uint16_t *adc_data, float *fft_input, float *fft_output, 
                 arm_cfft_radix4_instance_f32 *fft_instance, uint32_t fft_length, 
                 uint32_t sampling_freq, float *freq, float *vpp, WaveType *wave);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Generate_Sin_Table(void) {
    float vref = 3.3f;
    float counts_per_volt = 4095.0f / vref;
    float peak_counts = 0.512f * counts_per_volt; // Calibrated for 1000 mVpp
    for (int i = 0; i < WAVE_TABLE_LEN; i++) {
        float sin_value = sinf(2.0f * M_PI * i / WAVE_TABLE_LEN);
        float scaled_value = peak_counts * sin_value;
        float dac_value = roundf((counts_per_volt * 1.65f) + scaled_value);
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 4095) dac_value = 4095;
        sin_table[i] = (uint16_t)dac_value;
    }
}

void Generate_Square_Table(void) {
    float vref = 3.3f;
    float counts_per_volt = 4095.0f / vref;
    float peak_counts = 0.512f * counts_per_volt;
    for (int i = 0; i < WAVE_TABLE_LEN; i++) {
        float phase = 2.0f * M_PI * i / WAVE_TABLE_LEN;
        float square_value = (sinf(phase) >= 0) ? 1.0f : -1.0f;
        float scaled_value = peak_counts * square_value;
        float dac_value = roundf((counts_per_volt * 1.65f) + scaled_value);
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 4095) dac_value = 4095;
        square_table[i] = (uint16_t)dac_value;
    }
}

void Generate_Triangle_Table(void) {
    float vref = 3.3f;
    float counts_per_volt = 4095.0f / vref;
    float peak_counts = 0.512f * counts_per_volt;
    for (int i = 0; i < WAVE_TABLE_LEN; i++) {
        float phase = 2.0f * M_PI * i / WAVE_TABLE_LEN;
        float t = phase / (2.0f * M_PI);
        float triangle_value;
        if (t < 0.25f) {
            triangle_value = 4.0f * t;
        } else if (t < 0.75f) {
            triangle_value = 2.0f - 4.0f * t;
        } else {
            triangle_value = -4.0f + 4.0f * t;
        }
        float scaled_value = peak_counts * triangle_value;
        float dac_value = roundf((counts_per_volt * 1.65f) + scaled_value);
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 4095) dac_value = 4095;
        triangle_table[i] = (uint16_t)dac_value;
    }
}

void Set_DAC_Output_Frequency(uint32_t target_freq_hz) {
    if (target_freq_hz < MIN_FREQ_HZ) target_freq_hz = MIN_FREQ_HZ;
    if (target_freq_hz > MAX_FREQ_HZ) target_freq_hz = MAX_FREQ_HZ;

    uint32_t tim6_clk = HAL_RCC_GetPCLK1Freq() * 2;
    float update_rate = (float)target_freq_hz * WAVE_TABLE_LEN;
    uint32_t prescaler = 0;
    uint32_t arr = 0;
    float counter_clk = 0.0f;
    for (prescaler = 0; prescaler < 65536; prescaler++) {
        counter_clk = (float)tim6_clk / (prescaler + 1);
        float arr_float = (counter_clk / update_rate) - 1.0f;
        arr = (uint32_t)(arr_float + 0.5f);
        if (arr >= 10 && arr <= 1000) break;
    }

    __HAL_TIM_SET_PRESCALER(&htim6, prescaler);
    if (arr > 65535) arr = 65535;
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_press_time < DEBOUNCE_TIME_MS) return;

    if (GPIO_Pin == GPIO_PIN_4) { // KEY0 (PE4) - Waveform switch
        key0_pressed = 1;
        last_press_time = current_time;
    }
    else if (GPIO_Pin == GPIO_PIN_2) { // KEY1 (PE2) - Frequency decrease
        key1_pressed = 1;
        last_press_time = current_time;
    }
    else if (GPIO_Pin == GPIO_PIN_3) { // KEY3 (PE3) - Frequency increase
        key3_pressed = 1;
        last_press_time = current_time;
    }
}

void Oscilloscope_Display_Init(void)
{
    LCD_Clear(WHITE);
    
    POINT_COLOR = BLACK; 
    BACK_COLOR = WHITE;  

    LCD_DrawRectangle(0, 0, 239, 319);
    LCD_DrawLine(0, 106, 239, 106);

    LCD_ShowString(10, 10, 230, 96, 16, "frequency: ");
    LCD_ShowString(10, 30, 230, 96, 16, "Amplitude: ");
    LCD_ShowString(10, 50, 230, 96, 16, "type: ");
    LCD_ShowString(10, 70, 230, 96, 16, "author: proury.");

    LCD_DrawLine(20, 146, 20, 310);
    LCD_DrawLine(20, 310, 230, 310);

    for (u16 x = 40; x <= 230; x += 20) {
        LCD_DrawLine(x, 146, x, 310);
    }
    for (u16 y = 146; y <= 310; y += 20) {
        LCD_DrawLine(20, y, 230, y);
    }

    LCD_DrawLine(230, 310, 225, 305);
    LCD_DrawLine(230, 310, 225, 315);
    LCD_DrawLine(20, 146, 15, 151);
    LCD_DrawLine(20, 146, 25, 151);
}

void Oscilloscope_ClearWaveform(void)
{
    LCD_Fill(21, 146, 230, 309, WHITE);
}

void Oscilloscope_DrawWaveform(u16 *wave_data)
{
    u16 i;
    POINT_COLOR = BLUE;

    for (i = 0; i < 209; i++) {
        LCD_DrawLine(20 + i, wave_data[i], 20 + i + 1, wave_data[i + 1]);
    }
    
    POINT_COLOR = BLACK;
}

void Oscilloscope_GenerateSineWave(u16 *wave_data, u16 length, float amplitude, float frequency)
{
    u16 i;
    float phase;
    u16 center_y = 228;
    
    for (i = 0; i < length; i++) {
        phase = 2.0f * M_PI * frequency * i / length;
        wave_data[i] = (u16)(center_y - amplitude * sinf(phase));
        if (wave_data[i] < SCREEN_TOP) wave_data[i] = SCREEN_TOP;
        if (wave_data[i] > SCREEN_BOTTOM) wave_data[i] = SCREEN_BOTTOM;
    }
}

void Oscilloscope_UpdateWaveform(u16 *wave_data)
{
    Oscilloscope_ClearWaveform();
    Oscilloscope_DrawWaveform(wave_data);
}

void Oscilloscope_Test(void)
{
    u16 wave_data[210];
    Oscilloscope_Display_Init();
    Oscilloscope_GenerateSineWave(wave_data, 210, 50.0f, 2.0f);
    Oscilloscope_UpdateWaveform(wave_data);
}

void Low_Pass_Filter(uint16_t *input, uint16_t *output, uint16_t length, float alpha)
{
    output[0] = input[0];
    for (uint16_t i = 1; i < length; i++) {
        output[i] = (uint16_t)((1.0f - alpha) * output[i-1] + alpha * input[i]);
    }
}

void Convert_ADC_Data_To_Waveform(uint16_t *adc_data, u16 *filtered_data, u16 *wave_data, uint16_t adc_length, uint16_t wave_length)
{
    for (uint16_t i = 0; i < wave_length; i++) {
        uint16_t index = (i * 5) % adc_length;
        int32_t adc_value = adc_data[index];
        wave_data[i] = SCREEN_CENTER - ((adc_value - 2048) * (SCREEN_BOTTOM - SCREEN_TOP) / 2 / 2048);
        if (wave_data[i] < SCREEN_TOP) wave_data[i] = SCREEN_TOP;
        if (wave_data[i] > SCREEN_BOTTOM) wave_data[i] = SCREEN_BOTTOM;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        static uint32_t dma_count = 0;
        static uint32_t last_tick = 0;
        dma_count++;
        if (dma_count >= 100) {
            uint32_t tick = HAL_GetTick();
            uint32_t delta = tick - last_tick;
            last_tick = tick;
            dma_count = 0;
        }
        data_ready = 1;
    }
}

void CalculateFFT(uint16_t *adc_data, float *fft_input, float *fft_output, 
                 arm_cfft_radix4_instance_f32 *fft_instance, uint32_t fft_length, 
                 uint32_t sampling_freq, float *freq, float *vpp, WaveType *wave)
{
    for (uint32_t i = 0; i < fft_length; i++) {
        fft_input[2*i] = (float)adc_data[i];
        fft_input[2*i+1] = 0.0f;
    }

    arm_cfft_radix4_f32(fft_instance, fft_input);
    arm_cmplx_mag_f32(fft_input, fft_output, fft_length);

    float max_value1 = 0.0f;
    uint32_t max_index1 = 0;
    for (uint32_t i = 1; i < fft_length / 2; i++) {
        if (fft_output[i] > max_value1) {
            max_value1 = fft_output[i];
            max_index1 = i;
        }
    }

    *freq = (float)max_index1 * sampling_freq / fft_length;
    *freq = round(*freq / 100.0f) * 100.0f;
    if (*freq < 100.0f) *freq = 100.0f;
    if (*freq > 100000.0f) *freq = 100000.0f;

    uint16_t max_val = adc_data[0];
    uint16_t min_val = adc_data[0];
    for (uint32_t i = 1; i < fft_length; i++) {
        if (adc_data[i] > max_val) max_val = adc_data[i];
        if (adc_data[i] < min_val) min_val = adc_data[i];
    }

    uint16_t peak_to_peak = max_val - min_val;
    *vpp = (float)peak_to_peak * 3.3f / 4095.0f;

    *wave = WAVE_SINE;
    float threshold = max_value1 * 0.03f;
    float odd_harmonic_sum = 0.0f;
    int odd_harmonic_count = 0;

    for (int n = 3; n <= 5; n += 2) {
        uint32_t harmonic_index = n * max_index1;
        if (harmonic_index >= fft_length / 2) break;
        float harmonic_amp = 0.0f;
        for (int offset = -1; offset <= 1; offset++) {
            uint32_t idx = harmonic_index + offset;
            if (idx < fft_length / 2 && fft_output[idx] > harmonic_amp) {
                harmonic_amp = fft_output[idx];
            }
        }
        if (harmonic_amp > threshold) {
            odd_harmonic_count++;
            odd_harmonic_sum += harmonic_amp;
        }
    }

    if (odd_harmonic_count >= 1) {
        if (odd_harmonic_sum / max_value1 > 0.2f) {
            *wave = WAVE_SQUARE; // Strong odd harmonics
        } else {
            *wave = WAVE_TRIANGLE; // Weaker odd harmonics
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_DAC_Init();
    MX_TIM8_Init();
    MX_FSMC_Init();
    MX_USART1_UART_Init();
    MX_TIM6_Init();

    LCD_Init();
    LCD_Clear(GREEN);

    Generate_Sin_Table();
    Generate_Square_Table();
    Generate_Triangle_Table();
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sin_table, WAVE_TABLE_LEN, DAC_ALIGN_12B_R);
    Set_DAC_Output_Frequency(current_freq_hz);
    HAL_TIM_Base_Start(&htim6);
    Oscilloscope_Test();
    HAL_Delay(2000);

    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
    arm_cfft_radix4_init_f32(&scfft, FFT_LENGTH, 0, 1);

    while (1)
    {
        if (key0_pressed) {
            uint32_t current_time = HAL_GetTick();
            if (current_time - last_press_time >= DEBOUNCE_TIME_MS) {
                HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
                current_wave = (current_wave == WAVE_SINE) ? WAVE_SQUARE :
                               (current_wave == WAVE_SQUARE) ? WAVE_TRIANGLE : WAVE_SINE;
                uint32_t *wave_table = (current_wave == WAVE_SINE) ? (uint32_t*)sin_table :
                                       (current_wave == WAVE_SQUARE) ? (uint32_t*)square_table :
                                       (uint32_t*)triangle_table;
                HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, wave_table, WAVE_TABLE_LEN, DAC_ALIGN_12B_R);
                Set_DAC_Output_Frequency(current_freq_hz);
                key0_pressed = 0;
                last_press_time = current_time;
            }
        }
        if (key1_pressed) {
            uint32_t current_time = HAL_GetTick();
            if (current_time - last_press_time >= DEBOUNCE_TIME_MS) {
                current_freq_hz -= FREQ_STEP_HZ;
                if (current_freq_hz < MIN_FREQ_HZ) {
                    current_freq_hz = MAX_FREQ_HZ;
                }
                Set_DAC_Output_Frequency(current_freq_hz);
                key1_pressed = 0;
                last_press_time = current_time;
            }
        }
        if (key3_pressed) {
            uint32_t current_time = HAL_GetTick();
            if (current_time - last_press_time >= DEBOUNCE_TIME_MS) {
                current_freq_hz += FREQ_STEP_HZ;
                if (current_freq_hz > MAX_FREQ_HZ) {
                    current_freq_hz = MIN_FREQ_HZ;
                }
                Set_DAC_Output_Frequency(current_freq_hz);
                key3_pressed = 0;
                last_press_time = current_time;
            }
        }

        CalculateFFT(ADC_1_Value_DMA, FFT_InputBuf, FFT_OutputBuf, &scfft, 
                    FFT_LENGTH, CURRENT_SAMPLING_FREQ, &main_freq, &main_vpp, &wave_type);

        LCD_Fill(100, 10, 180, 26, WHITE);
        uint32_t freq_int = (uint32_t)(main_freq * 10);
        if (freq_int / 10 > 100000) freq_int = 1000000;
        LCD_ShowNum(100, 10, freq_int / 10, 6, 16);
        LCD_ShowString(148, 10, 230, 96, 16, ".");
        LCD_ShowNum(156, 10, freq_int % 10, 1, 16);
        LCD_ShowString(164, 10, 230, 96, 16, " Hz");

        LCD_Fill(100, 30, 180, 46, WHITE);
        uint32_t mvpp = (uint32_t)(round(main_vpp * 1000.0f));
        if (mvpp > 9999) mvpp = 9999;
        LCD_ShowNum(100, 30, mvpp, 4, 16);
        LCD_ShowString(132, 30, 230, 96, 16, " mV");

        LCD_Fill(100, 50, 180, 66, WHITE);
        const char *wave_str = (wave_type == WAVE_SINE) ? "Sine" : 
                               (wave_type == WAVE_SQUARE) ? "Square" : "Triangle";
        LCD_ShowString(100, 50, 230, 96, 16, (char *)wave_str);

        if (data_ready) {
            data_ready = 0;
            Convert_ADC_Data_To_Waveform(ADC_1_Value_DMA, filtered_data, wave_data, FFT_LENGTH, 210);
            Oscilloscope_UpdateWaveform(wave_data);
            HAL_Delay(20);
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */