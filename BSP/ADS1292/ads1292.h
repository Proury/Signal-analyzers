//#ifndef __ADS1292_H__
//#define __ADS1292_H__

//#include "stm32f4xx_hal.h"
//#include <stdint.h>

///* 定义ADS1292相关参数 */
//#define ADS1292_SPI_TIMEOUT      100
//#define ADS1292_DMA_BUFFER_SIZE  9     // 3通道×3字节
//#define ADS1292_APP_BUFFER_SIZE  256   // 应用层缓冲区大小

///* 定义状态枚举 */
//typedef enum {
//    ADS1292_OK     = 0x00,
//    ADS1292_ERROR  = 0x01,
//    ADS1292_BUSY   = 0x02,
//} ADS1292_Status;

///* 定义设备结构体 */
//typedef struct {
//    SPI_HandleTypeDef *hspi;          // SPI句柄
//    GPIO_TypeDef      *cs_gpio_port;  // CS引脚GPIO端口
//    uint16_t          cs_gpio_pin;    // CS引脚编号
//    DMA_HandleTypeDef *hdma;          // DMA句柄
//    uint8_t           dma_buffer[ADS1292_DMA_BUFFER_SIZE]; // DMA接收缓冲区
//    int32_t           app_buffer[ADS1292_APP_BUFFER_SIZE]; // 应用层数据缓冲区
//    volatile uint8_t  data_ready;     // 数据就绪标志
//    uint16_t          app_buffer_index; // 应用层缓冲区索引
//    volatile uint8_t  state;          // 设备状态（如初始化、运行）
//} ADS1292_Device;

///* 函数声明 */
//ADS1292_Status ADS1292_Init(ADS1292_Device *dev, 
//                            SPI_HandleTypeDef *hspi,
//                            GPIO_TypeDef *cs_gpio_port,
//                            uint16_t cs_gpio_pin,
//                            DMA_HandleTypeDef *hdma);

//void ADS1292_StartConversion(ADS1292_Device *dev);
//void ADS1292_ProcessData(ADS1292_Device *dev);
//void ADS1292_Send_to_Host(ADS1292_Device *dev);
//void DMA_Callback(DMA_HandleTypeDef *hdma);
//void ADS1292_task(void);

//#endif /* __ADS1292_H__ */
