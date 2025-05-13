#include "ads1292.h"
#include "main.h"  // 包含 UART_HandleTypeDef huart1 等定义

// 声明外部变量 --句柄为主
extern UART_HandleTypeDef huart1; 
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;

ADS1292_Device ads1292_dev;
/* 静态函数声明 */
static void ADS1292_CS_Enable(ADS1292_Device *dev);
static void ADS1292_CS_Disable(ADS1292_Device *dev);
static int32_t ADS1292_ParseData(uint8_t *raw);

/* 静态函数实现 */
static void ADS1292_CS_Enable(ADS1292_Device *dev) {
    HAL_GPIO_WritePin(dev->cs_gpio_port, dev->cs_gpio_pin, GPIO_PIN_RESET);
}

static void ADS1292_CS_Disable(ADS1292_Device *dev) {
    HAL_GPIO_WritePin(dev->cs_gpio_port, dev->cs_gpio_pin, GPIO_PIN_SET);
}

static int32_t ADS1292_ParseData(uint8_t *raw) {
    int32_t value = (raw[0] << 16) | (raw[1] << 8) | raw[2];
    if (value & 0x00800000) {
        value |= 0xFF000000;
    }
    return value;
}

/* 初始化函数 */
ADS1292_Status ADS1292_Init(ADS1292_Device *dev,
                            SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_gpio_port,
                            uint16_t cs_gpio_pin,
                            DMA_HandleTypeDef *hdma) {
    dev->hspi       = hspi;
    dev->cs_gpio_port = cs_gpio_port;
    dev->cs_gpio_pin = cs_gpio_pin;
    dev->hdma       = hdma;
    dev->data_ready = 0;
    dev->app_buffer_index = 0;
    dev->state      = ADS1292_OK;

    // 配置寄存器的初始化命令
    uint8_t config_cmd[] = {
        0x41, 0x01,   // 写CONFIG1（0x01）
        0x42, 0xA0,   // 写CONFIG2（0x02）
        0x46, 0x11,   // 写CONFIG3（0x03）
        0x44, 0x00,   // 写CONFIG4（0x04）
        // 其他配置...
    };

    // 发送配置命令
    ADS1292_CS_Enable(dev);
    if (HAL_SPI_Transmit(dev->hspi, config_cmd, sizeof(config_cmd), ADS1292_SPI_TIMEOUT) != HAL_OK) {
        ADS1292_CS_Disable(dev);
        return ADS1292_ERROR;
    }
    ADS1292_CS_Disable(dev);

    // 注册 DMA 回调函数
    HAL_DMA_RegisterCallback(dev->hdma, HAL_DMA_XFER_CPLT_CB_ID, DMA_Callback);

    // 将设备指针存入 DMA 的 Parent 字段
    dev->hdma->Parent = dev;

    return ADS1292_OK;
}

/* 启动转换函数 */
void ADS1292_StartConversion(ADS1292_Device *dev) {
    if (dev->state != ADS1292_OK) return;

    ADS1292_CS_Enable(dev);
    if (HAL_SPI_Receive_DMA(dev->hspi, dev->dma_buffer, ADS1292_DMA_BUFFER_SIZE) != HAL_OK) {
        ADS1292_CS_Disable(dev);
        dev->state = ADS1292_ERROR;
        return;
    }
    dev->state = ADS1292_BUSY;
}

/* DMA 回调函数 */
void DMA_Callback(DMA_HandleTypeDef *hdma) {
    ADS1292_Device *dev = (ADS1292_Device*)hdma->Parent; // 通过 DMA 的 Parent 获取设备指针

    // 解析数据并存入应用层缓冲区
    for (int i = 0; i < 3; i++) {
        int32_t value = ADS1292_ParseData(&dev->dma_buffer[i * 3]);
        dev->app_buffer[dev->app_buffer_index] = value;
        dev->app_buffer_index = (dev->app_buffer_index + 1) % ADS1292_APP_BUFFER_SIZE;
    }
    dev->data_ready = 1;

    // 重新启动 DMA 接收（自动连续采集）
    ADS1292_CS_Enable(dev);
    HAL_SPI_Receive_DMA(dev->hspi, dev->dma_buffer, ADS1292_DMA_BUFFER_SIZE);
}

/* 处理数据并发送到主机 */
void ADS1292_ProcessData(ADS1292_Device *dev) {
    if (dev->data_ready) {
        for (int i = 0; i < 3; i++) {
            int32_t value = dev->app_buffer[i];
            char data_str[20];
            sprintf(data_str, "%d\n", value);
            HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 100);
        }
        dev->data_ready = 0;
    }
}

/* 发送数据到主机的封装函数 */
void ADS1292_Send_to_Host(ADS1292_Device *dev) {
    ADS1292_ProcessData(dev);
}

void ADS1292_task(){
	
	//发送至上位机
	ADS1292_Send_to_Host(&ads1292_dev);
	
}
