#ifndef USR_BSP_H_
#define USR_BSP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"
#include "emb.h"

#define SDRAM_BANK_ADDR 0xD0000000 //外部SDRAM的起始地址
#define SDRAM_BANK_SIZE 0x007FFFFF //外部SDRAM的存储长度
#define SDRAM_DATA __attribute__((section(".sdram_data")))

void BSP_SDRAM_Configuration(void) ; //初始化SDRAM

void BSP_OVCAM_Configuration(void) ; //初始化OV摄像头
void BSP_OVCAM_StartSnapshot(void) ; //开始抓OV摄像图片

int BSP_USBMSC_LoadJpgToRgb565(char* jpg, uint8_t* ptr, uint16_t *width, uint16_t *height) ; //从USB DISK文件系统读入JPEG文件,从JPEG解码为RGB565
int BSP_USBMSC_SaveJpgByRgb565(char* jpg, uint8_t* ptr, uint16_t width, uint16_t height) ; //从RGB565编码为JPEG,向USB DISK文件系统存储JPEG文件

void BSP_USBCDC_SendTxtTest(char* ptr) ; //通过VCP发送文本
void BSP_USBCDC_SendImgTest(EMB_IMAGE_PTR src) ; //通过VCP发送图片

#ifdef __cplusplus
}
#endif
#endif /* USR_BSP_H_ */
