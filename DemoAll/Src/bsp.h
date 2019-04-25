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

#define SDRAM_BANK_ADDR 0xD0000000 //�ⲿSDRAM����ʼ��ַ
#define SDRAM_BANK_SIZE 0x007FFFFF //�ⲿSDRAM�Ĵ洢����
#define SDRAM_DATA __attribute__((section(".sdram_data")))

void BSP_SDRAM_Configuration(void) ; //��ʼ��SDRAM

void BSP_OVCAM_Configuration(void) ; //��ʼ��OV����ͷ
void BSP_OVCAM_StartSnapshot(void) ; //��ʼץOV����ͼƬ

int BSP_USBMSC_LoadJpgToRgb565(char* jpg, uint8_t* ptr, uint16_t *width, uint16_t *height) ; //��USB DISK�ļ�ϵͳ����JPEG�ļ�,��JPEG����ΪRGB565
int BSP_USBMSC_SaveJpgByRgb565(char* jpg, uint8_t* ptr, uint16_t width, uint16_t height) ; //��RGB565����ΪJPEG,��USB DISK�ļ�ϵͳ�洢JPEG�ļ�

void BSP_USBCDC_SendTxtTest(char* ptr) ; //ͨ��VCP�����ı�
void BSP_USBCDC_SendImgTest(EMB_IMAGE_PTR src) ; //ͨ��VCP����ͼƬ

#ifdef __cplusplus
}
#endif
#endif /* USR_BSP_H_ */
