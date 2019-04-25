#ifndef USR_EMB_H_
#define USR_EMB_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ---------------------emb.h----------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"

typedef enum
{
  PIXEL_RGB565,  //RGB565����
  PIXEL_RGB888,  //RGB888����
  PIXEL_YUV422,  //YUV422����
  PIXEL_GRAY8B,  //GRAY8B����
  PIXEL_HSV888,  //HSV888����
} PIXEL_TYPE;   //ͼ�����ظ�ʽ
typedef enum
{
  RGB565_RGB888,   //RGB565_RGB888��ʽת��
  RGB565_GRAY8B,   //RGB565_GRAY8B��ʽת��
  RGB888_RGB565,   //RGB888_RGB565��ʽת��
  RGB888_GRAY8B,   //RGB888_GRAY8B��ʽת��
  RGB888_YUV422,   //RGB888_YUV422��ʽת��
  YUV422_RGB888,   //YUV422_RGB888��ʽת��
  RGB888_HSV888,   //RGB888_HSV888��ʽת��
  HSV888_RGB888,   //HSV888_RGB888��ʽת��
} PIXEL_CVT_TYPE; //ͼ����ɫ�ռ�任����

typedef struct //R,G,B��ɫ����
{
  uint8_t r; //R
  uint8_t g; //G
  uint8_t b; //B
} RGB_COLOR;

typedef struct //ͼ����
{
  uint16_t width;       //ͼ��Ŀ��
  uint16_t heigh;       //ͼ��ĸ߶�
  uint8_t pixelsize;    //ÿ�������ֽ���
  PIXEL_TYPE pixeltype; //���ص����ظ�ʽ
  uint8_t* data;        //ͼ�������׵�ַ
} EMB_IMAGE, *EMB_IMAGE_PTR;

typedef struct      //ͼ������㶨��
{
  uint16_t x;        //��������
  uint16_t y;        //��������
} POINT, *POINT_PTR;

typedef enum
{
  THRESH_BINARY, THRESH_BINARY_INV, THRESH_TRUNC, THRESH_TOZERO, THRESH_TOZERO_INV
} THRESH_TYPE;

/*******************************************************************************************************************************************/
//��ʼ��ͼ��
void EmbImageInit(EMB_IMAGE_PTR ptr, uint16_t width, uint16_t heigh, PIXEL_TYPE pixeltype, uint8_t* data);
//ͼ����ɫ�ռ�任
void EmbImageConvert(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst, PIXEL_CVT_TYPE type);
//�ı�ͼ��ĳߴ�
void EmbImageResize(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst);
//ͼ�񼸺�����
void EmbImageGeomath(EMB_IMAGE_PTR src,EMB_IMAGE_PTR dst,int tx,int ty,float sx,float sy,float theta,float alphax, float alphay,uint8_t flip);
//ͼ���������
void EmbImageAlgmath(EMB_IMAGE_PTR srcA, EMB_IMAGE_PTR srcB,EMB_IMAGE_PTR dst,float a,float b) ;
//ͼ���߼�����
void EmbImageBitmath(EMB_IMAGE_PTR srcA, EMB_IMAGE_PTR srcB, EMB_IMAGE_PTR dst, EMB_IMAGE_PTR mask, int8_t mode) ;
//ͼ��2ά���
void EmbImageFilter2(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst, int m, int n, float kernel[m][n]) ;
/*******************************************************************************************************************************************/
//��ͼ���ϻ��Ƶ�
void EmbImageDrawPoint(EMB_IMAGE_PTR src, POINT pt, uint8_t size, RGB_COLOR color);
//��ͼ���ϻ�����
void EmbImageDrawLine(EMB_IMAGE_PTR src, POINT pt0, POINT pt1, uint8_t lw, RGB_COLOR color);
//��ͼ���ϻ�������
void EmbImageDrawPolygon(EMB_IMAGE_PTR src, POINT_PTR pts, uint16_t n, uint8_t close, uint8_t lw, RGB_COLOR color);
//��ͼ���ϻ��ƾ���
void EmbImageDrawRectangle(EMB_IMAGE_PTR src, POINT pt, uint16_t w, uint16_t h, uint8_t lw, RGB_COLOR color, float angle);
//��ͼ���ϻ���Բ��
void EmbImageDrawCircle(EMB_IMAGE_PTR src, POINT pt, uint16_t a, uint16_t b, uint8_t lw, RGB_COLOR color, float angle);
//��ͼ���ϻ����ı�
void EmbImageDrawText(EMB_IMAGE_PTR src, POINT pt, char* str, RGB_COLOR color, float angle);
/*******************************************************************************************************************************************/
void EmbImageThreshold(EMB_IMAGE_PTR src,uint8_t val, THRESH_TYPE threshtype);

void EmbImageThresholdAdaptive(EMB_IMAGE_PTR src, uint8_t val, THRESH_TYPE threshtype);

#ifdef __cplusplus
}
#endif
#endif /* USR_EMB_H_ */
