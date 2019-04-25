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
  PIXEL_RGB565,  //RGB565像素
  PIXEL_RGB888,  //RGB888像素
  PIXEL_YUV422,  //YUV422像素
  PIXEL_GRAY8B,  //GRAY8B像素
  PIXEL_HSV888,  //HSV888像素
} PIXEL_TYPE;   //图像像素格式
typedef enum
{
  RGB565_RGB888,   //RGB565_RGB888格式转换
  RGB565_GRAY8B,   //RGB565_GRAY8B格式转换
  RGB888_RGB565,   //RGB888_RGB565格式转换
  RGB888_GRAY8B,   //RGB888_GRAY8B格式转换
  RGB888_YUV422,   //RGB888_YUV422格式转换
  YUV422_RGB888,   //YUV422_RGB888格式转换
  RGB888_HSV888,   //RGB888_HSV888格式转换
  HSV888_RGB888,   //HSV888_RGB888格式转换
} PIXEL_CVT_TYPE; //图像颜色空间变换类型

typedef struct //R,G,B颜色定义
{
  uint8_t r; //R
  uint8_t g; //G
  uint8_t b; //B
} RGB_COLOR;

typedef struct //图像定义
{
  uint16_t width;       //图像的宽度
  uint16_t heigh;       //图像的高度
  uint8_t pixelsize;    //每个像素字节数
  PIXEL_TYPE pixeltype; //像素的像素格式
  uint8_t* data;        //图像数据首地址
} EMB_IMAGE, *EMB_IMAGE_PTR;

typedef struct      //图像坐标点定义
{
  uint16_t x;        //横轴坐标
  uint16_t y;        //纵轴坐标
} POINT, *POINT_PTR;

typedef enum
{
  THRESH_BINARY, THRESH_BINARY_INV, THRESH_TRUNC, THRESH_TOZERO, THRESH_TOZERO_INV
} THRESH_TYPE;

/*******************************************************************************************************************************************/
//初始化图像
void EmbImageInit(EMB_IMAGE_PTR ptr, uint16_t width, uint16_t heigh, PIXEL_TYPE pixeltype, uint8_t* data);
//图像颜色空间变换
void EmbImageConvert(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst, PIXEL_CVT_TYPE type);
//改变图像的尺寸
void EmbImageResize(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst);
//图像几何运算
void EmbImageGeomath(EMB_IMAGE_PTR src,EMB_IMAGE_PTR dst,int tx,int ty,float sx,float sy,float theta,float alphax, float alphay,uint8_t flip);
//图像代数运算
void EmbImageAlgmath(EMB_IMAGE_PTR srcA, EMB_IMAGE_PTR srcB,EMB_IMAGE_PTR dst,float a,float b) ;
//图像逻辑运算
void EmbImageBitmath(EMB_IMAGE_PTR srcA, EMB_IMAGE_PTR srcB, EMB_IMAGE_PTR dst, EMB_IMAGE_PTR mask, int8_t mode) ;
//图像2维卷积
void EmbImageFilter2(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst, int m, int n, float kernel[m][n]) ;
/*******************************************************************************************************************************************/
//在图像上绘制点
void EmbImageDrawPoint(EMB_IMAGE_PTR src, POINT pt, uint8_t size, RGB_COLOR color);
//在图像上绘制线
void EmbImageDrawLine(EMB_IMAGE_PTR src, POINT pt0, POINT pt1, uint8_t lw, RGB_COLOR color);
//在图像上绘制折线
void EmbImageDrawPolygon(EMB_IMAGE_PTR src, POINT_PTR pts, uint16_t n, uint8_t close, uint8_t lw, RGB_COLOR color);
//在图像上绘制矩形
void EmbImageDrawRectangle(EMB_IMAGE_PTR src, POINT pt, uint16_t w, uint16_t h, uint8_t lw, RGB_COLOR color, float angle);
//在图像上绘制圆形
void EmbImageDrawCircle(EMB_IMAGE_PTR src, POINT pt, uint16_t a, uint16_t b, uint8_t lw, RGB_COLOR color, float angle);
//在图像上绘制文本
void EmbImageDrawText(EMB_IMAGE_PTR src, POINT pt, char* str, RGB_COLOR color, float angle);
/*******************************************************************************************************************************************/
void EmbImageThreshold(EMB_IMAGE_PTR src,uint8_t val, THRESH_TYPE threshtype);

void EmbImageThresholdAdaptive(EMB_IMAGE_PTR src, uint8_t val, THRESH_TYPE threshtype);

#ifdef __cplusplus
}
#endif
#endif /* USR_EMB_H_ */
