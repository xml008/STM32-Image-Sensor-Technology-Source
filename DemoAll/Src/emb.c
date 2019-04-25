#include "emb.h"

#include <arm_math.h>

#define EMBMAX3(a,b,c) (a)>(b)?((a)>(c)?(a):(c)):((b)>(c)?(b):(c))
#define EMBMIN3(a,b,c) (a)<(b)?((a)<(c)?(a):(c)):((b)<(c)?(b):(c))
#define EMBMAX2(a,b) (a)>(b)?(a):(b)
#define EMBMIN2(a,b) (a)<(b)?(a):(b)

/*******************************************************************************************************************************************/

/*初始化图像*/
/**
 * EmbImageInit:图像结构初始化
 * ptr:图像结构指针
 * width:图像宽度像素
 * heigh:图像高度像素
 * pixeltype:图像的类型
 * data:图像数据的存储区首地址
 */
void EmbImageInit(EMB_IMAGE_PTR ptr, uint16_t width, uint16_t heigh, PIXEL_TYPE pixeltype, uint8_t* data)
{
  ptr->width = width ;
  ptr->heigh = heigh ;
  ptr->pixeltype = pixeltype ;
  switch ( pixeltype )
  {
  case PIXEL_GRAY8B : //GRAY8B格式，每个像素1字节
    ptr->pixelsize = 1 ;
    break ;
  case PIXEL_RGB565 : //RGB565格式，每个像素2字节
  case PIXEL_YUV422 : //YUV422格式，每个像素2字节
    ptr->pixelsize = 2 ;
    break ;
  case PIXEL_RGB888 : //RGB888格式，每个像素3字节
  case PIXEL_HSV888 : //HSV888格式，每个像素3字节
    ptr->pixelsize = 3 ;
    break ;
  default :
    ptr->pixelsize = 0 ;
    break ;
  }
  ptr->data = data ;
}

/*******************************************************************************************************************************************/

/*像素从RGB565到RGB888的转换*/
static void PIXEL_CVT_RGB565_RGB888(uint8_t src[2], uint8_t dst[3])
{
  //|-------- src[0]-------|-------- src[1]-------|
  //R4 R3 R2 R1 R0 G5 G4 G3 G2 G1 G0 B4 B3 B2 B1 B0
  //|-------- dst[0]------ |-------- dst[1]--------|-------- dst[1]-------|
  //R7 R6 R5 R4 R3 R2 R1 R0 G7 G6 G5 G4 G3 G2 G1 G0 B7 B6 B5 B4 B3 B2 B1 B0
  dst[0] = src[0] & 0xf8 ;                          //R4 R3 R2 R1 R0  0  0  0
  dst[1] = (src[0] << 5) | ((src[1] & 0xe0) >> 3) ; //G5 G4 G3 G2 G1 G0  0  0
  dst[2] = src[1] << 3 ;                            //B4 B3 B2 B1 B0  0  0  0
}

/*像素从RGB565到GRAY8B的转换*/
static void PIXEL_CVT_RGB565_GRAY8B(uint8_t src[2], uint8_t dst[1])
{
  //|-------- src[0]-------|-------- src[1]-------|
  //R4 R3 R2 R1 R0 G5 G4 G3 G2 G1 G0 B4 B3 B2 B1 B0
  //|-------- dst[0]-------|
  //Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0
  //小数计算公式:Gray = R*0.299 + G*0.587 + B*0.114
  //整数计算公式:Gray = (R*77 + G*150 + B*29)>>8
  dst[0] = ((uint8_t) (src[0] & 0xf8) * 77 + (uint8_t) ((src[0] << 5) | ((src[1] & 0xe0) >> 3)) * 150 + (uint8_t) (src[1] << 3) * 29) >> 8 ;
}

/*像素从RGB888到RGB565的转换*/
static void PIXEL_CVT_RGB888_RGB565(uint8_t src[3], uint8_t dst[2])
{
  //|-------- src[0]------ |-------- src[1]--------|-------- src[1]-------|
  //R7 R6 R5 R4 R3 R2 R1 R0 G7 G6 G5 G4 G3 G2 G1 G0 B7 B6 B5 B4 B3 B2 B1 B0
  //|-------- dst[0]-------|-------- dst[1]-------|
  //R4 R3 R2 R1 R0 G5 G4 G3 G2 G1 G0 B4 B3 B2 B1 B0
  dst[0] = (src[0] & 0xf8) | (src[1] >> 5) ;        //R4 R3 R2 R1 R0 G5 G4 G3
  dst[1] = ((src[1] & 0xfc) << 3) | (src[2] >> 3) ; //G2 G1 G0 B4 B3 B2 B1 B0
}

/*像素从RGB888到GRAY8B的转换*/
static void PIXEL_CVT_RGB888_GRAY8B(uint8_t src[3], uint8_t dst[1])
{
  //|-------- src[0]------ |-------- src[1]--------|-------- src[1]-------|
  //R7 R6 R5 R4 R3 R2 R1 R0 G7 G6 G5 G4 G3 G2 G1 G0 B7 B6 B5 B4 B3 B2 B1 B0
  //|-------- dst[0]-------|
  //Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0
  //整数计算公式:Gray = (R*77 + G*150 + B*29)>>8
  dst[0] = (src[0] * 77 + src[1] * 150 + src[2] * 29) >> 8 ;
}

/*像素从RGB888到YUV422的转换*/
static void PIXEL_CVT_RGB888_YUV422(uint8_t src[6], uint8_t dst[4])
{
  // 小数计算
  // Y =   0.2990 R + 0.5870 G + 0.1140 B
  // U = - 0.1687 R - 0.3313 G + 0.5000 B + 128
  // V =   0.5000 R - 0.4187 G - 0.0813 B + 128
  // 整数计算
  // Y = (77*R + 150*G + 29*B)>>8;
  // U = ((-44*R  - 87*G  + 131*B)>>8) + 128
  // V = ((131*R - 110*G - 21*B)>>8) + 128
  //处理第0个像素Y0 U0 Y1 V1 Y2 U2 Y3 V3 Y4 U4 Y5 V5
  dst[0] = (77 * src[0] + 150 * src[1] + 29 * src[2]) >> 8 ;  //计算像素的Y分量
  dst[1] = ((-44 * src[0] - 87 * src[1] + 131 * src[2]) >> 8) + 128 ;  //计算像素的U分量
  //处理第1个像素Y0 U0 Y1 V1 Y2 U2 Y3 V3 Y4 U4 Y5 V5
  dst[2] = (77 * src[3] + 150 * src[4] + 29 * src[5]) >> 8 ;  //计算像素的Y分量
  dst[3] = ((131 * src[3] - 110 * src[4] - 21 * src[5]) >> 8) + 128 ;  //计算像素的V分量
}

/*像素从YUV422到RGB888的转换*/
static void PIXEL_CVT_YUV422_RGB888(uint8_t src[4], uint8_t dst[6])
{
  //小数计算
  //R = Y + 1.4075 * (V-128)
  //G = Y - 0.3455 * (U-128) - 0.7169*(V-128)
  //B = Y + 1.7790 * (U-128)
  //整数计算
  //R= Y + (360 * (V - 128))>>8 ;
  //G= Y - (179 * (U - 128)  + 86 * (V - 128))>>8 ;
  //B= Y + (455 * (U - 128))>>8 ;
  //处理第0个像素Y0 U0 Y1 V1 Y2 U2 Y3 V3 Y4 U4 Y5 V5
  dst[0] = src[0] + ((360 * (src[3] - 128)) >> 8) ;
  dst[1] = src[0] - ((179 * (src[1] - 128) + 86 * (src[3] - 128)) >> 8) ;
  dst[2] = src[0] + ((455 * (src[1] - 128)) >> 8) ;
  //处理第1个像素Y0 U0 Y1 V1 Y2 U2 Y3 V3 Y4 U4 Y5 V5
  dst[3] = src[2] + ((360 * (src[3] - 128)) >> 8) ;
  dst[4] = src[2] - ((179 * (src[1] - 128) + 86 * (src[3] - 128)) >> 8) ;
  dst[5] = src[2] + ((455 * (src[1] - 128)) >> 8) ;
}

/*像素从RGB888到HSV888的转换*/
static void PIXEL_CVT_RGB888_HSV888(uint8_t src[3], uint8_t dst[3])
{
  uint8_t hsv_max ;
  uint8_t hsv_min ;
  hsv_max = EMBMAX3(src[0], src[1], src[2]) ;
  hsv_min = EMBMIN3(src[0], src[1], src[2]) ;
  //转换计算h=[0,360],s=[0,1],v=[0,255]
  //h=0                        if max==min
  //h=60*(g-b)/(max-min)+0     if max==r and g>=b
  //h=60*(g-b)/(max-min)+360   if max==r and g<b
  //h=60*(b-r)/(max-min)+120   if max==g
  //h=60*(r-g)/(max-min)+240   if max==b
  //s=0                        if max=0
  //s=(max-min)/min            if max!=0
  //v=max
  //转换计算h=[0,180],s=[0,255],v=[0,255]
  //h=0                            if max==min
  //h=(60*(g-b)/(max-min)+0)/2     if max==r and g>=b
  //h=(60*(g-b)/(max-min)+360)/2   if max==r and g<b
  //h=(60*(b-r)/(max-min)+120)/2   if max==g
  //h=(60*(r-g)/(max-min)+240)/2   if max==b
  //s=0                            if max=0
  //s=(max-min)/min*255            if max!=0
  //v=max
  //H分量计算,映射到[0，180)
  if (hsv_max == hsv_min)
  {
    dst[0] = 0 ;
  }
  if (hsv_max == src[0] && src[1] >= src[2])
  {
    dst[0] = (uint8_t) ((60 * (src[1] - src[2]) / (hsv_max - hsv_min) + 0) / 2) ;
  }
  else if (hsv_max == src[0] && src[1] < src[2])
  {
    dst[0] = (uint8_t) ((60 * (src[1] - src[2]) / (hsv_max - hsv_min) + 360) / 2) ;
  }
  else if (hsv_max == src[1])
  {
    dst[0] = (uint8_t) ((60 * (src[2] - src[0]) / (hsv_max - hsv_min) + 120) / 2) ;
  }
  else if (hsv_max == src[2])
  {
    dst[0] = (uint8_t) ((60 * (src[0] - src[1]) / (hsv_max - hsv_min) + 240) / 2) ;
  }
  //S分量计算
  dst[1] = (hsv_max == 0) ? 0 : (uint8_t) ((1 - hsv_min / hsv_max) * 255) ;
  //V分量计算
  dst[2] = hsv_max ;
}

/*像素从HSV888到RGB888的转换*/
static void PIXEL_CVT_HSV888_RGB888(uint8_t src[3], uint8_t dst[3])
{
  static uint8_t h, f, p, q, t ;
  //转换计算h=[0,360],s=[0,1],v=[0,255]
  //h=h/60
  //f=h%60
  //p=v*(1-s)
  //q=v*(1-f*s)
  //t=v*(1-(1-f)*s)
  //(r,g,b)=(v,t,p)     if h==0
  //(r,g,b)=(q,v,p)     if h==1
  //(r,g,b)=(p,v,t)     if h==2
  //(r,g,b)=(p,q,v)     if h==3
  //(r,g,b)=(t,p,v)     if h==4
  //(r,g,b)=(v,p,q)     if h==5
  //转换计算h=[180],s=[0,255],v=[0,255]
  //h=h/30
  //f=h%30
  //p=v*(1-s/255)
  //q=v*(1-f*s/255)
  //t=v*(1-(1-f)*s/255)
  //(r,g,b)=(v,t,p)     if h==0
  //(r,g,b)=(q,v,p)     if h==1
  //(r,g,b)=(p,v,t)     if h==2
  //(r,g,b)=(p,q,v)     if h==3
  //(r,g,b)=(t,p,v)     if h==4
  //(r,g,b)=(v,p,q)     if h==5
  h = src[0] / 30 ;
  f = src[0] % 30 ;
  p = src[2] * (1.0 - src[1] / 255.0) ;
  q = src[2] * (1.0 - f * src[1] / 255.0) ;
  t = src[2] * (1.0 - (1.0 - f) * src[1] / 255.0) ;
  switch ( h )
  {
  case 0 :
  {
    dst[0] = src[2] ;  //v
    dst[1] = t ;
    dst[2] = p ;
  }
    break ;
  case 1 :
  {
    dst[0] = q ;
    dst[1] = src[2] ;  //v
    dst[2] = p ;
  }
    break ;
  case 2 :
  {
    dst[0] = p ;
    dst[1] = src[2] ;  //v
    dst[2] = t ;
  }
    break ;
  case 3 :
  {
    dst[0] = p ;
    dst[1] = q ;
    dst[2] = src[2] ;  //v
  }
    break ;
  case 4 :
  {
    dst[0] = t ;
    dst[1] = p ;
    dst[2] = src[2] ;  //v
  }
    break ;
  case 5 :
  {
    dst[0] = src[2] ;  //v
    dst[1] = p ;
    dst[2] = q ;
  }
    break ;
  default :
    break ;
  }
}

/*颜色空间转换*/
void EmbImageConvert(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst, PIXEL_CVT_TYPE type)
{
  switch ( type )
  {
  case RGB565_RGB888 :
    for (int i = 0; i < src->width * src->heigh; i++)
    {
      PIXEL_CVT_RGB565_RGB888(&src->data[i * 2], &dst->data[i * 3]) ;
    }
    break ;
  case RGB565_GRAY8B :
    for (int i = 0; i < src->width * src->heigh; i++)
    {
      PIXEL_CVT_RGB565_GRAY8B(&src->data[i * 2], &dst->data[i * 1]) ;
    }
    break ;
  case RGB888_RGB565 :
    for (int i = 0; i < src->width * src->heigh; i++)
    {
      PIXEL_CVT_RGB888_RGB565(&src->data[i * 3], &dst->data[i * 2]) ;
    }
    break ;
  case RGB888_GRAY8B :
    for (int i = 0; i < src->width * src->heigh; i++)
    {
      PIXEL_CVT_RGB888_GRAY8B(&src->data[i * 3], &dst->data[i * 1]) ;
    }
    break ;
  case RGB888_YUV422 :
    for (int i = 0; i < src->width * src->heigh; i += 2)  //一次处理2个像素
    {
      PIXEL_CVT_RGB888_YUV422(&src->data[i * 3], &dst->data[i * 2]) ;
    }
    break ;
  case YUV422_RGB888 :
    for (int i = 0; i < src->width * src->heigh; i += 2)  //一次处理2个像素
    {
      PIXEL_CVT_YUV422_RGB888(&src->data[i * 2], &dst->data[i * 3]) ;
    }
    break ;
  case RGB888_HSV888 :
    for (int i = 0; i < src->width * src->heigh; i++)
    {
      PIXEL_CVT_RGB888_HSV888(&src->data[i * 3], &dst->data[i * 3]) ;
    }
    break ;
  case HSV888_RGB888 :
    for (int i = 0; i < src->width * src->heigh; i++)
    {
      PIXEL_CVT_HSV888_RGB888(&src->data[i * 3], &dst->data[i * 3]) ;
    }
    break ;
  default :
    break ;
  }
}

//改变图像的尺寸
/**将源图像的尺寸大小更为为目标图像的大小,像素类型不变
 * src:源图像
 * dst:目标图像
 */
void EmbImageResize(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst)
{
  if ((src->pixeltype != dst->pixeltype) || src->heigh == 0 || src->width == 0 || dst->pixelsize == 0)
  {
    return ;
  }
  int width = src->width ;
  int heigh = src->heigh ;
  int pixelsize = src->pixelsize ;
  //为减小计算量，在此采用最近邻插值，根据公式:
  //src_x=src_w/dst_w*i
  //src_y=src_h/dst_h*j
  float sx = 1.0f * src->width / dst->width ;  //宽度方向的缩放因子倒数
  float sy = 1.0f * src->heigh / dst->heigh ;  //高度方向的缩放因子倒数
  int x, y ;
  for (int j = 0; j < heigh; j++)
  {
    for (int i = 0; i < width; i++)
    {
      x = (int) (sx * i) ;
      y = (int) (sy * j) ;
      memcpy((uint8_t*) (dst->data + (j * width + i) * pixelsize), (uint8_t*) (src->data + (y * width + x) * pixelsize), pixelsize) ;
    }
  }
}

/**将源图像按给定参数变换到目标图像,图像几何运算
 * src:源图像
 * dst:目标图像
 * tx:图像X方向偏移像素
 * ty:图像Y方向偏移像素
 * sx:图像X方向缩放系数
 * sy:图像Y方向缩放系数
 * theta:图像旋转角度
 * alphax:图像X方向错切角度
 * alphay:图像Y方向错切角度
 * flip:镜像参数,1:关于Y=-X镜像(对角度翻转),2:关于X镜像(上下翻转),3:关于Y镜像(左右翻转)
 */
void EmbImageGeomath(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst, int tx, int ty, float sx, float sy, float theta, float alphax, float alphay, uint8_t flip)
{
  if ((src->pixeltype != dst->pixeltype) || (src->width != dst->width) || (src->heigh != dst->heigh))
  {
    return ;
  }
  int width = src->width ;
  int heigh = src->heigh ;
  int pixelsize = src->pixelsize ;
  //根据输入参数配置变换矩阵的数据
  float Mat0[3][3] = { { 1, 0, tx }, { 0, 1, ty }, { 0, 0, 1 } } ;  //平移矩阵
  float Mat1[3][3] = { { sx, 0, 0 }, { 0, sy, 0 }, { 0, 0, 1 } } ;  //缩放矩阵
  float cosa = cosf(theta / 180.0f * PI) ;  //计算旋转角度theta正弦值
  float sina = sinf(theta / 180.0f * PI) ;  //计算旋转角度theta余弦值
  float Mat2[3][3] = { { cosa, -1.0f * sina, 0 }, { sina, cosa, 0 }, { 0, 0, 1 } } ;  //旋转矩阵
  float tanx = tanf(alphax / 180.0f * PI) ;  //计算X方向错切角度alphax的正切值
  float tany = tanf(alphay / 180.0f * PI) ;  //计算Y方向错切角度alphay的正切值
  float Mat3[3][3] = { { 1, tanx, 0 }, { tany, tanx * tany + 1, 0 }, { 0, 0, 1 } } ;  //剪切矩阵
  float Mat4[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } ;  //反射矩阵
  if (flip == 1)  //关于Y=-X镜像
  {
    Mat4[0][0] = -1, Mat4[1][1] = -1 ;
    Mat0[0][2] += width - 1, Mat0[1][2] += heigh - 1 ;  //从第3象限移到第1象限
  }
  else if (flip == 2) //关于X镜像
  {
    Mat4[0][0] = 1, Mat4[1][1] = -1 ;
    Mat0[1][2] += heigh - 1 ; //从第4象限移到第1象限
  }
  else if (flip == 3) //关于Y镜像
  {
    Mat4[0][0] = -1, Mat4[1][1] = 1 ;
    Mat0[0][2] += width - 1 ; //从第2象限移到第1象限
  }
  //初始化变换矩阵为CMSIS DSP中矩阵
  float* mat_array[5] = { (float*) Mat0, (float*) Mat1, (float*) Mat2, (float*) Mat3, (float*) Mat4 } ;
  arm_matrix_instance_f32 inst_array[5] = { 0 } ;
  for (int i = 0; i < 5; i++)
  {
    arm_mat_init_f32(&inst_array[i], 3, 3, mat_array[i]) ;
  }
  //创建两个临时矩阵，用来存储计算中间结果和最终结果
  //初始化变换矩阵为CMSIS DSP中矩阵
  float TmpMat0[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } ;
  float TmpMat1[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } ;
  float* tmpmat_array[2] = { (float*) TmpMat0, (float*) TmpMat1 } ;
  arm_matrix_instance_f32 tmpinst_array[2] = { 0 } ;
  for (int i = 0; i < 2; i++)
  {
    arm_mat_init_f32(&tmpinst_array[i], 3, 3, tmpmat_array[i]) ;
  }
  //CMSIS DSP的6个矩阵连乘,最终结果存储在tmpinst_array[1]的data中
  for (int i = 0; i < 5; i++)
  {
    arm_mat_mult_f32(&tmpinst_array[i % 2], &inst_array[i], &tmpinst_array[(i % 2 + 1) % 2]) ;
  }
  arm_mat_inverse_f32(&tmpinst_array[(4 % 2 + 1) % 2], &tmpinst_array[4 % 2]) ;
  float a11 = *(tmpinst_array[4 % 2].pData + 0 * 3 + 0) ;
  float a12 = *(tmpinst_array[4 % 2].pData + 0 * 3 + 1) ;
  float a13 = *(tmpinst_array[4 % 2].pData + 0 * 3 + 2) ;
  float a21 = *(tmpinst_array[4 % 2].pData + 1 * 3 + 0) ;
  float a22 = *(tmpinst_array[4 % 2].pData + 1 * 3 + 1) ;
  float a23 = *(tmpinst_array[4 % 2].pData + 1 * 3 + 2) ;
  //计算坐标变换
  int x, y ;
  for (int j = 0; j < heigh; j++)
  {
    for (int i = 0; i < width; i++)
    {
      x = (int) (i * a11 + j * a12 + 1 * a13) ;
      y = (int) (i * a21 + j * a22 + 1 * a23) ;
      if ((0 <= x && x < width) && (0 <= y && y < heigh))  //边界判断
      {
        memcpy((uint8_t*) (dst->data + (j * width + i) * pixelsize), (uint8_t*) (src->data + (y * width + x) * pixelsize), pixelsize) ;
      }
    }
  }
}

/**将源图像按给定参数变换到目标图像,图像代数运算DST=a*A+b*B,仅支持8位灰度图像
 * srcA:源图像
 * srcB:源图像
 * dst:目标图像
 * k:参数
 */
void EmbImageAlgmath(EMB_IMAGE_PTR srcA, EMB_IMAGE_PTR srcB, EMB_IMAGE_PTR dst,float a,float b)
{
  if (srcA->pixeltype != PIXEL_GRAY8B || srcB->pixeltype != PIXEL_GRAY8B || dst->pixeltype != PIXEL_GRAY8B || srcA->width != srcB->width || srcA->width != dst->width || srcA->heigh != srcB->heigh || srcA->heigh != dst->heigh)
  {
    return ;
  }
  for (int j = 0; j < heigh; j++)//ROW
  {
    for (int i = 0; i < width; i++)//COL
    {
      dst[j][i] = a*srcA[j][i] + b*srcB[j][i] ;
    }
  }
}

/**将源图像按给定参数变换到目标图像,图像逻辑运算,仅支持8位灰度图像
 * srcA:源图像
 * srcB:源图像
 * dst:目标图像
 * mask:运算掩码(区域),只能是灰度图像
 * mode:镜像参数,1:and与,2:or或,3:not非,3:nor异或
 */
void EmbImageBitmath(EMB_IMAGE_PTR srcA, EMB_IMAGE_PTR srcB, EMB_IMAGE_PTR dst, EMB_IMAGE_PTR mask, int8_t mode)
{
  if (srcA->pixeltype != PIXEL_GRAY8B || srcB->pixeltype != PIXEL_GRAY8B || dst->pixeltype != PIXEL_GRAY8B ||
      srcA->width != srcB->width || srcA->width != dst->width ||srcA->width != mask->width||
      srcA->heigh != srcB->heigh || srcA->heigh != dst->heigh ||srcA->heigh != mask->heigh)
    {
      return ;
    }
}

//图像2维卷积,仅支持8位灰度图像
void EmbImageFilter2(EMB_IMAGE_PTR src, EMB_IMAGE_PTR dst, int m, int n, float kernel[m][n])
{
  if ((src->width != dst->width) || (src->heigh != dst->heigh) || (src->pixeltype != dst->pixeltype))
  {
    return ;
  }
  int width = src->width ;
  int heigh = src->heigh ;
  int x, y ;
  float tmp[3] = { 0.0f } ;
  int xmin = -n / 2 ;
  int xmax = n / 2 + n % 2 ? 0 : 1 ;  //偶数则+1
  int ymin = -m / 2 ;
  int ymax = m / 2 + m % 2 ? 0 : 1 ;  //偶数则+1
  for (int j = 0; j < heigh; j++)  //图像的行遍历
  {
    for (int i = 0; i < width; i++)  //图像的列遍历，(i,j)是图像卷积的中心
    {
      tmp[0] = 0.0f, tmp[1] = 0.0f, tmp[2] = 0.0f ;
      for (int r = ymin; r < ymax; r++)
      {
        for (int c = xmin; c < xmax; c++)
        {
          if (j + r < 0)  //垂直方向越界
          {
            y = 0 ;
          }
          else if (j + r >= heigh)  //垂直方向越界
          {
            y = heigh - 1 ;
          }
          else//在范围内部
          {
            y = j + r ;
          }
          if (i + c < 0)  //水平方向越界
          {
            x = 0 ;
          }
          else if (i + c >= width)  //水平方向越界
          {
            x = width - 1 ;
          }
          else//在范围内部
          {
            x = i + c ;
          }
          if (src->pixeltype==PIXEL_GRAY8B)
          {
            tmp[0] +=*(src->data+(y*width+x)*1)*kernel[c-ymin][r-xmin];
          }
        }
      }
      if (src->pixeltype == PIXEL_GRAY8B)
      {
        *(dst->data+(j*width+i)*1)=tmp[0];
      }
    }
  }
}
/*******************************************************************************************************************************************/

/*在图像SRC中的PT处以RGB颜色绘制点*/
static void IMAGE_DRAW(EMB_IMAGE_PTR src, POINT pt, RGB_COLOR color)
{
  if (pt.x >= src->width || pt.y >= src->heigh)
  {
    return ;
  }
  static uint8_t rgb888[3] ;
  rgb888[0] = color.r ;
  rgb888[1] = color.g ;
  rgb888[2] = color.b ;
  if (src->pixeltype == PIXEL_RGB565)
  {
    static uint8_t rgb565[2] ;
    PIXEL_CVT_RGB888_RGB565(rgb888, rgb565) ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 0] = rgb565[0] ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 1] = rgb565[1] ;
  }
  else if (src->pixeltype == PIXEL_RGB888)
  {
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 0] = rgb888[0] ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 1] = rgb888[1] ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 2] = rgb888[2] ;
  }
  else if (src->pixeltype == PIXEL_GRAY8B)
  {
    static uint8_t gray8b[1] ;
    PIXEL_CVT_RGB888_GRAY8B(rgb888, gray8b) ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 0] = gray8b[0] ;
  }
  else if (src->pixeltype == PIXEL_HSV888)
  {
    static uint8_t hsv888[3] ;
    PIXEL_CVT_RGB888_HSV888(rgb888, hsv888) ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 0] = hsv888[0] ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 1] = hsv888[0] ;
    src->data[(pt.y * src->width + pt.x) * src->pixelsize + 2] = hsv888[0] ;
  }
}

/*在图像SRC中的PT处以RGB颜色绘制大小为SIZE的点*/
void EmbImageDrawPoint(EMB_IMAGE_PTR src, POINT pt, uint8_t size, RGB_COLOR color)
{
  POINT pt0 ;
  for (pt0.x = pt.x - size / 2 ; pt0.x <= pt.x + size / 2; pt0.x++)
  {
    for (pt0.y = pt.y - size / 2 ; pt0.y <= pt.y + size / 2; pt0.y++)
    {
      IMAGE_DRAW(src, pt0, color) ;
    }
  }
}

/*在图像SRC中的PT0到PT1间以RGB颜色绘制宽度为LW的直线*/
void EmbImageDrawLine(EMB_IMAGE_PTR src, POINT pt0, POINT pt1, uint8_t lw, RGB_COLOR color)
{
//使用参数方程进行直线绘制
//x=x_0+t*cos(a)
//y=y_0+t*sin(a)
//t:从(x_0,y_0)出发的线段的长度
//a:与X轴正方向的夹角,-pi~+pi
  float a ;
  float t ;
  float sin_a, cos_a ;
  a = atan2f(pt1.y - pt0.y, pt1.x - pt0.x) ;
  t = sqrtf((pt1.y - pt0.y) * (pt1.y - pt0.y) + (pt1.x - pt0.x) * (pt1.x - pt0.x)) ;
  sin_a = arm_sin_f32(a) ;
  cos_a = arm_cos_f32(a) ;
  for (float t0 = 1; t0 < t; t0++)
  {
    pt1.x = pt0.x + rintf(t0 * cos_a) ;
    pt1.y = pt0.y + rintf(t0 * sin_a) ;
    EmbImageDrawPoint(src, pt1, lw, color) ;
  }
}

/*在图像SRC中的以PTS为定点以RGB颜色绘制宽度为LW的多边形折现，若CLOSE非0则绘制为闭合多边形，若FILL非0则绘制填充*/
void EmbImageDrawPolygon(EMB_IMAGE_PTR src, POINT_PTR pts, uint16_t n, uint8_t close, uint8_t lw, RGB_COLOR color)
{
  for (int i = 1; i < n; i++)
  {
    EmbImageDrawLine(src, pts[i - 1], pts[i], lw, color) ;
  }
  if (close != 0 && n >= 3)
  {
    EmbImageDrawLine(src, pts[n - 1], pts[0], lw, color) ;
  }
}

void EmbImageDrawRectangle(EMB_IMAGE_PTR src, POINT pt, uint16_t w, uint16_t h, uint8_t lw, RGB_COLOR color, float angle)
{
//坐标平移+旋转
//x=i·cos(a)-j·sin(a)+x0*/
//y=i·sin(a)+j·cos(a)+y0*/
  POINT pts[4] = { pt } ;  //顺时针方向存放顶点
  float cos_a = arm_cos_f32(angle / 180.0 * PI) ;
  float sin_a = arm_sin_f32(angle / 180.0 * PI) ;
  pts[1].x = w * cos_a - w * sin_a + pt.x ;
  pts[1].y = 0 * cos_a + 0 * sin_a + pt.y ;
  pts[2].x = w * cos_a - w * sin_a + pt.x ;
  pts[2].y = h * cos_a + h * sin_a + pt.y ;
  pts[3].x = 0 * cos_a - 0 * sin_a + pt.x ;
  pts[3].y = h * cos_a + h * sin_a + pt.y ;
  EmbImageDrawPolygon(src, pts, 4, 1, lw, color) ;
}

void EmbImageDrawCircle(EMB_IMAGE_PTR src, POINT pt, uint16_t a, uint16_t b, uint8_t lw, RGB_COLOR color, float angle)
{
//坐标平移+旋转
//x=i·cos(a)-j·sin(a)+x0
//y=i·sin(a)+i·cos(a)+y0
//椭圆方程:i^2/a^2+j^2/b^2=1
//上半弦:j=+b/a·sqrt(a^2-i^2)
//下半弦:j=-b/a·sqrt(a^2-i^2)
  float cos_a = arm_cos_f32(angle / 180.0 * PI) ;
  float sin_a = arm_sin_f32(angle / 180.0 * PI) ;
  POINT pt1 ;
  float y ;
  for (int x = -a; x <= a; x++)
  {
    y = b / a * sqrtf(a * a - x * x) ;
    pt1.x = x * cos_a - x * sin_a + pt.x ;
    pt1.y = y * cos_a + y * sin_a + pt.y ;
    EmbImageDrawPoint(src, pt1, lw, color) ;

    y = -b / a * sqrtf(a * a - x * x) ;
    pt1.x = x * cos_a - x * sin_a + pt.x ;
    pt1.y = y * cos_a + y * sin_a + pt.y ;
    EmbImageDrawPoint(src, pt1, lw, color) ;
  }
}

const uint8_t Font16X8[95][16] = {
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },/*" ",0*/

{ 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00 },/*"!",1*/

{ 0x36, 0x24, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },/*""",2*/

{ 0x00, 0x24, 0x24, 0x24, 0x24, 0xFE, 0x48, 0x48, 0x48, 0x48, 0xFC, 0x48, 0x48, 0x48, 0x48, 0x00 },/*"#",3*/

{ 0x10, 0x38, 0x54, 0x92, 0x92, 0x50, 0x30, 0x18, 0x14, 0x12, 0x92, 0x92, 0x54, 0x38, 0x10, 0x00 },/*"$",4*/

{ 0x00, 0x62, 0x92, 0x94, 0x94, 0x68, 0x08, 0x10, 0x20, 0x2C, 0x52, 0x52, 0x92, 0x8C, 0x00, 0x00 },/*"%",5*/

{ 0x00, 0x30, 0x48, 0x48, 0x48, 0x48, 0x30, 0x20, 0x54, 0x94, 0x88, 0x88, 0x94, 0x62, 0x00, 0x00 },/*"&",6*/

{ 0x30, 0x30, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },/*"'",7*/

{ 0x04, 0x08, 0x10, 0x10, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x08, 0x04, 0x00 },/*"(",8*/

{ 0x40, 0x20, 0x10, 0x10, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x10, 0x10, 0x20, 0x40, 0x00 },/*")",9*/

{ 0x00, 0x00, 0x00, 0x10, 0x92, 0x54, 0x38, 0x10, 0x38, 0x54, 0x92, 0x10, 0x00, 0x00, 0x00, 0x00 },/*"*",10*/

{ 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0xFE, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 },/*"+",11*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x10, 0x20, 0x00 },/*",",12*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },/*"-",13*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00 },/*".",14*/

{ 0x00, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x10, 0x20, 0x20, 0x40, 0x40, 0x80, 0x80, 0x00, 0x00 },/*"/",15*/

{ 0x00, 0x30, 0x48, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x48, 0x30, 0x00, 0x00 },/*"0",16*/

{ 0x00, 0x10, 0x70, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00 },/*"1",17*/

{ 0x00, 0x30, 0x48, 0x84, 0x84, 0x04, 0x08, 0x08, 0x10, 0x20, 0x20, 0x40, 0x80, 0xFC, 0x00, 0x00 },/*"2",18*/

{ 0x00, 0x30, 0x48, 0x84, 0x84, 0x04, 0x08, 0x30, 0x08, 0x04, 0x84, 0x84, 0x48, 0x30, 0x00, 0x00 },/*"3",19*/

{ 0x00, 0x08, 0x08, 0x18, 0x18, 0x28, 0x28, 0x48, 0x48, 0x88, 0xFC, 0x08, 0x08, 0x08, 0x00, 0x00 },/*"4",20*/

{ 0x00, 0xFC, 0x80, 0x80, 0x80, 0xB0, 0xC8, 0x84, 0x04, 0x04, 0x04, 0x84, 0x48, 0x30, 0x00, 0x00 },/*"5",21*/

{ 0x00, 0x30, 0x48, 0x84, 0x84, 0x80, 0xB0, 0xC8, 0x84, 0x84, 0x84, 0x84, 0x48, 0x30, 0x00, 0x00 },/*"6",22*/

{ 0x00, 0xFC, 0x04, 0x04, 0x08, 0x08, 0x08, 0x10, 0x10, 0x10, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00 },/*"7",23*/

{ 0x00, 0x30, 0x48, 0x84, 0x84, 0x84, 0x48, 0x30, 0x48, 0x84, 0x84, 0x84, 0x48, 0x30, 0x00, 0x00 },/*"8",24*/

{ 0x00, 0x30, 0x48, 0x84, 0x84, 0x84, 0x84, 0x4C, 0x34, 0x04, 0x84, 0x84, 0x48, 0x30, 0x00, 0x00 },/*"9",25*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00 },/*":",26*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x10, 0x20, 0x00 },/*";",27*/

{ 0x00, 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x00, 0x00, 0x00 },/*"<",28*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },/*"=",29*/

{ 0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00 },/*">",30*/

{ 0x00, 0x30, 0x48, 0x84, 0x84, 0x04, 0x08, 0x10, 0x20, 0x20, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00 },/*"?",31*/

{ 0x00, 0x38, 0x44, 0x82, 0x9A, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x9C, 0x80, 0x42, 0x3C, 0x00, 0x00 },/*"@",32*/

{ 0x00, 0x10, 0x10, 0x28, 0x28, 0x28, 0x28, 0x44, 0x44, 0x44, 0x7C, 0x82, 0x82, 0x82, 0x00, 0x00 },/*"A",33*/

{ 0x00, 0xF8, 0x84, 0x82, 0x82, 0x82, 0x84, 0xF8, 0x84, 0x82, 0x82, 0x82, 0x84, 0xF8, 0x00, 0x00 },/*"B",34*/

{ 0x00, 0x38, 0x44, 0x82, 0x82, 0x80, 0x80, 0x80, 0x80, 0x80, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00 },/*"C",35*/

{ 0x00, 0xF8, 0x84, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x84, 0xF8, 0x00, 0x00 },/*"D",36*/

{ 0x00, 0xFE, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFC, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFE, 0x00, 0x00 },/*"E",37*/

{ 0x00, 0xFE, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFC, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 },/*"F",38*/

{ 0x00, 0x38, 0x44, 0x82, 0x82, 0x80, 0x80, 0x80, 0x8E, 0x82, 0x82, 0x82, 0x46, 0x3A, 0x00, 0x00 },/*"G",39*/

{ 0x00, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0xFE, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x00, 0x00 },/*"H",40*/

{ 0x00, 0x38, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00 },/*"I",41*/

{ 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x84, 0x84, 0x48, 0x30, 0x00, 0x00 },/*"J",42*/

{ 0x00, 0x82, 0x84, 0x84, 0x88, 0x90, 0x90, 0xA0, 0xD0, 0x88, 0x88, 0x84, 0x82, 0x82, 0x00, 0x00 },/*"K",43*/

{ 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFE, 0x00, 0x00 },/*"L",44*/

{ 0x00, 0x82, 0x82, 0xC6, 0xC6, 0xC6, 0xC6, 0xAA, 0xAA, 0xAA, 0xAA, 0x92, 0x92, 0x92, 0x00, 0x00 },/*"M",45*/

{ 0x00, 0x82, 0x82, 0xC2, 0xC2, 0xA2, 0xA2, 0x92, 0x92, 0x8A, 0x8A, 0x86, 0x86, 0x82, 0x00, 0x00 },/*"N",46*/

{ 0x00, 0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00 },/*"O",47*/

{ 0x00, 0xF8, 0x84, 0x82, 0x82, 0x82, 0x84, 0xF8, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 },/*"P",48*/

{ 0x00, 0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x92, 0x8A, 0x44, 0x3A, 0x00, 0x00 },/*"Q",49*/

{ 0x00, 0xF8, 0x84, 0x82, 0x82, 0x82, 0x84, 0xF8, 0x88, 0x88, 0x84, 0x84, 0x82, 0x82, 0x00, 0x00 },/*"R",50*/

{ 0x00, 0x38, 0x44, 0x82, 0x82, 0x80, 0x60, 0x18, 0x04, 0x02, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00 },/*"S",51*/

{ 0x00, 0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00 },/*"T",52*/

{ 0x00, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00 },/*"U",53*/

{ 0x00, 0x82, 0x82, 0x82, 0x44, 0x44, 0x44, 0x44, 0x28, 0x28, 0x28, 0x10, 0x10, 0x10, 0x00, 0x00 },/*"V",54*/

{ 0x00, 0x92, 0x92, 0x92, 0x92, 0xAA, 0xAA, 0xAA, 0xAA, 0x44, 0x44, 0x44, 0x44, 0x44, 0x00, 0x00 },/*"W",55*/

{ 0x00, 0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x10, 0x28, 0x28, 0x44, 0x44, 0x82, 0x82, 0x00, 0x00 },/*"X",56*/

{ 0x00, 0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00 },/*"Y",57*/

{ 0x00, 0xFE, 0x02, 0x04, 0x04, 0x08, 0x08, 0x10, 0x20, 0x20, 0x40, 0x40, 0x80, 0xFE, 0x00, 0x00 },/*"Z",58*/

{ 0x7C, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7C, 0x00 },/*"[",59*/

{ 0x00, 0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x7C, 0x10, 0x10, 0x7C, 0x10, 0x10, 0x10, 0x00, 0x00 },/*"\",60*/

{ 0x7C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x7C, 0x00 },/*"]",61*/

{ 0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },/*"^",62*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF },/*"_",63*/

{ 0x30, 0x30, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },/*"'",64*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x84, 0x04, 0x3C, 0x44, 0x84, 0x8C, 0x76, 0x00, 0x00 },/*"a",65*/

{ 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0xB8, 0xC4, 0x82, 0x82, 0x82, 0x82, 0xC4, 0xB8, 0x00, 0x00 },/*"b",66*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x42, 0x80, 0x80, 0x80, 0x80, 0x42, 0x3C, 0x00, 0x00 },/*"c",67*/

{ 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x3A, 0x46, 0x82, 0x82, 0x82, 0x82, 0x46, 0x3A, 0x00, 0x00 },/*"d",68*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x82, 0xFE, 0x80, 0x80, 0x42, 0x3C, 0x00, 0x00 },/*"e",69*/

{ 0x00, 0x18, 0x20, 0x20, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00 },/*"f",70*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x44, 0x44, 0x38, 0x40, 0x7C, 0x82, 0x82, 0x7C, 0x00 },/*"g",71*/

{ 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0xB8, 0xC4, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x00, 0x00 },/*"h",72*/

{ 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00 },/*"i",73*/

{ 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x60, 0x00 },/*"j",74*/

{ 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x84, 0x88, 0x90, 0xA0, 0xD0, 0x88, 0x84, 0x82, 0x00, 0x00 },/*"k",75*/

{ 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00 },/*"l",76*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAC, 0xD2, 0x92, 0x92, 0x92, 0x92, 0x92, 0x92, 0x00, 0x00 },/*"m",77*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0xC4, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x00, 0x00 },/*"n",78*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00 },/*"o",79*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0xC4, 0x82, 0x82, 0x82, 0xC4, 0xB8, 0x80, 0x80, 0x00 },/*"p",80*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x46, 0x82, 0x82, 0x82, 0x46, 0x3A, 0x02, 0x02, 0x00 },/*"q",81*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2E, 0x30, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00 },/*"r",82*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x82, 0x80, 0x60, 0x1C, 0x02, 0x82, 0x7C, 0x00, 0x00 },/*"s",83*/

{ 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x18, 0x00, 0x00 },/*"t",84*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x46, 0x3A, 0x00, 0x00 },/*"u",85*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x10, 0x10, 0x00, 0x00 },/*"v",86*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92, 0x92, 0x92, 0xAA, 0xAA, 0x44, 0x44, 0x44, 0x00, 0x00 },/*"w",87*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x44, 0x28, 0x10, 0x10, 0x28, 0x44, 0x82, 0x00, 0x00 },/*"x",88*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x10, 0x20, 0xC0, 0x00 },/*"y",89*/

{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFE, 0x00, 0x00 },/*"z",90*/

{ 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x20, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C, 0x00 },/*"{",91*/

{ 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 },/*"|",92*/

{ 0x70, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x70, 0x00 },/*"}",93*/

{ 0x64, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, /*"~",94*/
} ;
//在图像上绘制文本
void EmbImageDrawText(EMB_IMAGE_PTR src, POINT pt, char* str, RGB_COLOR color, float angle)
{

}
/*******************************************************************************************************************************************/
void EmbImageThreshold(EMB_IMAGE_PTR src, uint8_t val, THRESH_TYPE threshtype)
{
  if (src->pixeltype != PIXEL_GRAY8B)
  {
    return ;
  }
  for (uint32_t i = 0; i < src->width * src->heigh; i++)
  {
    if (threshtype == THRESH_BINARY)
    {
      src->data[i] = src->data[i] >= val ? 255 : 0 ;
    }
    else if (threshtype == THRESH_BINARY_INV)
    {
      src->data[i] = src->data[i] <= val ? 255 : 0 ;
    }
    else if (threshtype == THRESH_TRUNC)
    {
      if (src->data[i] >= val)
      {
        src->data[i] = val ;
      }
    }
    else if (threshtype == THRESH_TOZERO)
    {
      if (src->data[i] < val)
      {
        src->data[i] = 0 ;
      }
    }
    else if (threshtype == THRESH_TOZERO_INV)
    {
      if (src->data[i] >= val)
      {
        src->data[i] = 0 ;
      }
    }
  }
}

void EmbImageThresholdAdaptive(EMB_IMAGE_PTR src, uint8_t val, THRESH_TYPE threshtype)
{
  if (src->pixeltype != PIXEL_GRAY8B)
  {
    return ;
  }
  uint32_t arr0[64] = { 0 } ;  //64个等级的直方图存储,灰度0-3为0级,灰度4-7为1级,...,灰度252-255为63级
  uint32_t arr1[64] = { 0 } ;  //平滑后的数据
  for (uint32_t i = 0; i < src->width * src->heigh; i++)
  {
    arr0[*(src->data + i) / 4] += 1 ;
  }
  for (uint16_t i = 0, r = 5; i < 64; i++)
  {
    for (int16_t j = r / 2; j < r / 2 + r % 2; j++)
    {
      if (i + j < 0) //数据在0元素之前
      {
        arr1[i] += arr0[-i - j] ;
      }
      else if (i + j > 63) //数据在63元素之后
      {
        arr1[i] += arr0[63 - (j + i - 63)] ;
      }
      else //数据在0~63元素之间
      {
        arr1[i] += arr0[i + j] ;
      }
    }
    arr1[i] /= r ;
  }
}
/*******************************************************************************************************************************************/
