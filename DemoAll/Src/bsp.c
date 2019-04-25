#include "bsp.h"

/*sdram��Ҫ��ͷ�ļ�*/
#include "fmc.h"
#include <stdlib.h>

/*sdram��Ҫ��ͷ�ļ�*/
#include "dcmi.h"
#include "i2c.h"

/*USB��Ҫ��ͷ�ļ�*/
#include "usb_host.h"
#include "fatfs.h"
#include "jpeg.h"
#include "JPEG/jpeg_utils.h"
#include "usbd_cdc_if.h"
#include <string.h>

/*****************************************************************************************************/
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

void BSP_SDRAM_Configuration(void)
{
  FMC_SDRAM_CommandTypeDef Command;

  /* Step 1: ����ʱ��ʹ������ */
  Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF); /* �������� */

  /* Step 2: �ȴ�ָ���ӳ�1ms*/
  HAL_Delay(1);

  /* Step 3: ����Ԥ���洢�������� */
  Command.CommandMode = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF); /* �������� */

  /* Step 4: ������ˢ������,8����ˢ������ */
  Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber = 8;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF); /* �������� */

  /* Step 5: ���� SDRAMģʽ�Ĵ���������:
   * ͻ������:1 ;
   * ͻ�����䷽ʽ:˳��;
   * CASǱ����:2;
   * ����ģʽ:��׼;
   * ����ģʽ:ͻ����/��һд */
  uint32_t tmpmrd = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1 |\
 SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |\
 SDRAM_MODEREG_CAS_LATENCY_2 |\
 SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
 SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED;
  Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.ModeRegisterDefinition = tmpmrd;
  Command.AutoRefreshNumber = 1;
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF); /* �������� */

  /* Step 6: ����ˢ���� */
  HAL_SDRAM_ProgramRefreshRate(&hsdram1, 1675);

}
/*****************************************************************************************************/
//��ʼ���Ĵ������м����Ӧ��ֵ
const struct
{
  uint8_t reg;
  uint8_t val;
} OVCAM_REG[] = {
    //Frame Rate Adjustment for 8Mhz input clock
    //30 fps, PCLK = 24Mhz
    { 0x0d, 0x81 }, //�ⲿʱ��6��Ƶ
    { 0x11, 0x00 }, //�ڲ�ʱ��2��Ƶ
    { 0x2a, 0x00 }, //
    { 0x2b, 0x00 }, //
    { 0x33, 0x00 }, //
    { 0x34, 0x00 }, //
    { 0x2d, 0x00 }, //
    { 0x2e, 0x00 }, //
    { 0x0e, 0x65 }, //
    //Banding Filter Setting for 24Mhz Input Clock
    //30fps for 60Hz light frequency
    { 0x13, 0xff }, //banding filter enable
    { 0x22, 0x7f }, //60Hz banding filter
    { 0x23, 0x03 }, //4 step for 60hz
    //����ģʽ
    //Auto
    { 0x13, 0xff }, //AWB on
    { 0x0e, 0x65 }, //
    { 0x2d, 0x00 }, //
    { 0x2e, 0x00 }, //
    //���Ͷ�
    //Saturation + 0
    { 0xa7, 0x50 }, //
    { 0xa8, 0x50 }, //
    //������
    //Brightness +1
    { 0x9b, 0x18 }, //
    { 0xab, 0x06 }, //
    //�Աȶ�
    //Contrast +1
    { 0x9c, 0x24 }, //
    //��Чģʽ
    //Normal
    { 0xa6, 0x06 }, //
    { 0x60, 0x80 }, //
    { 0x61, 0x80 }, //
    //DSP���Ʋ���
    { 0x42, 0x7f }, //
    { 0x4d, 0x09 }, //
    { 0x63, 0xe0 }, //
    { 0x64, 0xff }, //
    { 0x65, 0x20 }, //
    { 0x66, 0x00 }, //
    { 0x67, 0x05 }, //
    //ɫ�ʻ�ԭ��������
    { 0x94, 0x2c }, //
    { 0x95, 0x24 }, //
    { 0x96, 0x08 }, //
    { 0x97, 0x14 }, //
    { 0x98, 0x24 }, //
    { 0x99, 0x38 }, //
    //gama��������
    { 0x7e, 0x0c }, //
    { 0x7f, 0x16 }, //
    { 0x80, 0x2a }, //
    { 0x81, 0x4e }, //
    { 0x82, 0x61 }, //
    { 0x83, 0x6f }, //
    { 0x84, 0x7b }, //
    { 0x85, 0x86 }, //
    { 0x86, 0x8e }, //
    { 0x87, 0x97 }, //
    { 0x88, 0xa4 }, //
    { 0x89, 0xaf }, //
    { 0x8a, 0xc5 }, //
    { 0x8b, 0xd7 }, //
    { 0x8c, 0xe8 }, //
    { 0x8d, 0x20 }, //

    { 0x12, 0x06 }, //VGA RGB565
    { 0x17, 0x23 }, //HSTART,VGA:{ 0x17, 0x23 },QVGA:{ 0x17, 0x3f }
    { 0x18, 0xa0 }, //HSIZE,VGA:{ 0x18, 0xA0 },QVGA:{ 0x18, 0x50 }
    { 0x19, 0x07 }, //VSTART,VGA:{ 0x19, 0x07 },QVGA:{ 0x19, 0x03 }
    { 0x1a, 0xf0 }, //VSIZE, VGA:{ 0x1a, 0xF0 },QVGA:{ 0x1a, 0x78 }
    { 0x32, 0x00 }, //HREF

    { 0x0c, 0x90 }, //ͼ���������ҷ�ת�����MSB/LSB�������
    { 0x0e, 0x65 }, //���óɹ̶�֡��
    };
#define W  640              //ͼ����
#define H  480              //ͼ��߶�
#define PIXEL_SIZE  (W*H)   //���ظ���

SDRAM_DATA uint8_t ImgRGB565Src[PIXEL_SIZE * 2];

//ʹ��I2Cģ��SCCB��������OV7670�Ĵ���д������
static void OVCAM_SCCB_WR_REG(uint8_t reg, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c1, 0x42, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 0xff);
}
//ʹ��I2Cģ��SCCB��������OV7670�Ĵ�����������
static void OVCAM_SCCB_RD_REG(uint8_t reg, uint8_t* val)
{
  HAL_I2C_Master_Transmit(&hi2c1, 0x42, &reg, 1, 0xff);
  HAL_I2C_Master_Receive(&hi2c1, 0x42, val, 1, 0xff);
}

void BSP_OVCAM_Configuration(void)
{
  HAL_GPIO_WritePin(DCMI_RESET_GPIO_Port, DCMI_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);
  HAL_Delay(5);
  OVCAM_SCCB_WR_REG(0x12, 0x80);
  HAL_Delay(50);
  uint8_t val;
  for (int i = 0; i < sizeof(OVCAM_REG) / sizeof(OVCAM_REG[0]); i++)
  {
    OVCAM_SCCB_WR_REG(OVCAM_REG[i].reg, OVCAM_REG[i].val);
    HAL_Delay(1);
    OVCAM_SCCB_RD_REG(OVCAM_REG[i].reg, &val);
    HAL_Delay(1);
  }
}

void BSP_OVCAM_StartSnapshot(void)
{
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) ImgRGB565Src, PIXEL_SIZE / 2);
}

#define OVCAM_RGB565

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  static EMB_IMAGE src;
  EmbImageInit(&src, W, H, PIXEL_RGB565, (uint8_t*) ImgRGB565Src);
#if defined	    OVCAM_RGB565//ԭʼRGB565
  //����ͼ��
  static EMB_IMAGE dst1;
  static SDRAM_DATA uint8_t ImgRGB2ByteDst1[W * H * 2 / 4];
  memset((uint8_t*) ImgRGB2ByteDst1,0,W * H * 2 / 4);
  EmbImageInit(&dst1, W / 2, H / 2, PIXEL_RGB565, (uint8_t*) ImgRGB2ByteDst1);
  EmbImageResize(&src, &dst1);
  //���α任ͼ��
  static EMB_IMAGE dst2 ;
  static  SDRAM_DATA uint8_t ImgRGB2ByteDst2[W * H * 2 / 4] ;
  memset((uint8_t*) ImgRGB2ByteDst2,0,W * H * 2 / 4);
  EmbImageInit(&dst2, W / 2, H / 2, PIXEL_RGB565, (uint8_t*) ImgRGB2ByteDst2) ;
  EmbImageGeomath(&dst1,&dst2,0,0,1,1,0,0,0,2);
  //ͨ�����⴮�ڷ��͵�PC
  BSP_USBCDC_SendImgTest(&dst1);

#elif defined 	OVCAM_RGB888//ԭʼRGB565תRGB888
  static EMB_IMAGE dst1;
  static SDRAM_DATA uint8_t ImgRGB3ByteDst[W*H * 3];
  EmbImageInit(&dst1,W,H,PIXEL_RGB888,(uint8_t*)ImgRGB3ByteDst);
  EmbImageConvert(&src,&dst1,RGB565_RGB888);
  BSP_USBCDC_SendImgTest(&dst1);

#elif defined 	OVCAM_GRAY8B//ԭʼRGB565װGRAY8B
  static EMB_IMAGE dst1;
  static SDRAM_DATA uint8_t ImgRGB1ByteDst1[W * H * 1];
  EmbImageInit(&dst1, W, H, PIXEL_GRAY8B, (uint8_t*) ImgRGB1ByteDst1);
  EmbImageConvert(&src, &dst1, RGB565_GRAY8B);
  BSP_USBCDC_SendImgTest(&dst1);
#endif
  BSP_OVCAM_StartSnapshot();
}

/*****************************************************************************************************/
#define CHUNK_JPGBUF_SIZE    ((uint32_t)(1024))//1K�ֽ�
#define CHUNK_MCUBUF_SIZE    ((uint32_t)(4096))//4K�ֽ�
//����ѹ��:MCU->JPEG,�Ӵ�Ļ��嵽С�Ļ���
//�����ѹ:JPEG->MCU,��С�Ļ��嵽��Ļ���
uint8_t CHUNK_JPGBUFFER[CHUNK_JPGBUF_SIZE];    //jpeg����/����С����
uint8_t CHUNK_MCUBUFFER[CHUNK_MCUBUF_SIZE];    //jpeg����/����󻺴�

struct JpegCodeParameter_t    //��RGB565��JPEG�ı���/�������
{
  FIL fil ;    //FATFS�ļ�����
  uint32_t getNb ;  //ÿ�δ�RGB565ͼ�������JPEG�ļ���ȡ�����ֽ���
  uint32_t cvtNb ;  //ÿ��RGB565->YUV��YUV->RGB565ת�����ֽ���
  uint8_t* ptr ;    //��������/�������RGB565�����׵�ַ
  uint32_t len ;    //��������/�������RGB565���ݵĳ���,�����º�֪,������ǰ֪
  uint32_t pos ;    //�Ѿ���ɵĽ�������/���������λ��
  uint8_t m ;       //0:����,1:����
};
struct JpegCodeParameter_t JpegCodePara;                          //����/�������
JPEG_RGBToYCbCr_Convert_Function pRGBToYCbCr_Convert_Function;    //RGB565->YUV422,����ǰ��Ҫ����
JPEG_YCbCrToRGB_Convert_Function pYCbCrToRGB_Convert_Function;    //YUV422->RGB565,�������Ҫ����


//��չ:��������ת����ɵĻص�����
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbCodedData)
{
  HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);
  if (JpegCodePara.m == 0)    //����
  {
    if (JpegCodePara.getNb - NbCodedData > 0)    //δ��ȫ���룬��ع���������
    {
      JpegCodePara.pos -= (JpegCodePara.getNb - NbCodedData);
    }
    if (JpegCodePara.len - JpegCodePara.pos > 0)    //��������û�б���
    {
      JpegCodePara.getNb = (JpegCodePara.len - JpegCodePara.pos > CHUNK_MCUBUF_SIZE) ? (CHUNK_MCUBUF_SIZE) : (JpegCodePara.len - JpegCodePara.pos);
      pRGBToYCbCr_Convert_Function((JpegCodePara.ptr + JpegCodePara.pos), CHUNK_MCUBUFFER, 0, JpegCodePara.getNb, &JpegCodePara.cvtNb);
      JpegCodePara.pos += JpegCodePara.cvtNb;
      HAL_JPEG_ConfigInputBuffer(hjpeg, CHUNK_MCUBUFFER, JpegCodePara.cvtNb);
    }
    else    //�Ѿ�û�и���ı�������
    {
      HAL_JPEG_ConfigInputBuffer(hjpeg, CHUNK_MCUBUFFER, 0);
    }
  }
  else    //����
  {
    if (JpegCodePara.getNb - NbCodedData > 0)    //δ��ȫ��ѹ
    {
      if (f_lseek(&JpegCodePara.fil, JpegCodePara.pos - (JpegCodePara.getNb - NbCodedData)) != FR_OK)    //��ع�ʣ�ಿ�ֵ��ļ�ָ��
      {
        HAL_JPEG_Abort(hjpeg);
      }
    }
    if (f_read(&JpegCodePara.fil, CHUNK_JPGBUFFER, CHUNK_JPGBUF_SIZE, &JpegCodePara.getNb) != FR_OK)
    {
      HAL_JPEG_Abort(hjpeg);
    }
    JpegCodePara.pos += JpegCodePara.getNb;
    HAL_JPEG_ConfigInputBuffer(hjpeg, CHUNK_JPGBUFFER, JpegCodePara.getNb);
  }
  HAL_JPEG_Resume(hjpeg, JPEG_PAUSE_RESUME_INPUT);
}

//��չ:�������ת����ɵĻص�����
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
  HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);
  if (JpegCodePara.m == 0)    //����
  {
    static UINT bw;
    if (f_write(&JpegCodePara.fil, pDataOut, OutDataLength, &bw) != FR_OK)
    {
      HAL_JPEG_Abort(hjpeg);
    }
    f_sync(&JpegCodePara.fil);
    HAL_JPEG_ConfigOutputBuffer(hjpeg, CHUNK_JPGBUFFER, CHUNK_JPGBUF_SIZE);
  }
  else    //����
  {
    pYCbCrToRGB_Convert_Function(pDataOut, (JpegCodePara.ptr + JpegCodePara.len), 0, OutDataLength, &JpegCodePara.cvtNb);
    JpegCodePara.len += JpegCodePara.cvtNb;
    HAL_JPEG_ConfigOutputBuffer(hjpeg, CHUNK_MCUBUFFER, CHUNK_MCUBUF_SIZE);
  }
  HAL_JPEG_Resume(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);
}

//��USB DISK�ļ�ϵͳ����JPEG�ļ�,��JPEG����ΪRGB565
int BSP_USBMSC_LoadJpgToRgb565(char* jpg, uint8_t* ptr, uint16_t *width, uint16_t *height)
{
  memset((uint8_t*) &JpegCodePara, 0, sizeof(JpegCodePara));
  JpegCodePara.m = 1;
  JpegCodePara.pos = 0;
  JpegCodePara.ptr = ptr;
  if (f_open(&JpegCodePara.fil, jpg, FA_READ) != FR_OK)  //��JPEG�ļ�
  {
    return -1;
  }
  if (f_read(&JpegCodePara.fil, CHUNK_JPGBUFFER, CHUNK_JPGBUF_SIZE, &JpegCodePara.getNb) != FR_OK)  //���ļ��ж�ȡ���ݵ�С������
  {
    f_close(&JpegCodePara.fil);
    return 1;
  }
  JpegCodePara.pos += JpegCodePara.getNb;  //��¼�ļ���ȡλ��
  if (HAL_JPEG_Decode(&hjpeg, CHUNK_JPGBUFFER, JpegCodePara.getNb, CHUNK_MCUBUFFER, CHUNK_MCUBUF_SIZE, 0xfff))
  {
    f_close(&JpegCodePara.fil);
    return 2;
  }
  f_close(&JpegCodePara.fil);
  JPEG_ConfTypeDef JpegConfType;
  HAL_JPEG_GetInfo(&hjpeg, &JpegConfType);
  *width = (uint16_t) JpegConfType.ImageWidth;
  *height = (uint16_t) JpegConfType.ImageHeight;
  return 0;
}

//��RGB565����ΪJPEG,��USB DISK�ļ�ϵͳ�洢JPEG�ļ�
int BSP_USBMSC_SaveJpgByRgb565(char* jpg, uint8_t* ptr, uint16_t width, uint16_t height)
{
  memset((uint8_t*) &JpegCodePara, 0, sizeof(JpegCodePara));
  JpegCodePara.m = 0;
  JpegCodePara.pos = 0;
  JpegCodePara.ptr = ptr;
  JpegCodePara.len = width * height * 2;
  if (f_open(&JpegCodePara.fil, jpg, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)  //��JPEG�ļ�
  {
    return -1;
  }
  JPEG_ConfTypeDef JpegConfType = { JPEG_YCBCR_COLORSPACE, JPEG_422_SUBSAMPLING, width, height, 90 };
  HAL_JPEG_ConfigEncoding(&hjpeg, &JpegConfType);
  JpegCodePara.getNb = (JpegCodePara.len - JpegCodePara.pos > CHUNK_MCUBUF_SIZE) ? (CHUNK_MCUBUF_SIZE) : (JpegCodePara.len - JpegCodePara.pos);
  pRGBToYCbCr_Convert_Function((JpegCodePara.ptr + JpegCodePara.pos), CHUNK_MCUBUFFER, 0, JpegCodePara.getNb, &JpegCodePara.cvtNb);
  JpegCodePara.pos += JpegCodePara.cvtNb;
  if (HAL_JPEG_Encode(&hjpeg, CHUNK_MCUBUFFER, JpegCodePara.cvtNb, CHUNK_JPGBUFFER, CHUNK_JPGBUF_SIZE, 0xfff) != HAL_OK)
  {
    f_close(&JpegCodePara.fil);
    return 1;
  }
  f_close(&JpegCodePara.fil);
  return 0;
}
/*****************************************************************************************************/

#define CDC_TX_MAX_PERBLOCK   8192  //USB VCPÿ��������ֽ���

static void BSP_USBCDC_Send(uint8_t* ptr, uint32_t num)
{
  for (uint32_t n = 0; n < num;)
  {
    while (CDC_Transmit_HS(ptr + n, (num - n >= CDC_TX_MAX_PERBLOCK) ? CDC_TX_MAX_PERBLOCK : (num - n)) != USBD_OK)
    {
      ;
    }
    n += (num - n >= CDC_TX_MAX_PERBLOCK) ? CDC_TX_MAX_PERBLOCK : (num - n);
  }
}

void BSP_USBCDC_SendTxtTest(char* ptr)
{
  BSP_USBCDC_Send((uint8_t*) ptr, strlen(ptr));
}

const uint8_t bof[2] = { 0x01, 0xfe };
const uint8_t eof[2] = { 0xfe, 0x01 };
void BSP_USBCDC_SendImgTest(EMB_IMAGE_PTR src)
{
  BSP_USBCDC_Send((uint8_t*) bof, 2);
  BSP_USBCDC_Send(src->data, src->width * src->heigh * src->pixelsize);
  BSP_USBCDC_Send((uint8_t*) eof, 2);
}
/*****************************************************************************************************/
