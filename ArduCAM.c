/*
 * arducam.c
 *
 *  Created on: Oct 26, 2021
 *      Author: a0232137
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART2.h>
//#include <ti/sysbios/knl/Task.h> // not sure about that
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "FreeRTOS.h"
#include "ti_drivers_config.h"
#include "memorysaver.h"
#include "ArduCAM.h"
#include "ov2640_regs.h"
#include "task.h"
#include <stdio.h>
#include <unistd.h>

#define INFERENCE (1)
#define V2 (1)
#define RDP (0)

#define CENTER_CROP (1)
#define RGB565_CONV (1)
//Before Cropping 240 X 320 X 2
//After  Cropping 224 x 224 X2
#define NUM_ROW      (240)
#define NUM_COLS     (640)
#define NUM_CROP_ROW (8)
#define NUM_CROP_COL (96)
#define EFF_COLS     (224)

#define MASK5 (0x1F)
#define MASK6 (0x3F)

#define HIGH    1
#define LOW     0

#define TEST_VAL    0x55

#if INFERENCE
#include <tinie/mobileNet.h>
uint8_t flush_input[3][224] = {0};
#endif

uint8_t inputMobileNet[3][EFF_COLS];

char msg[50];
char msg1[]="\r\n";
char msg2[LINELEN];
uint8_t txBuffer[LINELEN], rxBuffer[LINELEN];
uint32_t spiLen;
extern SemaphoreP_Handle sem;

SPI_Handle spi;
I2C_Handle i2c;
extern Display_Handle display;
uint8_t m_fmt;
uint32_t img_len, img_height, img_width;

extern UART2_Handle  uartHandle;

const uint8_t bmp_header[BMPIMAGEOFFSET] =
{
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};


void spi_reset()
{
    SPI_Params spiParams;

    SPI_transferCancel(spi);
    SPI_close(spi);

    SPI_Params_init(&spiParams);
    spiParams.dataSize = 8;
    spiParams.bitRate = 1000000;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spi = SPI_open(CONFIG_SPI_MASTER, &spiParams);

}

uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat) {
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];
    bool transferOK;

    i2cTransaction.targetAddress = SENSOR_ADDR;
    i2cTransaction.writeCount = 2;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 0;

    txBuffer[0] = regID;
    txBuffer[1] = regDat;

    transferOK = I2C_transfer(i2c, &i2cTransaction);
    if (!transferOK) {
        return 1;
    }
    return 0;
}

uint16_t wrSensorRegs8_8(struct sensor_reg arr[]) {
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];
    uint16_t i = 0;
    bool transferOK;

    i2cTransaction.targetAddress = SENSOR_ADDR;
    i2cTransaction.writeCount = 2;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 0;

    while ((arr[i].reg != 0xff) | (arr[i].val != 0xff)) {
        txBuffer[0] = (uint8_t) arr[i].reg;
        txBuffer[1] = (uint8_t) arr[i].val;

        transferOK = I2C_transfer(i2c, &i2cTransaction);
        if (!transferOK) {
            return 1;
        }
        i++;
    }
    return 0;
}

uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat) {
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[1], rxBuffer[1];
    bool transferOK;

    i2cTransaction.targetAddress = SENSOR_ADDR;
    i2cTransaction.writeCount = 1;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    txBuffer[0] = regID;

    transferOK = I2C_transfer(i2c, &i2cTransaction);
    if (!transferOK) {
        return 1;
    }
    *regDat = rxBuffer[0];
    return 0;
}

void OV2640_set_JPEG_size(uint8_t size) {
    uint8_t failed = 0;

    switch(size) {
        case OV2640_160x120:
            failed |= wrSensorRegs8_8(OV2640_160x120_JPEG);
            break;
        case OV2640_176x144:
            failed |= wrSensorRegs8_8(OV2640_176x144_JPEG);
            break;
        case OV2640_320x240:
            failed |= wrSensorRegs8_8(OV2640_320x240_JPEG);
            break;
        case OV2640_352x288:
            failed |= wrSensorRegs8_8(OV2640_352x288_JPEG);
            break;
        case OV2640_640x480:
            failed |= wrSensorRegs8_8(OV2640_640x480_JPEG);
            break;
        case OV2640_800x600:
            failed |= wrSensorRegs8_8(OV2640_800x600_JPEG);
            break;
        case OV2640_1024x768:
            failed |= wrSensorRegs8_8(OV2640_1024x768_JPEG);
            break;
        case OV2640_1280x1024:
            failed |= wrSensorRegs8_8(OV2640_1280x1024_JPEG);
            break;
        case OV2640_1600x1200:
            failed |= wrSensorRegs8_8(OV2640_1600x1200_JPEG);
            break;
        default:
            failed |= wrSensorRegs8_8(OV2640_320x240_JPEG);
            break;
    }

    if (failed) {
        while(1);
    }
}

void set_format() {
    m_fmt = M_FMT;
}

void flush_fifo() {
    write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
}

void start_capture() {
    write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void clear_fifo_flag() {
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t read_fifo_length() {
    uint32_t len1,len2,len3,length=0;
    len1 = read_reg(FIFO_SIZE1);
    len2 = read_reg(FIFO_SIZE2);
    len3 = read_reg(FIFO_SIZE3) & 0x7f;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return length;
}

void set_bit(uint8_t addr, uint8_t bit) {
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp | bit);
}

void clear_bit(uint8_t addr, uint8_t bit) {
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp & (~bit));
}

uint8_t get_bit(uint8_t addr, uint8_t bit) {
    uint8_t temp;
    temp = read_reg(addr);
    temp = temp & bit;
    return temp;
}

void set_mode(uint8_t mode) {
    switch (mode)
    {
        case MCU2LCD_MODE:
            write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
            break;
        case CAM2LCD_MODE:
            write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
            break;
        case LCD2MCU_MODE:
            write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
            break;
        default:
            write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
            break;
    }
}

uint8_t read_reg(uint8_t addr) {
    SPI_Transaction transaction;
    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];
    bool transferOK;

    memset((void *) txBuffer, 0, sizeof(txBuffer));
    memset((void *) rxBuffer, 0, sizeof(rxBuffer));

    txBuffer[0] = addr & 0x7f;

    transaction.count = 2;
    transaction.txBuf = (void *) txBuffer;
    transaction.rxBuf = (void *) rxBuffer;

    GPIO_write(CONFIG_SPI_CS, LOW);
    transferOK = SPI_transfer(spi, &transaction);
    GPIO_write(CONFIG_SPI_CS, HIGH);
    if (!transferOK) {
    }

    return rxBuffer[1];
}

void write_reg(uint8_t addr, uint8_t data) {
    SPI_Transaction transaction;
    uint8_t txBuffer[2], rxBuffer[2];
    bool transferOK;

    txBuffer[0] = addr | 0x80;
    txBuffer[1] = data;

    transaction.count = 2;
    transaction.txBuf = txBuffer;
    transaction.rxBuf = rxBuffer;

    GPIO_write(CONFIG_SPI_CS, LOW);
    transferOK = SPI_transfer(spi, &transaction);
    GPIO_write(CONFIG_SPI_CS, HIGH);
    if (!transferOK) {
    }

}

void InitCAM() {
    uint8_t failed = 0;

    failed |= wrSensorReg8_8(0xff, 0x01);
    failed |= wrSensorReg8_8(0x12, 0x80);
    usleep(100000);
    if (m_fmt == JPEG) {
        failed |= wrSensorRegs8_8(OV2640_JPEG_INIT);
        failed |= wrSensorRegs8_8(OV2640_YUV422);
        failed |= wrSensorRegs8_8(OV2640_JPEG);
        failed |= wrSensorReg8_8(0xff, 0x01);
        failed |= wrSensorReg8_8(0x15, 0x00);
    }
    else {
        failed |= wrSensorRegs8_8(OV2640_QVGA);
    }

    if (failed) {
        //Display_printf(display, 0, 0, "Unsuccessful Init CAM. Hanging...");
        while (1);
    }
}

void Arducam_start_capture() {
    // start capture
    flush_fifo();
    clear_fifo_flag();
    start_capture();
}

//Convert Raw Image Data to 565
// 240 * 320 * 2
// 224 * 224 * 2
//Input Data Shape: 1 X 448  (224 pixels, each pixel is 2B)
//Output Data Shape: 3 X 224
// [b0, b1, ...... b223]
// [g0, g1, ...... g223]
// [r0, r1, .......r223]
//each pixel is of 2 bytes rgb format - b15:0 -- b4:0 --> blue, b11:5 --> green, b15:11 --> red
void  tinie_raw_image_conversion(uint8_t* input, uint16_t size, uint8_t*output)
{
    uint16_t i;
    uint16_t rgb565;
    uint8_t b, g, r;
    uint16_t numCol = size/2;

    for(i = 0; i < numCol; i++)
    {
        rgb565 = (*(input+i*2) << 8) + (*(input+i*2+1));
        b = (rgb565 & MASK5) << 3;
        g = ((rgb565 >> 5) & MASK6) << 2;
        r = ((rgb565 >> 11) & MASK5) << 3;
        *(output+i) = r;
        *(output+i+numCol) = g;
        *(output+i+numCol*2) = b;
    }

}

void tinie_image_processing(uint8_t* image, uint16_t numCols)
{
#if RGB565_CONV
    uint8_t* pCropImage = image + NUM_CROP_COL;
    tinie_raw_image_conversion(pCropImage, numCols - NUM_CROP_COL*2, (uint8_t*)inputMobileNet);

#if INFERENCE
#if V2
    inferenceStart_v2(inputMobileNet, 0);
#endif
#if RDP
    inferenceStart(inputMobileNet, 0); // Comment for camera debug
#endif
#else
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < EFF_COLS; j++)
        {
            sprintf(msg2," %d", inputMobileNet[i][j]);
            UART2_write(uartHandle, msg2, strlen(msg2), NULL); // This is where we print data to uart
        }
        UART2_write(uartHandle, msg1, strlen(msg1), NULL);
    }
#endif
#else
    /*uint16_t i;
    for (i = NUM_CROP_COL; i < numCols - NUM_CROP_COL; i++)
    {
        sprintf(msg2," %d", image[i]);
        UART2_write(uartHandle, msg2, strlen(msg2), NULL);
    }
    UART2_write(uartHandle, msg1, strlen(msg1), NULL);*/

#endif
}

uint32_t Arducam_read_image(void) {
    SPI_Transaction spiTransaction;
    uint32_t len;
    uint32_t j = 0;
    uint8_t temp = 0, temp_last = 0;
    bool isHeader, transferOK;
    uint8_t curRow = 0;

    isHeader = false;

    len = read_fifo_length();
    sprintf(msg,"Fifo length = %d \r\n", len);
    UART2_write(uartHandle, msg, strlen(msg), NULL);

    if (len >= MAX_FIFO_SIZE) {
     sprintf(msg,"len is > MAX_FIFO_SIZE len = %3d \r\n", len);
     UART2_write(uartHandle, msg, strlen(msg), NULL);
     return 1;
    } else if (len == 0) {
     sprintf(msg2,"len = %d \r\n", len);
     UART2_write(uartHandle, msg2, strlen(msg2), NULL);
     return 1;
    }

    sprintf(msg,"Reading %d bytes of image. \r\n", len);
    UART2_write(uartHandle, msg, strlen(msg), NULL);
    sprintf(msg,"Start of image. \r\n"); // RAW Print
    UART2_write(uartHandle, msg, strlen(msg), NULL);

#if INFERENCE
#if V2
    inferenceInit_v2();
#endif
#if RDP
     inferenceInit(); // Comment for camera debug
#endif
#endif

    spiTransaction.count = 1;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = rxBuffer;

    txBuffer[0] = BURST_FIFO_READ;
    GPIO_write(CONFIG_SPI_CS, LOW);
    transferOK = SPI_transfer(spi, &spiTransaction);

    // while 1 - clean buffer
    spiTransaction.count = 1;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = rxBuffer;
    txBuffer[0] = 0;
    transferOK = SPI_transfer(spi, &spiTransaction);
    temp = rxBuffer[0];

    if (m_fmt == RAW)
    {
#if !CENTER_CROP
        sprintf(msg2," %i", temp);
        UART2_write(uartHandle, msg2, strlen(msg2), NULL);
#endif
        spiTransaction.count = LINELEN-1;
        spiTransaction.txBuf = txBuffer;
        spiTransaction.rxBuf = rxBuffer;
        transferOK = SPI_transfer(spi, &spiTransaction);

        curRow++;

#if !CENTER_CROP
        for (j = 0; j < LINELEN-1; j++)
        {
             temp = rxBuffer[j];
             sprintf(msg2," %i", temp);
             UART2_write(uartHandle, msg2, strlen(msg2), NULL);
        }
        UART2_write(uartHandle, msg1, strlen(msg1), NULL);
        len -= (LINELEN+8);    //find out why we are getting extra 8 byte for length
#endif
    }
    else
    {
        len--;
    }

    SemaphoreP_Status isAvailable = SemaphoreP_TIMEOUT;
    while (len)
    {
        /* Acquire semaphore */
        isAvailable = SemaphoreP_pend(sem, SemaphoreP_TIMEOUT);
        if (isAvailable != SemaphoreP_OK) {
            break;
        }

        spiLen = len > LINELEN ? LINELEN: len;
        len -= spiLen;
        spiTransaction.count = spiLen;
        spiTransaction.txBuf = txBuffer;
        spiTransaction.rxBuf = rxBuffer;

        transferOK = SPI_transfer(spi, &spiTransaction);

        if (!transferOK)
        {
            sprintf(msg,"SPI error\r\n");
            UART2_write(uartHandle, msg, strlen(msg), NULL);
            while(1);
        }

        if (m_fmt == JPEG)
        {
            for (j = 0; j < spiLen; j++)
            {
                temp_last = temp;
                temp = rxBuffer[j];

                if (isHeader == true)
                {
                    sprintf(msg2," %02x", temp);
                    UART2_write(uartHandle, msg2, strlen(msg2), NULL);
                }
                else if ((temp == 0xD8) && (temp_last == 0xFF))
                {
                    isHeader = true;
                    sprintf(msg,"Start of image. \r\n");
                    UART2_write(uartHandle, msg, strlen(msg), NULL);
                    sprintf(msg2,"ff d8");
                    UART2_write(uartHandle, msg2, strlen(msg2), NULL);
                }

                if ((temp == 0xD9) && (temp_last == 0xFF))
                {
                    sprintf(msg,"\n End of image. \r\n");
                    UART2_write(uartHandle, msg, strlen(msg), NULL);
                    GPIO_write(CONFIG_SPI_CS, HIGH);
                    clear_fifo_flag();
                    return 0;
                }
            }
        }
        else
        {
#if CENTER_CROP
           curRow++;
           /* Release semaphore after reading one row data through spi */
           SemaphoreP_post(sem); // Curr

           if (curRow <= NUM_CROP_ROW)
           {
               continue;
           }
           else if (curRow > (NUM_ROW - NUM_CROP_ROW))
           {
#if INFERENCE
#if V2
                inferenceStart_v2(flush_input, 0);
                inferenceStart_v2(flush_input, 1);
#endif
#if RDP
                inferenceStart(flush_input, 0); // Comment for camera debug
                inferenceStart(flush_input, 1); // Comment for camera debug
#endif
#else
               sprintf(msg2,"End of image \r\n ");
               UART2_write(uartHandle, msg2, strlen(msg2), NULL);
#endif
               /* clear spiLen to stop inferencing till next image is captured and first row data is read */
               spiLen = 0;
               break;
           }
           tinie_image_processing(rxBuffer, spiLen);

#else
           for (j = 0; j < spiLen; j++)
           {
              sprintf(msg2," %d", rxBuffer[j]);
              UART2_write(uartHandle, msg2, strlen(msg2), NULL);
           }

           UART2_write(uartHandle, msg1, strlen(msg1), NULL);
           if(len == 0)
           {
               //end of RAW image
               sprintf(msg2,"End of image\r\n ");
               UART2_write(uartHandle, msg2, strlen(msg2), NULL);
               break;
           }
#endif
        }
    }

    GPIO_write(CONFIG_SPI_CS, HIGH);
    clear_fifo_flag();

    return 0;
}

uint8_t Arducam_image_ready() {
    return get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK);
}

void Arducam_init() {
    I2C_Params i2cParams;
    SPI_Params spiParams;
    uint8_t vid, pid;

    GPIO_setConfig(CONFIG_SPI_CS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_SPI_CS, HIGH);

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2c = I2C_open(CONFIG_I2C_ARDUCAM, &i2cParams);
    if (i2c == NULL) {
        sprintf(msg,"I2C_open failed = %d \r\n", i2c);
        UART2_write(uartHandle, msg, strlen(msg), NULL);
        return;
    }

    SPI_Params_init(&spiParams);
    spiParams.dataSize = 8;
    spiParams.bitRate = 1000000;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spi = SPI_open(CONFIG_SPI_MASTER, &spiParams);
    if (spi == NULL) {
        sprintf(msg,"SPI open failed = %d \r\n", i2c);
        UART2_write(uartHandle, msg, strlen(msg), NULL);
        return;
    }

    // Reset the CPLD
    write_reg(0x07, 0x80);
    usleep(100000);
    write_reg(0x07, 0x00);
    usleep(100000);

    uint8_t temp;
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    while(1){
      //Check if the ArduCAM SPI bus is OK
      write_reg(ARDUCHIP_TEST1, TEST_VAL);
      usleep(100000);
      temp = read_reg(ARDUCHIP_TEST1);
      if (temp != TEST_VAL) {
          char msg[50];
          sprintf(msg,"ACK CMD SPI interface Error!\r\n");
          UART2_write(uartHandle, msg, strlen(msg), NULL);
          usleep(1000000);
          continue;
      }
      else {
          GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

          char msg[50];
          uint8_t len = 0;
          sprintf(msg,"ACK CMD SPI interface OK. \r\n");
          len = strlen(msg);
          UART2_write(uartHandle, msg, len, NULL);
          break;
      }
    }

    // Check if the camera module type is OV2640
    while (1) {
        wrSensorReg8_8(0xff, 0x01);
        usleep(10000);
        rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
        usleep(10000);
        rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
        usleep(10000);
        if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 )))
        {
            usleep(10000);
            continue;
        } else{
            char msg[50];
            sprintf(msg,"ACK CMD OV2640 detected.\r\n");
            UART2_write(uartHandle, msg, strlen(msg), NULL);
            break;
        }
    }

    set_format();
    InitCAM();
    OV2640_set_JPEG_size(IMG_SIZE);
}


