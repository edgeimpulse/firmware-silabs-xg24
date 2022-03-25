/*
    Based on ArduCAM.cpp by Lee
    https://github.com/ArduCAM/Arduino/tree/3577fbd1b5a9f4afbe9923718fa0f265d671dc3a
    http://www.ArduCAM.com
*/
#include "ArduCAM.h"
#include <cstdio>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
extern "C" {
#include "em_cmu.h"
#include "em_eusart.h"
#include "em_gpio.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"
};

#define CAM_SPI EUSART1

#define CAM_SPI_PORT_MOSI gpioPortC
#define CAM_SPI_PIN_MOSI  3
#define CAM_SPI_PORT_MISO gpioPortC
#define CAM_SPI_PIN_MISO  2
#define CAM_SPI_PORT_SCLK gpioPortC
#define CAM_SPI_PIN_SCLK  1
#define CAM_SPI_PORT_CS   gpioPortD
#define CAM_SPI_PIN_CS    2

ArduCAM::ArduCAM()
{
    EUSART_TypeDef *cam_spi = CAM_SPI;
    EUSART_SpiInit_TypeDef init = EUSART_SPI_MASTER_INIT_DEFAULT_HF;
    EUSART_SpiAdvancedInit_TypeDef advancedInit = EUSART_SPI_ADVANCED_INIT_DEFAULT;

    sensor_addr = 0x60;

    init.bitRate = 8000000UL;
    init.advancedSettings = &advancedInit;

    advancedInit.autoCsEnable = false;
    advancedInit.msbFirst = true;

    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_EUSART1, true);

    // /* IO config */
    GPIO_PinModeSet(CAM_SPI_PORT_MOSI, CAM_SPI_PIN_MOSI, gpioModePushPull, 0);
    GPIO_PinModeSet(CAM_SPI_PORT_MISO, CAM_SPI_PIN_MISO, gpioModeInput, 0);
    GPIO_PinModeSet(CAM_SPI_PORT_SCLK, CAM_SPI_PIN_SCLK, gpioModePushPull, 0);
    GPIO_PinModeSet(CAM_SPI_PORT_CS, CAM_SPI_PIN_CS, gpioModePushPull, 1);

    EUSART_SpiInit(cam_spi, &init);

    GPIO->EUSARTROUTE[1].TXROUTE =
        ((CAM_SPI_PORT_MOSI << _GPIO_EUSART_TXROUTE_PORT_SHIFT) |
         (CAM_SPI_PIN_MOSI << _GPIO_EUSART_TXROUTE_PIN_SHIFT));
    GPIO->EUSARTROUTE[1].RXROUTE =
        ((CAM_SPI_PORT_MISO << _GPIO_EUSART_RXROUTE_PORT_SHIFT) |
         (CAM_SPI_PIN_MISO << _GPIO_EUSART_RXROUTE_PIN_SHIFT));
    GPIO->EUSARTROUTE[1].SCLKROUTE =
        ((CAM_SPI_PORT_SCLK << _GPIO_EUSART_SCLKROUTE_PORT_SHIFT) |
         (CAM_SPI_PIN_SCLK << _GPIO_EUSART_SCLKROUTE_PIN_SHIFT));
    GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN |
        GPIO_EUSART_ROUTEEN_SCLKPEN;
}

void ArduCAM::InitCAM()
{
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x12, 0x80);
    sl_sleeptimer_delay_millisecond(100);
    if (m_fmt == JPEG) {
        wrSensorRegs8_8(OV2640_JPEG_INIT);
        wrSensorRegs8_8(OV2640_YUV422);
        wrSensorRegs8_8(OV2640_JPEG);
        wrSensorReg8_8(0xff, 0x01);
        wrSensorReg8_8(0x15, 0x00);
        wrSensorRegs8_8(OV2640_320x240_JPEG);
        //wrSensorReg8_8(0xff, 0x00);
        //wrSensorReg8_8(0x44, 0x32);
    }
    else {
        wrSensorRegs8_8(OV2640_QVGA);
    }
}

bool ArduCAM::detect_camera(void)
{
    unsigned char vid, pid, temp;

    write_reg(ARDUCHIP_TEST1, 0x55);
    temp = read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55) {
        // printf("SPI interface Error!\n");
        return false;
    }

    wrSensorReg8_8(0xff, 0x01);
    rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) {
        // printf("Can't find OV2640 module!\r\n");
        return false;
    }

    return true;
}

int ArduCAM::capture_frame(uint8_t **buf)
{
    int length;
    int data_num = 0;
    bool is_header = false;
    // bool is_end = false;
    // bool errorFlag = false;
    uint8_t temp_last = 0;
    uint8_t temp = 0;
    *buf = nullptr;

    flush_fifo();
    start_capture();

    //TODO: add timeout
    while (!get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));

    length = read_fifo_length();
    if (length == 0 || length > 0x7FFFF) { //512K
        // printf("ERR: The camera length error.\r\n");
        return 0;
    }

    *buf = (uint8_t*)ei_malloc(length);
    if(*buf == nullptr) {
        // printf("ERR: failed to allocate frame buffer\n");
        return 0;
    }

    CS_LOW();
    set_fifo_burst();

    while (length--) {
        temp_last = temp;
        temp = EUSART_Spi_TxRx(CAM_SPI, 0x00);
        //Read JPEG data from FIFO
        if ((temp == 0xD9) && (temp_last == 0xFF)) { //If find the end ,break while,
            // is_end = true;
            (*buf)[data_num++] = temp;
            break;
        }

        if (is_header == true) {
            (*buf)[data_num++] = temp;
        }
        else if ((temp == 0xD8) && (temp_last == 0xFF)) {
            is_header = true;
            (*buf)[data_num++] = temp_last;
            (*buf)[data_num++] = temp;
        }
    }

    // if (is_header == false) {
    //     printf("The camera can't find the header\r\n");
    //     errorFlag = 1;
    // }

    // if (is_end == 0) {
    //     printf("The camera can't find the end\r\n");
    //     errorFlag = 1;
    // }

    // if (errorFlag) {
    //     printf("The frame error\r\n");
    // }

    CS_HIGH();
    return data_num;
}

void ArduCAM::resetFirmware(void)
{
    //Reset the CPLD
    write_reg(0x07, 0x80);
    sl_sleeptimer_delay_millisecond(100);
    write_reg(0x07, 0x00);
    sl_sleeptimer_delay_millisecond(100);
}

void ArduCAM::flush_fifo(void)
{
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCAM::start_capture(void)
{
    write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void ArduCAM::clear_fifo_flag(void)
{
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t ArduCAM::read_fifo_length(void)
{
    uint32_t len1, len2, len3, length = 0;
    len1 = read_reg(FIFO_SIZE1);
    len2 = read_reg(FIFO_SIZE2);
    len3 = read_reg(FIFO_SIZE3) & 0x7f;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return length;
}

void ArduCAM::set_fifo_burst()
{
    EUSART_Spi_TxRx(CAM_SPI, BURST_FIFO_READ);
}

void ArduCAM::CS_HIGH(void)
{
    GPIO_PinOutSet(CAM_SPI_PORT_CS, CAM_SPI_PIN_CS);
}
void ArduCAM::CS_LOW(void)
{
    GPIO_PinOutClear(CAM_SPI_PORT_CS, CAM_SPI_PIN_CS);
}

uint8_t ArduCAM::read_fifo(void)
{
    uint8_t data;
    data = bus_read(SINGLE_FIFO_READ);
    return data;
}

uint8_t ArduCAM::read_reg(uint8_t addr)
{
    uint8_t data;
    data = bus_read(addr & 0x7F);
    return data;
}

void ArduCAM::write_reg(uint8_t addr, uint8_t data)
{
    bus_write(addr | 0x80, data);
}

//Set corresponding bit
void ArduCAM::set_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp | bit);
}
//Clear corresponding bit
void ArduCAM::clear_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t ArduCAM::get_bit(uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    temp = read_reg(addr);
    temp = temp & bit;
    return temp;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void ArduCAM::set_mode(uint8_t mode)
{
    switch (mode) {
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

uint8_t ArduCAM::bus_write(int address, int value)
{
    GPIO_PinOutClear(CAM_SPI_PORT_CS, CAM_SPI_PIN_CS);
    EUSART_Spi_TxRx(CAM_SPI, address);
    EUSART_Spi_TxRx(CAM_SPI, value);
    GPIO_PinOutSet(CAM_SPI_PORT_CS, CAM_SPI_PIN_CS);
    return 1;
}

uint8_t ArduCAM::bus_read(int address)
{
    uint8_t value;
    GPIO_PinOutClear(CAM_SPI_PORT_CS, CAM_SPI_PIN_CS);
    EUSART_Spi_TxRx(CAM_SPI, address);
    value = EUSART_Spi_TxRx(CAM_SPI, 0x00);
    // take the SS pin high to de-select the chip:
    GPIO_PinOutSet(CAM_SPI_PORT_CS, CAM_SPI_PIN_CS);
    return value;
}

void ArduCAM::OV2640_set_JPEG_size(uint8_t size)
{
    switch (size) {
    case OV2640_160x120:
        wrSensorRegs8_8(OV2640_160x120_JPEG);
        break;
    case OV2640_176x144:
        wrSensorRegs8_8(OV2640_176x144_JPEG);
        break;
    case OV2640_320x240:
        wrSensorRegs8_8(OV2640_320x240_JPEG);
        break;
    case OV2640_352x288:
        wrSensorRegs8_8(OV2640_352x288_JPEG);
        break;
    case OV2640_640x480:
        wrSensorRegs8_8(OV2640_640x480_JPEG);
        break;
    case OV2640_800x600:
        wrSensorRegs8_8(OV2640_800x600_JPEG);
        break;
    case OV2640_1024x768:
        wrSensorRegs8_8(OV2640_1024x768_JPEG);
        break;
    case OV2640_1280x1024:
        wrSensorRegs8_8(OV2640_1280x1024_JPEG);
        break;
    case OV2640_1600x1200:
        wrSensorRegs8_8(OV2640_1600x1200_JPEG);
        break;
    default:
        wrSensorRegs8_8(OV2640_320x240_JPEG);
        break;
    }
}

void ArduCAM::set_format(uint8_t fmt)
{
    if (fmt == BMP)
        m_fmt = BMP;
    else if (fmt == RAW)
        m_fmt = RAW;
    else
        m_fmt = JPEG;
}

void ArduCAM::OV2640_set_Light_Mode(uint8_t Light_Mode)
{
    switch (Light_Mode) {
    case Auto:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0xc7, 0x00); //AWB on
        break;
    case Sunny:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0xc7, 0x40); //AWB off
        wrSensorReg8_8(0xcc, 0x5e);
        wrSensorReg8_8(0xcd, 0x41);
        wrSensorReg8_8(0xce, 0x54);
        break;
    case Cloudy:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0xc7, 0x40); //AWB off
        wrSensorReg8_8(0xcc, 0x65);
        wrSensorReg8_8(0xcd, 0x41);
        wrSensorReg8_8(0xce, 0x4f);
        break;
    case Office:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0xc7, 0x40); //AWB off
        wrSensorReg8_8(0xcc, 0x52);
        wrSensorReg8_8(0xcd, 0x41);
        wrSensorReg8_8(0xce, 0x66);
        break;
    case Home:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0xc7, 0x40); //AWB off
        wrSensorReg8_8(0xcc, 0x42);
        wrSensorReg8_8(0xcd, 0x3f);
        wrSensorReg8_8(0xce, 0x71);
        break;
    default:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0xc7, 0x00); //AWB on
        break;
    }
    sl_sleeptimer_delay_millisecond(100);
}

void ArduCAM::OV2640_set_Color_Saturation(uint8_t Color_Saturation)
{
    switch (Color_Saturation) {
    case Saturation2:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x02);
        wrSensorReg8_8(0x7c, 0x03);
        wrSensorReg8_8(0x7d, 0x68);
        wrSensorReg8_8(0x7d, 0x68);
        break;
    case Saturation1:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x02);
        wrSensorReg8_8(0x7c, 0x03);
        wrSensorReg8_8(0x7d, 0x58);
        wrSensorReg8_8(0x7d, 0x58);
        break;
    case Saturation0:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x02);
        wrSensorReg8_8(0x7c, 0x03);
        wrSensorReg8_8(0x7d, 0x48);
        wrSensorReg8_8(0x7d, 0x48);
        break;
    case Saturation_1:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x02);
        wrSensorReg8_8(0x7c, 0x03);
        wrSensorReg8_8(0x7d, 0x38);
        wrSensorReg8_8(0x7d, 0x38);
        break;
    case Saturation_2:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x02);
        wrSensorReg8_8(0x7c, 0x03);
        wrSensorReg8_8(0x7d, 0x28);
        wrSensorReg8_8(0x7d, 0x28);
        break;
    }
}

void ArduCAM::OV2640_set_Brightness(uint8_t Brightness)
{
    switch (Brightness) {
    case Brightness2:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x09);
        wrSensorReg8_8(0x7d, 0x40);
        wrSensorReg8_8(0x7d, 0x00);
        break;
    case Brightness1:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x09);
        wrSensorReg8_8(0x7d, 0x30);
        wrSensorReg8_8(0x7d, 0x00);
        break;
    case Brightness0:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x09);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x00);
        break;
    case Brightness_1:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x09);
        wrSensorReg8_8(0x7d, 0x10);
        wrSensorReg8_8(0x7d, 0x00);
        break;
    case Brightness_2:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x09);
        wrSensorReg8_8(0x7d, 0x00);
        wrSensorReg8_8(0x7d, 0x00);
        break;
    }
}

void ArduCAM::OV2640_set_Contrast(uint8_t Contrast)
{
    switch (Contrast) {
    case Contrast2:

        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x07);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x28);
        wrSensorReg8_8(0x7d, 0x0c);
        wrSensorReg8_8(0x7d, 0x06);
        break;
    case Contrast1:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x07);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x24);
        wrSensorReg8_8(0x7d, 0x16);
        wrSensorReg8_8(0x7d, 0x06);
        break;
    case Contrast0:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x07);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x06);
        break;
    case Contrast_1:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x07);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x2a);
        wrSensorReg8_8(0x7d, 0x06);
        break;
    case Contrast_2:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x04);
        wrSensorReg8_8(0x7c, 0x07);
        wrSensorReg8_8(0x7d, 0x20);
        wrSensorReg8_8(0x7d, 0x18);
        wrSensorReg8_8(0x7d, 0x34);
        wrSensorReg8_8(0x7d, 0x06);
        break;
    }
}

void ArduCAM::OV2640_set_Special_effects(uint8_t Special_effect)
{
    switch (Special_effect) {
    case Antique:

        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x18);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0x40);
        wrSensorReg8_8(0x7d, 0xa6);
        break;
    case Bluish:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x18);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0xa0);
        wrSensorReg8_8(0x7d, 0x40);
        break;
    case Greenish:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x18);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0x40);
        wrSensorReg8_8(0x7d, 0x40);
        break;
    case Reddish:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x18);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0x40);
        wrSensorReg8_8(0x7d, 0xc0);
        break;
    case BW:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x18);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0x80);
        wrSensorReg8_8(0x7d, 0x80);
        break;
    case Negative:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x40);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0x80);
        wrSensorReg8_8(0x7d, 0x80);
        break;
    case BWnegative:
        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x58);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0x80);
        wrSensorReg8_8(0x7d, 0x80);

        break;
    case Normal:

        wrSensorReg8_8(0xff, 0x00);
        wrSensorReg8_8(0x7c, 0x00);
        wrSensorReg8_8(0x7d, 0x00);
        wrSensorReg8_8(0x7c, 0x05);
        wrSensorReg8_8(0x7d, 0x80);
        wrSensorReg8_8(0x7d, 0x80);

        break;
    }
}

// Write 8 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_8(const struct sensor_reg reglist[])
{
    int err = 0;
    uint8_t reg_addr = 0;
    uint8_t reg_val = 0;
    const struct sensor_reg *next = reglist;

    while ((reg_addr != 0xff) | (reg_val != 0xff)) {
        reg_addr = next->reg;
        reg_val = next->val;
        err += wrSensorReg8_8(reg_addr, reg_val);
        next++;
    }

    return err;
}

// Write 16 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_16(const struct sensor_reg reglist[])
{
    int err = 0;
    uint8_t reg_addr = 0;
    uint16_t reg_val = 0;
    const struct sensor_reg *next = reglist;

    while ((reg_addr != 0xff) | (reg_val != 0xffff)) {
        reg_addr = next->reg;
        reg_val = next->val;
        err += wrSensorReg8_16(reg_addr, reg_val);
        next++;
    }

    return err;
}

// Write 8 bit values to 16 bit register address
int ArduCAM::wrSensorRegs16_8(const struct sensor_reg reglist[])
{
    int err = 0;
    uint16_t reg_addr = 0;
    uint8_t reg_val = 0;
    const struct sensor_reg *next = reglist;

    while ((reg_addr != 0xffff) | (reg_val != 0xff)) {
        reg_addr = next->reg;
        reg_val = next->val;
        err += wrSensorReg16_8(reg_addr, reg_val);
        next++;
    }

    return err;
}

//I2C Array Write 16bit address, 16bit data
int ArduCAM::wrSensorRegs16_16(const struct sensor_reg reglist[])
{
    int err = 0;
    uint16_t reg_addr = 0;
    uint16_t reg_val = 0;
    const struct sensor_reg *next = reglist;

    while ((reg_addr != 0xffff) | (reg_val != 0xffff)) {
        reg_addr = next->reg;
        reg_val = next->val;
        err += wrSensorReg16_16(reg_addr, reg_val);
        next++;
    }

    return err;
}

// Read/write 8 bit value to/from 8 bit register address
uint8_t ArduCAM::wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    seq.buf[0].data = &regID;
    seq.buf[0].len = 1;
    seq.buf[1].data = &regDat;
    seq.buf[1].len = 1;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    return ret == i2cTransferDone ? 0 : 1;
}

uint8_t ArduCAM::rdSensorReg8_8(uint8_t regID, uint8_t *regDat)
{
    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_READ;
    seq.buf[0].data = &regID;
    seq.buf[0].len = 1;
    seq.buf[1].data = regDat;
    seq.buf[1].len = 1;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    return ret == i2cTransferDone ? 0 : 1;
}

// Read/write 16 bit value to/from 8 bit register address
uint8_t ArduCAM::wrSensorReg8_16(uint8_t regID, uint16_t regDat)
{
    uint8_t buf[2];
    buf[0] = regDat >> 8;
    buf[1] = regDat & 0x00ff;

    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    seq.buf[0].data = &regID;
    seq.buf[0].len = 1;
    seq.buf[1].data = buf;
    seq.buf[1].len = 2;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    return ret == i2cTransferDone ? 0 : 1;
}

uint8_t ArduCAM::rdSensorReg8_16(uint8_t regID, uint16_t *regDat)
{
    uint8_t buf[2];

    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_READ;
    seq.buf[0].data = &regID;
    seq.buf[0].len = 1;
    seq.buf[1].data = buf;
    seq.buf[1].len = 2;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    *regDat = (buf[0] << 8) | buf[1];

    return ret == i2cTransferDone ? 0 : 1;
}

// Read/write 8 bit value to/from 16 bit register address
uint8_t ArduCAM::wrSensorReg16_8(uint16_t regID, uint8_t regDat)
{
    uint8_t buf[2];
    buf[0] = regID >> 8;
    buf[1] = regID & 0x00ff;

    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    seq.buf[0].data = buf;
    seq.buf[0].len = 2;
    seq.buf[1].data = &regDat;
    seq.buf[1].len = 1;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    return ret == i2cTransferDone ? 0 : 1;
}

uint8_t ArduCAM::rdSensorReg16_8(uint16_t regID, uint8_t *regDat)
{
    uint8_t buf[2];
    buf[0] = regID >> 8;
    buf[1] = regID & 0x00ff;

    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_READ;
    seq.buf[0].data = buf;
    seq.buf[0].len = 2;
    seq.buf[1].data = regDat;
    seq.buf[1].len = 1;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    return ret == i2cTransferDone ? 0 : 1;
}

//I2C Write 16bit address, 16bit data
uint8_t ArduCAM::wrSensorReg16_16(uint16_t regID, uint16_t regDat)
{
    uint8_t bufAddr[2];
    uint8_t bufData[2];
    bufAddr[0] = regID >> 8;
    bufAddr[1] = regID & 0x00ff;
    bufData[0] = regDat >> 8;
    bufData[1] = regDat & 0x00ff;

    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    seq.buf[0].data = bufAddr;
    seq.buf[0].len = 2;
    seq.buf[1].data = bufData;
    seq.buf[1].len = 2;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    return ret == i2cTransferDone ? 0 : 1;
}

//I2C Read 16bit address, 16bit data
uint8_t ArduCAM::rdSensorReg16_16(uint16_t regID, uint16_t *regDat)
{
    uint8_t bufAddr[2];
    uint8_t bufData[2];
    bufAddr[0] = regID >> 8;
    bufAddr[1] = regID & 0x00ff;

    /* Transfer structure */
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    /* Initializing I2C transfer */
    seq.addr = sensor_addr;
    seq.flags = I2C_FLAG_WRITE_READ;
    seq.buf[0].data = bufAddr;
    seq.buf[0].len = 2;
    seq.buf[1].data = bufData;
    seq.buf[1].len = 2;

    ret = I2CSPM_Transfer(sl_i2cspm_cam, &seq);

    *regDat = (bufData[0] << 8) | bufData[1];

    return ret == i2cTransferDone ? 0 : 1;
}
