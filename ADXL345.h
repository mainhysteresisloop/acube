/*
 * ADXL345.h
 *
 * Created: 19.03.2017 22:17:09
 * Author: Sergey Shelepin
 */ 


#ifndef ADXL345_H_
#define ADXL345_H_

#include "pins_config.h"
#include <stdint.h>


#define adxl345_cs_pin_as_output() ADXL345_CS_DDR  |= _BV(ADXL345_CS_PIN)
#define adxl345_cs_high()          ADXL345_CS_PORT |= _BV(ADXL345_CS_PIN)
#define adxl345_cs_low()           ADXL345_CS_PORT &= ~_BV(ADXL345_CS_PIN)

//registers
#define ADXL345_READ_REG_MASK 0x80
#define ADXL345_MB_MASK       0x40  // multiple bytes bit - to be set in order to read/write multiple bytes in one transaction


#define ADXL345_THRESH_ACT    0x24
#define ADXL345_THRESH_INACT  0x25
#define ADXL345_TIME_INACT    0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_BW_RATE       0x2C
#define ADXL345_POWER_CTL     0x2D
#define ADXL345_INT_ENABLE    0x2E
#define ADXL345_INT_SOURCE    0x30
#define ADXL345_DATA_FORMAT   0x31
#define ADXL345_DATAX0        0x32
#define ADXL345_DATAX1        0x33
#define ADXL345_DATAY0        0x34
#define ADXL345_DATAY1        0x35
#define ADXL345_DATAZ0        0x36
#define ADXL345_DATAZ1        0x37


//bits
//BW_RATE
#define ADXL_LOW_POWER   4
#define ADXL_RATE_B3     3
#define ADXL_RATE_B2     2
#define ADXL_RATE_B1     1
#define ADXL_RATE_B0     0


//POWER_CTL
#define ADXL_LINK       5
#define ADXL_AUTO_SLEEP 4
#define ADXL_MEASURE    3
#define ADXL_SLEEP      2
#define ADXL_WAKEUP_B1  1
#define ADXL_WAKEUP_B0  0

//INT_ENABLE
//INT_MAP
//INT_SOURCE
#define ADXL_DATA_READY 7
#define ADXL_SINGLE_TAP 6
#define ADXL_DOUBLE_TAP 5
#define ADXL_ACTIVITY   4
#define ADXL_INACTIVITY 3
#define ADXL_FREE_FALL  2
#define ADXL_WATERMARK  1
#define ADXL_OVERRUN    0

//DATA_FORMAT
#define ADXL_SELF_TEST  7
#define ADXL_SPI        6
#define ADXL_INT_INVERT 5
#define ADXL_FULL_RES   3
#define ADXL_JUSTIFY    2
#define ADXL_RANGE_B1   1
#define ADXL_RANGE_B0   0


uint8_t ADXL345_read_register(uint8_t addr);
void ADXL345_write_register(char addr, char value);
void ADXL345_read_xyz_row(uint8_t *ad);
void ADXL345_read_xyz(int16_t *xyz);
void ADXL345_print_details();


#endif /* ADXL345_H_ */