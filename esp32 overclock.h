#include <Arduino.h>
#include "soc/rtc_periph.h"

float timing = 1.0;

extern "C" void rom_i2c_writeReg(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t data);

#define I2C_BBPLL 0x66
#define I2C_BBPLL_ENDIV5 11
#define I2C_BBPLL_BBADC_DSMP 9
#define I2C_BBPLL_HOSTID 4
#define I2C_BBPLL_OC_LREF 2
#define I2C_BBPLL_OC_DIV_7_0 3
#define I2C_BBPLL_OC_DCUR 5

#define BBPLL_ENDIV5_VAL_480M 0xc3
#define BBPLL_BBADC_DSMP_VAL_480M 0x74

#define DIG_DBIAS_240M 7

#define I2C_WRITEREG_MASK_RTC(block, reg_add, indata) rom_i2c_writeReg_Mask(block, block##_HOSTID,  reg_add,  reg_add##_MSB,  reg_add##_LSB,  indata)
#define I2C_READREG_MASK_RTC(block, reg_add) rom_i2c_readReg_Mask(block, block##_HOSTID,  reg_add,  reg_add##_MSB,  reg_add##_LSB)
#define I2C_WRITEREG_RTC(block, reg_add, indata) rom_i2c_writeReg(block, block##_HOSTID,  reg_add, indata)
#define I2C_READREG_RTC(block, reg_add) rom_i2c_readReg(block, block##_HOSTID,  reg_add)

void overclock(uint8_t OC_LEVEL) {
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_ENDIV5, BBPLL_ENDIV5_VAL_480M);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_BBADC_DSMP, BBPLL_BBADC_DSMP_VAL_480M);

  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, DIG_DBIAS_240M);
  delayMicroseconds(3);

  uint8_t div_ref;
  uint8_t div7_0;
  uint8_t div10_8;
  uint8_t lref;
  uint8_t dcur;
  uint8_t bw;

  div_ref = 0;

  switch (OC_LEVEL){
    case 1:div7_0 = 40;timing = 1.29000000;break; // ~SQRT(5/3)x speed (~310MHz?)
    case 2:div7_0 = 48;timing = 1.41421356;break; // ~1.41421356x speed (~340MHz?)
    case 3:div7_0 = 52;timing = 1.47732683;break; // ~1.47732683x speed (~355MHz?)
    case 4:div7_0 = 56;timing = 1.54044011;break; // ~1.54044011x speed (~370MHz?) UNSTABLE
    case 5:div7_0 = 64;timing = 1.66666667;break; // ~1.66666667x speed (~400MHz?) VERY UNSTABLE
    default:div7_0 = 32;break; // 1x speed (240MHz)
  }
  div10_8 = 0;
  lref = 0;
  dcur = 6;
  bw = 3;

  uint8_t i2c_bbpll_lref = (lref << 7) | (div10_8 << 4) | (div_ref);
  uint8_t i2c_bbpll_div_7_0 = div7_0;
  uint8_t i2c_bbpll_dcur = (bw << 6) | dcur;
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_LREF, i2c_bbpll_lref);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);
}

#define Serial_begin(x) Serial.begin((x) / timing)
#define millis() ((uint64_t)(millis() / timing))
#define micros() ((uint64_t)(micros() / timing))
#define delay(x) delay((uint32_t)(x) * timing)
#define delayMicroseconds(x) delayMicroseconds((uint32_t)(x) * timing)
#define vTaskDelay(x) vTaskDelay((uint32_t)((x) * timing)
