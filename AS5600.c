//
//    FILE: AS56000.cpp
//  AUTHOR: Rob Tillaart, Dylan Taft
// VERSION: 0.6.5
// PURPOSE: Arduino library for AS5600 magnetic rotation meter
//    DATE: 2022-05-28
//     URL: https://github.com/dylanetaft/AS5600_C#


#include "AS5600.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>


//  default addresses
const uint8_t AS5600_DEFAULT_DEV_ADDRESS   = 0x36;
const uint8_t AS5600L_DEFAULT_DEV_ADDRESS   = 0x40;
const uint8_t AS5600_SW_DIRECTION_PIN   = 255;

//  setDirection
const uint8_t AS5600_CLOCK_WISE         = 0;  //  LOW
const uint8_t AS5600_COUNTERCLOCK_WISE  = 1;  //  HIGH

//  0.087890625;
const float   AS5600_RAW_TO_DEGREES     = 360.0 / 4096;
const float   AS5600_DEGREES_TO_RAW     = 4096 / 360.0;
//  0.00153398078788564122971808758949;
const float   AS5600_RAW_TO_RADIANS     = M_PI * 2.0 / 4096;
//  4.06901041666666e-6
const float   AS5600_RAW_TO_RPM         = 60.0 / 4096;

//  getAngularSpeed
const uint8_t AS5600_MODE_DEGREES       = 0;
const uint8_t AS5600_MODE_RADIANS       = 1;
const uint8_t AS5600_MODE_RPM           = 2;


//  ERROR CODES
const int     AS5600_OK                 = 0;
const int     AS5600_ERROR_I2C_READ_0   = -100;
const int     AS5600_ERROR_I2C_READ_1   = -101;
const int     AS5600_ERROR_I2C_READ_2   = -102;
const int     AS5600_ERROR_I2C_READ_3   = -103;
const int     AS5600_ERROR_I2C_WRITE_0  = -200;
const int     AS5600_ERROR_I2C_WRITE_1  = -201;


//  CONFIGURE CONSTANTS
//  check datasheet for details

//  setOutputMode
const uint8_t AS5600_OUTMODE_ANALOG_100 = 0;
const uint8_t AS5600_OUTMODE_ANALOG_90  = 1;
const uint8_t AS5600_OUTMODE_PWM        = 2;

//  setPowerMode
const uint8_t AS5600_POWERMODE_NOMINAL  = 0;
const uint8_t AS5600_POWERMODE_LOW1     = 1;
const uint8_t AS5600_POWERMODE_LOW2     = 2;
const uint8_t AS5600_POWERMODE_LOW3     = 3;

//  setPWMFrequency
const uint8_t AS5600_PWM_115            = 0;
const uint8_t AS5600_PWM_230            = 1;
const uint8_t AS5600_PWM_460            = 2;
const uint8_t AS5600_PWM_920            = 3;

//  setHysteresis
const uint8_t AS5600_HYST_OFF           = 0;
const uint8_t AS5600_HYST_LSB1          = 1;
const uint8_t AS5600_HYST_LSB2          = 2;
const uint8_t AS5600_HYST_LSB3          = 3;

//  setSlowFilter
const uint8_t AS5600_SLOW_FILT_16X      = 0;
const uint8_t AS5600_SLOW_FILT_8X       = 1;
const uint8_t AS5600_SLOW_FILT_4X       = 2;
const uint8_t AS5600_SLOW_FILT_2X       = 3;

//  setFastFilter
const uint8_t AS5600_FAST_FILT_NONE     = 0;
const uint8_t AS5600_FAST_FILT_LSB6     = 1;
const uint8_t AS5600_FAST_FILT_LSB7     = 2;
const uint8_t AS5600_FAST_FILT_LSB9     = 3;
const uint8_t AS5600_FAST_FILT_LSB18    = 4;
const uint8_t AS5600_FAST_FILT_LSB21    = 5;
const uint8_t AS5600_FAST_FILT_LSB24    = 6;
const uint8_t AS5600_FAST_FILT_LSB10    = 7;

//  setWatchDog
const uint8_t AS5600_WATCHDOG_OFF       = 0;
const uint8_t AS5600_WATCHDOG_ON        = 1;


uint8_t  readReg(struct AS5600_DEV *dev, uint8_t reg);
uint16_t readReg2(struct AS5600_DEV *dev, uint8_t reg);
uint8_t  writeReg(struct AS5600_DEV *dev, uint8_t reg, uint8_t value);
uint8_t  writeReg2(struct AS5600_DEV *dev, uint8_t reg, uint16_t value);

//  CONFIGURATION REGISTERS
const uint8_t AS5600_ZMCO = 0x00;
const uint8_t AS5600_ZPOS = 0x01;   //  + 0x02
const uint8_t AS5600_MPOS = 0x03;   //  + 0x04
const uint8_t AS5600_MANG = 0x05;   //  + 0x06
const uint8_t AS5600_CONF = 0x07;   //  + 0x08

//  CONFIGURATION BIT MASKS - byte level
const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;


//  UNKNOWN REGISTERS 0x09-0x0A

//  OUTPUT REGISTERS
const uint8_t AS5600_RAW_ANGLE = 0x0C;   //  + 0x0D
const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

// I2Cdev->address REGISTERS (AS5600L)
const uint8_t AS5600_I2CADDR   = 0x20;
const uint8_t AS5600_I2CUPDT   = 0x21;

//  STATUS REGISTERS
const uint8_t AS5600_STATUS    = 0x0B;
const uint8_t AS5600_AGC       = 0x1A;
const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C
const uint8_t AS5600_BURN      = 0xFF;

//  STATUS BITS
const uint8_t AS5600_MAGNET_HIGH   = 0x08;
const uint8_t AS5600_MAGNET_LOW    = 0x10;
const uint8_t AS5600_MAGNET_DETECT = 0x20;




bool AS5600_begin(struct AS5600_DEV *dev, uint8_t directionPin, struct AS5600_HAL hal)
{
    dev->address         = AS5600_DEFAULT_DEV_ADDRESS;
    dev->directionPin    = 255;
    dev->direction       = AS5600_CLOCK_WISE;
    dev->error           = AS5600_OK;
    dev->lastMeasurement = 0;
    dev->lastAngle       = 0;
    dev->lastReadAngle   = 0;
    dev->offset         = 0;
    dev->position        = 0;
    dev->lastPosition    = 0;
    dev->hal             = hal;
  
  assert(hal.i2c_start != NULL);
  assert(hal.i2c_end != NULL);
  assert(hal.i2c_writeBytes != NULL);
  assert(hal.i2c_readBytes != NULL);
  assert(hal.micros != NULL);
  assert(hal.digitalWrite != NULL);

  dev->directionPin = directionPin;

  AS5600_setDirection(dev, AS5600_CLOCK_WISE);

  if (! AS5600_isConnected(dev)) return false;
  return true;
}


bool AS5600_isConnected(struct AS5600_DEV *dev)
{

  dev->hal.i2c_start(dev->address);
  int ret = dev->hal.i2c_writeBytes(NULL,0);
  dev->hal.i2c_end();
  return ( ret == 0);
}


uint8_t AS5600_getAddress(struct AS5600_DEV *dev)
{
  return dev->address;
}


/////////////////////////////////////////////////////////
//
//  CONFIGURATION REGISTERS + direction pin
//
void AS5600_setDirection(struct AS5600_DEV *dev, uint8_t direction)
{
  dev->direction = direction;
  if (dev->directionPin != AS5600_SW_DIRECTION_PIN)
  {
    dev->hal.digitalWrite(dev->directionPin, direction);
  }
}


uint8_t AS5600_getDirection(struct AS5600_DEV *dev)
{
  return dev->direction;
}


uint8_t AS5600_getZMCO(struct AS5600_DEV *dev)
{
  uint8_t value = readReg(dev, AS5600_ZMCO);
  return value;
}


bool AS5600_setZPosition(struct AS5600_DEV *dev, uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(dev, AS5600_ZPOS, value);
  return true;
}


uint16_t AS5600_getZPosition(struct AS5600_DEV *dev)
{
  uint16_t value = readReg2(dev, AS5600_ZPOS) & 0x0FFF;
  return value;
}


bool AS5600_setMPosition(struct AS5600_DEV *dev, uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(dev, AS5600_MPOS, value);
  return true;
}


uint16_t AS5600_getMPosition(struct AS5600_DEV *dev)
{
  uint16_t value = readReg2(dev, AS5600_MPOS) & 0x0FFF;
  return value;
}


bool AS5600_setMaxAngle(struct AS5600_DEV *dev, uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(dev, AS5600_MANG, value);
  return true;
}


uint16_t AS5600_getMaxAngle(struct AS5600_DEV *dev)
{
  uint16_t value = readReg2(dev, AS5600_MANG) & 0x0FFF;
  return value;
}


/////////////////////////////////////////////////////////
//
//  CONFIGURATION
//
bool AS5600_setConfigure(struct AS5600_DEV *dev, uint16_t value)
{
  if (value > 0x3FFF) return false;
  writeReg2(dev, AS5600_CONF, value);
  return true;
}


uint16_t AS5600_getConfigure(struct AS5600_DEV *dev)
{
  uint16_t value = readReg2(dev, AS5600_CONF) & 0x3FFF;
  return value;
}


//  details configure
bool AS5600_setPowerMode(struct AS5600_DEV *dev, uint8_t powerMode)
{
  if (powerMode > 3) return false;
  uint8_t value = readReg(dev, AS5600_CONF + 1);
  value &= ~AS5600_CONF_POWER_MODE;
  value |= powerMode;
  writeReg(dev, AS5600_CONF + 1, value);
  return true;
}


uint8_t AS5600_getPowerMode(struct AS5600_DEV *dev)
{
  return readReg(dev, AS5600_CONF + 1) & 0x03;
}


bool AS5600_setHysteresis(struct AS5600_DEV *dev, uint8_t hysteresis)
{
  if (hysteresis > 3) return false;
  uint8_t value = readReg(dev, AS5600_CONF + 1);
  value &= ~AS5600_CONF_HYSTERESIS;
  value |= (hysteresis << 2);
  writeReg(dev, AS5600_CONF + 1, value);
  return true;
}


uint8_t AS5600_getHysteresis(struct AS5600_DEV *dev)
{
  return (readReg(dev, AS5600_CONF + 1) >> 2) & 0x03;
}


bool AS5600_setOutputMode(struct AS5600_DEV *dev, uint8_t outputMode)
{
  if (outputMode > 2) return false;
  uint8_t value = readReg(dev, AS5600_CONF + 1);
  value &= ~AS5600_CONF_OUTPUT_MODE;
  value |= (outputMode << 4);
  writeReg(dev, AS5600_CONF + 1, value);
  return true;
}


uint8_t AS5600_getOutputMode(struct AS5600_DEV *dev)
{
  return (readReg(dev, AS5600_CONF + 1) >> 4) & 0x03;
}


bool AS5600_setPWMFrequency(struct AS5600_DEV *dev, uint8_t pwmFreq)
{
  if (pwmFreq > 3) return false;
  uint8_t value = readReg(dev, AS5600_CONF + 1);
  value &= ~AS5600_CONF_PWM_FREQUENCY;
  value |= (pwmFreq << 6);
  writeReg(dev, AS5600_CONF + 1, value);
  return true;
}


uint8_t AS5600_getPWMFrequency(struct AS5600_DEV *dev)
{
  return (readReg(dev, AS5600_CONF + 1) >> 6) & 0x03;
}


bool AS5600_setSlowFilter(struct AS5600_DEV *dev, uint8_t mask)
{
  if (mask > 3) return false;
  uint8_t value = readReg(dev, AS5600_CONF);
  value &= ~AS5600_CONF_SLOW_FILTER;
  value |= mask;
  writeReg(dev, AS5600_CONF, value);
  return true;
}


uint8_t AS5600_getSlowFilter(struct AS5600_DEV *dev)
{
  return readReg(dev, AS5600_CONF) & 0x03;
}


bool AS5600_setFastFilter(struct AS5600_DEV *dev, uint8_t mask)
{
  if (mask > 7) return false;
  uint8_t value = readReg(dev, AS5600_CONF);
  value &= ~AS5600_CONF_FAST_FILTER;
  value |= (mask << 2);
  writeReg(dev, AS5600_CONF, value);
  return true;
}


uint8_t AS5600_getFastFilter(struct AS5600_DEV *dev)
{
  return (readReg(dev, AS5600_CONF) >> 2) & 0x07;
}


bool AS5600_setWatchDog(struct AS5600_DEV *dev, uint8_t mask)
{
  if (mask > 1) return false;
  uint8_t value = readReg(dev, AS5600_CONF);
  value &= ~AS5600_CONF_WATCH_DOG;
  value |= (mask << 5);
  writeReg(dev, AS5600_CONF, value);
  return true;
}


uint8_t AS5600_getWatchDog(struct AS5600_DEV *dev)
{
  return (readReg(dev, AS5600_CONF) >> 5) & 0x01;
}


/////////////////////////////////////////////////////////
//
//  OUTPUT REGISTERS
//
uint16_t AS5600_rawAngle(struct AS5600_DEV *dev)
{
  int16_t value = readReg2(dev, AS5600_RAW_ANGLE);
  if (dev->offset > 0) value += dev->offset;
  value &= 0x0FFF;

  if ((dev->directionPin == AS5600_SW_DIRECTION_PIN) &&
      (dev->direction == AS5600_COUNTERCLOCK_WISE))
  {
    value = (4096 - value) & 0x0FFF;
  }
  return value;
}


uint16_t AS5600_readAngle(struct AS5600_DEV *dev)
{
  uint16_t value = readReg2(dev, AS5600_ANGLE);
  if (dev->error != AS5600_OK)
  {
    return dev->lastReadAngle;
  }
  if (dev->offset > 0) value += dev->offset;
  value &= 0x0FFF;

  if ((dev->directionPin == AS5600_SW_DIRECTION_PIN) &&
      (dev->direction == AS5600_COUNTERCLOCK_WISE))
  {
    //  mask needed for value == 0.
    value = (4096 - value) & 0x0FFF;
  }
  dev->lastReadAngle = value;
  return value;
}


bool AS5600_setOffset(struct AS5600_DEV *dev, float degrees)
{
  //  expect loss of precision.
  if (fabsf(degrees) > 36000) return false;
  bool neg = (degrees < 0);
  if (neg) degrees = -degrees;

  uint16_t offset = round(degrees * AS5600_DEGREES_TO_RAW);
  offset &= 0x0FFF;
  if (neg) offset = (4096 - offset) & 0x0FFF;
  dev->offset = offset;
  return true;
}


float AS5600_getOffset(struct AS5600_DEV *dev)
{
  return dev->offset * AS5600_RAW_TO_DEGREES;
}


bool AS5600_increaseOffset(struct AS5600_DEV *dev, float degrees)
{
  //  add offset to existing offset in degrees.
  return AS5600_setOffset(dev, (dev->offset * AS5600_RAW_TO_DEGREES) + degrees);
}


/////////////////////////////////////////////////////////
//
//  STATUS REGISTERS
//
uint8_t AS5600_readStatus(struct AS5600_DEV *dev)
{
  uint8_t value = readReg(dev, AS5600_STATUS);
  return value;
}


uint8_t AS5600_readAGC(struct AS5600_DEV *dev)
{
  uint8_t value = readReg(dev, AS5600_AGC);
  return value;
}


uint16_t AS5600_readMagnitude(struct AS5600_DEV *dev)
{
  uint16_t value = readReg2(dev, AS5600_MAGNITUDE) & 0x0FFF;
  return value;
}


bool AS5600_detectMagnet(struct AS5600_DEV *dev)
{
  return (AS5600_readStatus(dev) & AS5600_MAGNET_DETECT) > 1;
}


bool AS5600_magnetTooStrong(struct AS5600_DEV *dev)
{
  return (AS5600_readStatus(dev) & AS5600_MAGNET_HIGH) > 1;
}


bool AS5600_magnetTooWeak(struct AS5600_DEV *dev)
{
  return (AS5600_readStatus(dev) & AS5600_MAGNET_LOW) > 1;
}


/////////////////////////////////////////////////////////
//
//  BURN COMMANDS
//
//  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
//
//  void AS5600_burnAngle()
//  {
//    writeReg(dev, AS5600_BURN, x0x80);
//  }
//
//
//  See https://github.com/RobTillaart/AS5600/issues/38
//  void AS5600_burnSetting()
//  {
//    writeReg(dev, AS5600_BURN, 0x40);
//    delay(5);
//    writeReg(dev, AS5600_BURN, 0x01);
//    writeReg(dev, AS5600_BURN, 0x11);
//    writeReg(dev, AS5600_BURN, 0x10);
//    delay(5);
//  }


float AS5600_getAngularSpeed(struct AS5600_DEV *dev, uint8_t mode, bool update)
{


  if (update)
  {
    dev->lastReadAngle = AS5600_readAngle(dev);
    if (dev->error != AS5600_OK)
    {
      return NAN;
    }
  }
  //  default behaviour
  uint32_t now     = dev->hal.micros();
  int      angle   = dev->lastReadAngle;
  uint32_t deltaT  = now - dev->lastMeasurement;
  int      deltaA  = angle - dev->lastAngle;

  //  assumption is that there is no more than 180° rotation
  //  between two consecutive measurements.
  //  => at least two measurements per rotation (preferred 4).
  if (deltaA >  2048)      deltaA -= 4096;
  else if (deltaA < -2048) deltaA += 4096;
  float speed = (deltaA * 1e6) / deltaT;

  //  remember last time & angle
  dev->lastMeasurement = now;
  dev->lastAngle       = angle;

  //  return radians, RPM or degrees.
  if (mode == AS5600_MODE_RADIANS)
  {
    return speed * AS5600_RAW_TO_RADIANS;
  }
  if (mode == AS5600_MODE_RPM)
  {
    return speed * AS5600_RAW_TO_RPM;
  }
  //  default return degrees
  return speed * AS5600_RAW_TO_DEGREES;
}


/////////////////////////////////////////////////////////
//
//  POSITION cumulative
//
int32_t AS5600_getCumulativePosition(struct AS5600_DEV *dev, bool update)
{
  if (update)
  {
    dev->lastReadAngle = AS5600_readAngle(dev);
    if (dev->error != AS5600_OK)
    {
      return dev->position;  //  last known position.
    }
  }
  int16_t value = dev->lastReadAngle;

  //  whole rotation CW?
  //  less than half a circle
  if ((dev->lastPosition > 2048) && ( value < (dev->lastPosition - 2048)))
  {
    dev->position = dev->position + 4096 - dev->lastPosition + value;
  }
  //  whole rotation CCW?
  //  less than half a circle
  else if ((value > 2048) && ( dev->lastPosition < (value - 2048)))
  {
    dev->position = dev->position - 4096 - dev->lastPosition + value;
  }
  else
  {
    dev->position = dev->position - dev->lastPosition + value;
  }
  dev->lastPosition = value;

  return dev->position;
}


int32_t AS5600_getRevolutions(struct AS5600_DEV *dev)
{
  int32_t p = dev->position >> 12;  //  divide by 4096
  if (p < 0) p++;  //  correct negative values, See #65
  return p;
}


int32_t AS5600_resetPosition(struct AS5600_DEV *dev, int32_t position)
{
  int32_t old = dev->position;
  dev->position = position;
  return old;
}


int32_t AS5600_resetCumulativePosition(struct AS5600_DEV *dev, int32_t position)
{
  dev->lastPosition = AS5600_readAngle(dev);
  int32_t old = dev->position;
  dev->position = position;
  return old;
}


int AS5600_lastError(struct AS5600_DEV *dev)
{
  int value = dev->error;
  dev->error = AS5600_OK;
  return value;
}


/////////////////////////////////////////////////////////
//
//  These are private functions
//
uint8_t readReg(struct AS5600_DEV *dev, uint8_t reg)
{

  dev->error = AS5600_OK;
  dev->hal.i2c_start(dev->address);
  int ret = dev->hal.i2c_writeBytes(&reg, 1);
  dev->hal.i2c_end();
  if (ret != 0)
  {
    dev->error = AS5600_ERROR_I2C_READ_0;
    return 0;
  }
  uint8_t data;
  dev->hal.i2c_start(dev->address);
  uint8_t n = dev->hal.i2c_readBytes(&data, 1);
  dev->hal.i2c_end();
  if (n != 1)
  {
    dev->error = AS5600_ERROR_I2C_READ_1;
    return 0;
  }

  return data;
}

uint16_t readReg2(struct AS5600_DEV *dev, uint8_t reg)
{

    dev->error = AS5600_OK;
    dev->hal.i2c_start(dev->address);
    int ret = dev->hal.i2c_writeBytes(&reg, 1);
    dev->hal.i2c_end();
    if (ret != 0) {
      dev->error = AS5600_ERROR_I2C_READ_2;
      return 0;
    }

    uint8_t data[2];  
    dev->hal.i2c_start(dev->address);
    uint8_t n = dev->hal.i2c_readBytes(data, 2); 
    dev->hal.i2c_end();
    if (n != 2) {
        dev->error = AS5600_ERROR_I2C_READ_3;
        return 0;
    }

    return ((uint16_t)data[0] << 8) | data[1];  // Big-endian
}


uint8_t writeReg(struct AS5600_DEV *dev, uint8_t reg, uint8_t value)
{

    dev->error = AS5600_OK;
    uint8_t data[2] = { reg, value };
    dev->hal.i2c_start(dev->address);
    int ret = dev->hal.i2c_writeBytes(data, 2); 
    dev->hal.i2c_end();
    if (ret != 0) {
        dev->error = AS5600_ERROR_I2C_WRITE_0;
    }

    return dev->error;
}


uint8_t writeReg2(struct AS5600_DEV *dev, uint8_t reg, uint16_t value)
{

    dev->error = AS5600_OK;
    uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),     // High byte
        (uint8_t)(value & 0xFF)    // Low byte
    };
    dev->hal.i2c_start(dev->address);
    int ret = dev->hal.i2c_writeBytes(data, 3);
    dev->hal.i2c_end();
    if (ret != 0) {
        dev->error = AS5600_ERROR_I2C_WRITE_0;
    }

    return dev->error;
}