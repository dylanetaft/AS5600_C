#ifndef AS5600_H
#define AS5600_H  

#include <stdint.h>
#include <stdbool.h>
#define AS5600_LIB_VERSION              (F("0.6.5"))

struct AS5600_HAL {
    void (*digitalWrite)(uint16_t pin, uint8_t value);
    void (*i2c_begin)(uint8_t address);
    int (*i2c_end)();
    int (*i2c_writeBytes)(uint8_t address, uint8_t *data, uint8_t length);
    int (*i2c_readBytes)(uint8_t address, uint8_t *data, uint8_t length);
    uint32_t (*micros)();
} ;

bool AS5600_isConnected();

void AS5600_setHAL(struct AS5600_HAL hal);

bool AS5600_begin(uint8_t directionPin);

//  address = fixed   0x36 for AS5600,
//          = default 0x40 for AS5600L

uint8_t AS5600_getAddress();


//  SET CONFIGURE REGISTERS
//  read datasheet first

//  0         = AS5600_CLOCK_WISE
//  1         = AS5600_COUNTERCLOCK_WISE
//  all other = AS5600_COUNTERCLOCK_WISE
void     AS5600_setDirection(uint8_t direction);
uint8_t  AS5600_getDirection();

uint8_t  AS5600_getZMCO();

//  0 .. 4095
//  returns false if parameter out of range
bool     AS5600_setZPosition(uint16_t value);
uint16_t AS5600_getZPosition();

//  0 .. 4095
//  returns false if parameter out of range
bool     AS5600_setMPosition(uint16_t value);
uint16_t AS5600_getMPosition();

//  0 .. 4095
//  returns false if parameter out of range
bool     AS5600_setMaxAngle(uint16_t value);
uint16_t AS5600_getMaxAngle();

//  access the whole configuration register
//  check datasheet for bit fields
//  returns false if parameter out of range
bool     AS5600_setConfigure(uint16_t value);
uint16_t AS5600_getConfigure();

//  access details of the configuration register
//  0 = Normal
//  1,2,3 are low power mode - check datasheet
//  returns false if parameter out of range
bool     AS5600_setPowerMode(uint8_t powerMode);
uint8_t  AS5600_getPowerMode();

//  0 = off    1 = lsb1    2 = lsb2    3 = lsb3
//  returns false if parameter out of range
//  suppresses noise when the magnet is not moving.
bool     AS5600_setHysteresis(uint8_t hysteresis);
uint8_t  AS5600_getHysteresis();

//  0 = analog 0-100%
//  1 = analog 10-90%
//  2 = PWM
//  returns false if parameter out of range
bool    AS5600_setOutputMode(uint8_t outputMode);
uint8_t AS5600_getOutputMode();

//  0 = 115    1 = 230    2 = 460    3 = 920 (Hz)
//  returns false if parameter out of range
bool     AS5600_setPWMFrequency(uint8_t pwmFreq);
uint8_t  AS5600_getPWMFrequency();

//  0 = 16x    1 = 8x     2 = 4x     3 = 2x
//  returns false if parameter out of range
bool     AS5600_setSlowFilter(uint8_t mask);
uint8_t  AS5600_getSlowFilter();

//  0 = none   1 = LSB6   2 = LSB7   3 = LSB9
//  4 = LSB18  5 = LSB21  6 = LSB24  7 = LSB10
//  returns false if parameter out of range
bool     AS5600_setFastFilter(uint8_t mask);
uint8_t  AS5600_getFastFilter();

//  0 = OFF
//  1 = ON   (auto low power mode)
//  returns false if parameter out of range
bool     AS5600_setWatchDog(uint8_t mask);
uint8_t  AS5600_getWatchDog();


//  READ OUTPUT REGISTERS
uint16_t AS5600_rawAngle();
uint16_t AS5600_readAngle();

//  software based offset.
//  degrees = -359.99 .. 359.99 (preferred)
//  returns false if abs(parameter) > 36000
//          => expect loss of precision
bool     AS5600_setOffset(float degrees);       //  sets an absolute offset
float    AS5600_getOffset();
bool     AS5600_increaseOffset(float degrees);  //  adds to existing offset.


//  READ STATUS REGISTERS
uint8_t  AS5600_readStatus();
uint8_t  AS5600_readAGC();
uint16_t AS5600_readMagnitude();

//  access detail status register
bool     AS5600_detectMagnet();
bool     AS5600_magnetTooStrong();
bool     AS5600_magnetTooWeak();


//  BURN COMMANDS
//  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
//  void burnAngle();
//  void burnSetting();


//  EXPERIMENTAL 0.1.2 - to be tested.
//  approximation of the angular speed in rotations per second.
//  mode == 1: radians /second
//  mode == 0: degrees /second  (default)
float    AS5600_getAngularSpeed(uint8_t mode,
                          bool update);

//  EXPERIMENTAL CUMULATIVE POSITION
//  reads sensor and updates cumulative position
int32_t  AS5600_getCumulativePosition(bool update);
//  converts last position to whole revolutions.
int32_t  AS5600_getRevolutions();
//  resets position only (not the i)
//  returns last position but not internal lastPosition.
int32_t  AS5600_resetPosition(int32_t position);
//  resets position and internal lastPosition
//  returns last position.
int32_t  AS5600_resetCumulativePosition(int32_t position);

//  EXPERIMENTAL 0.5.2
int      AS5600_lastError();


#endif