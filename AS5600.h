#ifndef AS5600_H
#define AS5600_H  

#include <stdint.h>
#include <stdbool.h>
#define AS5600_LIB_VERSION              (F("0.6.5"))

/*
A note on HAL
The HAL (Hardware Abstraction Layer) is a set of function pointers that allow the library to interact with the hardware without being tied to a specific implementation. 
This allows for flexibility in how the library can be used, such as with different microcontrollers and their SDKs
Some SDKs may, for example, send the START or STOP condition automatically with their TX or RX functions.  In which case, your HAL implementation should not send those conditions in the i2c_start() and i2c_end() functions
Make sure to implement the HAL functions in a way that matches your specific hardware and SDK requirements.
*/

struct AS5600_HAL {
    void (*digitalWrite)(uint16_t pin, uint8_t value);
    void (*i2c_start)(uint8_t address); // SDA sent low, SCL HI, buffers address
    void (*i2c_end)(); // SDA sent high, SCL HI, release bus
    int (*i2c_writeBytes)(uint8_t *data, uint8_t length); // address + 0 write bit + data sent, returns ack
    int (*i2c_readBytes)(uint8_t *data, uint8_t length); // address + 1 read bit sent, wait for data, returns ack
    uint32_t (*micros)();
} ;

struct AS5600_DEV {
    uint8_t  address;
    uint8_t  directionPin;
    uint8_t  direction;
    int      error;


    //  for getAngularSpeed()
    uint32_t lastMeasurement;
    int16_t  lastAngle;
    int16_t  lastReadAngle;

    //  for readAngle() and rawAngle()
    uint16_t offset;


    //  EXPERIMENTAL
    //  cumulative position counter
    //  works only if the sensor is read often enough.
    int32_t  position;
    int16_t  lastPosition; 
    struct AS5600_HAL hal;
};

bool AS5600_isConnected(struct AS5600_DEV *dev);

bool AS5600_begin(struct AS5600_DEV *dev, uint8_t directionPin, struct AS5600_HAL hal);

//  address = fixed   0x36 for AS5600,
//          = default 0x40 for AS5600L

uint8_t AS5600_getAddress(struct AS5600_DEV *dev);


//  SET CONFIGURE REGISTERS
//  read datasheet first

//  0         = AS5600_CLOCK_WISE
//  1         = AS5600_COUNTERCLOCK_WISE
//  all other = AS5600_COUNTERCLOCK_WISE
void     AS5600_setDirection(struct AS5600_DEV *dev, uint8_t direction);
uint8_t  AS5600_getDirection(struct AS5600_DEV *dev);

uint8_t  AS5600_getZMCO(struct AS5600_DEV *dev);

//  0 .. 4095
//  returns false if parameter out of range
bool     AS5600_setZPosition(struct AS5600_DEV *dev, uint16_t value);
uint16_t AS5600_getZPosition(struct AS5600_DEV *dev);

//  0 .. 4095
//  returns false if parameter out of range
bool     AS5600_setMPosition(struct AS5600_DEV *dev, uint16_t value);
uint16_t AS5600_getMPosition(struct AS5600_DEV *dev);

//  0 .. 4095
//  returns false if parameter out of range
bool     AS5600_setMaxAngle(struct AS5600_DEV *dev, uint16_t value);
uint16_t AS5600_getMaxAngle(struct AS5600_DEV *dev);

//  access the whole configuration register
//  check datasheet for bit fields
//  returns false if parameter out of range
bool     AS5600_setConfigure(struct AS5600_DEV *dev, uint16_t value);
uint16_t AS5600_getConfigure(struct AS5600_DEV *dev);

//  access details of the configuration register
//  0 = Normal
//  1,2,3 are low power mode - check datasheet
//  returns false if parameter out of range
bool     AS5600_setPowerMode(struct AS5600_DEV *dev, uint8_t powerMode);
uint8_t  AS5600_getPowerMode(struct AS5600_DEV *dev);

//  0 = off    1 = lsb1    2 = lsb2    3 = lsb3
//  returns false if parameter out of range
//  suppresses noise when the magnet is not moving.
bool     AS5600_setHysteresis(struct AS5600_DEV *dev, uint8_t hysteresis);
uint8_t  AS5600_getHysteresis(struct AS5600_DEV *dev);

//  0 = analog 0-100%
//  1 = analog 10-90%
//  2 = PWM
//  returns false if parameter out of range
bool    AS5600_setOutputMode(struct AS5600_DEV *dev, uint8_t outputMode);
uint8_t AS5600_getOutputMode(struct AS5600_DEV *dev);

//  0 = 115    1 = 230    2 = 460    3 = 920 (Hz)
//  returns false if parameter out of range
bool     AS5600_setPWMFrequency(struct AS5600_DEV *dev, uint8_t pwmFreq);
uint8_t  AS5600_getPWMFrequency(struct AS5600_DEV *dev);

//  0 = 16x    1 = 8x     2 = 4x     3 = 2x
//  returns false if parameter out of range
bool     AS5600_setSlowFilter(struct AS5600_DEV *dev, uint8_t mask);
uint8_t  AS5600_getSlowFilter(struct AS5600_DEV *dev);

//  0 = none   1 = LSB6   2 = LSB7   3 = LSB9
//  4 = LSB18  5 = LSB21  6 = LSB24  7 = LSB10
//  returns false if parameter out of range
bool     AS5600_setFastFilter(struct AS5600_DEV *dev, uint8_t mask);
uint8_t  AS5600_getFastFilter(struct AS5600_DEV *dev);

//  0 = OFF
//  1 = ON   (auto low power mode)
//  returns false if parameter out of range
bool     AS5600_setWatchDog(struct AS5600_DEV *dev, uint8_t mask);
uint8_t  AS5600_getWatchDog(struct AS5600_DEV *dev);


//  READ OUTPUT REGISTERS
uint16_t AS5600_rawAngle(struct AS5600_DEV *dev);
uint16_t AS5600_readAngle(struct AS5600_DEV *dev);

//  software based offset.
//  degrees = -359.99 .. 359.99 (preferred)
//  returns false if abs(parameter) > 36000
//          => expect loss of precision
bool     AS5600_setOffset(struct AS5600_DEV *dev, float degrees);       //  sets an absolute offset
float    AS5600_getOffset(struct AS5600_DEV *dev);
bool     AS5600_increaseOffset(struct AS5600_DEV *dev, float degrees);  //  adds to existing offset.


//  READ STATUS REGISTERS
uint8_t  AS5600_readStatus(struct AS5600_DEV *dev);
uint8_t  AS5600_readAGC(struct AS5600_DEV *dev);
uint16_t AS5600_readMagnitude(struct AS5600_DEV *dev);

//  access detail status register
bool     AS5600_detectMagnet(struct AS5600_DEV *dev);
bool     AS5600_magnetTooStrong(struct AS5600_DEV *dev);
bool     AS5600_magnetTooWeak(struct AS5600_DEV *dev);


//  BURN COMMANDS
//  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
//  void burnAngle();
//  void burnSetting();


//  EXPERIMENTAL 0.1.2 - to be tested.
//  approximation of the angular speed in rotations per second.
//  mode == 1: radians /second
//  mode == 0: degrees /second  (default)
float    AS5600_getAngularSpeed(struct AS5600_DEV *dev, uint8_t mode,
                          bool update);

//  EXPERIMENTAL CUMULATIVE POSITION
//  reads sensor and updates cumulative position
int32_t  AS5600_getCumulativePosition(struct AS5600_DEV *dev, bool update);
//  converts last position to whole revolutions.
int32_t  AS5600_getRevolutions(struct AS5600_DEV *dev);
//  resets position only (not the i)
//  returns last position but not internal lastPosition.
int32_t  AS5600_resetPosition(struct AS5600_DEV *dev, int32_t position);
//  resets position and internal lastPosition
//  returns last position.
int32_t  AS5600_resetCumulativePosition(struct AS5600_DEV *dev, int32_t position);

//  EXPERIMENTAL 0.5.2
int      AS5600_lastError(struct AS5600_DEV *dev);


#endif