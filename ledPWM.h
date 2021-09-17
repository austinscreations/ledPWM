// h file v0.1.0

/*!
 *
 * This library is based of one created by Adafruit for the PCA9685,
 * it has been modified for easy control of RGB led strips via mosfets
 *
 *  @file Adafruit_PWMServoDriver.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */



#ifndef ledPWM_H
#define ledPWM_H

#include <Arduino.h>
#include <Wire.h>


// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

/*!
 *  @brief  Class that stores state and functions for interacting with PCA9685
 * PWM chip
 */
class PWMDriver {
public:
  PWMDriver();
  void begin(const uint8_t addr);
  void setPWMFreq(float freq);
  uint8_t getPWM(uint8_t num);

  void fadereset(uint8_t chans);

  void LED(uint8_t chans, uint8_t offsets, uint8_t c1);
  void LED(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2);
  void LED(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3);
  void LED(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4);
  void LED(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5);

  void crossfadeBUTTON(uint8_t chans, uint8_t offsets, uint8_t c1);
  void crossfadeBUTTON(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2);
  void crossfadeBUTTON(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3);
  void crossfadeBUTTON(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4);
  void crossfadeBUTTON(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5);

  void crossfade(uint8_t chans, uint8_t offsets, uint8_t c1);
  void crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2);
  void crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3);
  void crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4);
  void crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5);
  
  uint8_t complete[17] = {false};



private:
  void clear();
  void setOscillatorFrequency(uint32_t freq);
  void setPWM(uint8_t num, uint16_t on, uint16_t off);
  void reset();
  uint8_t _i2caddr;
  TwoWire *_i2c;
  void setOutputMode(bool totempole);

  int calculateVal(int step, int val, int i);
  int calculateStep(int prevValue, int endValue);

  uint32_t _oscillator_freq;
  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t d);

  int C1 = 0;
  int C2 = 0;
  int C3 = 0;
  int C4 = 0;
  int C5 = 0;
  int i = 0;
  int prev1[17] = {};
  int prev2[17] = {};
  int prev3[17] = {};
  int prev4[17] = {};
  int prev5[17] = {};
  int c1Val[17] = {};
  int c2Val[17] = {}; 
  int c3Val[17] = {};
  int c4Val[17] = {};
  int c5Val[17] = {};
  int step1;
  int step2; 
  int step3;
  int step4;
  int step5;
};

#endif
