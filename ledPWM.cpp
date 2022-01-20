// cpp file v0.2.0

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

#include "ledPWM.h"
#include <Arduino.h>
#include <Wire.h>

PWMDriver::PWMDriver() {}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
void PWMDriver::begin_i2c(const uint8_t addr) {
  _i2c_mode = true;
  _i2c_addr = addr;
  _i2c = &Wire;
  _i2c->begin();
  
  // Send a reset command to the PCA9685 chip
  i2c_write8(PCA9685_MODE1, MODE1_RESTART);
  delay(10);

  // Set the PWM chip frequency
  setPWMFreq(PCA_PWM_FREQ, PCA_OSCILLATOR_FREQ);
  
  clear();
}

void PWMDriver::begin_gpio(const uint8_t g1, const uint8_t g2, const uint8_t g3, const uint8_t g4, const uint8_t g5) {
  ledArray[0] = g1;
  ledArray[1] = g2;
  ledArray[2] = g3;
  ledArray[3] = g4;
  ledArray[4] = g5;

  #if defined(ESP32)
  for (int i = 0; i < 5; i++)
  { 
    ledcSetup(channelArray[i], ESP32_PWM_FREQ, ESP32_PWM_RESOLUTION);
    ledcAttachPin(ledArray[i], channelArray[i]);
  }
  #endif
    
  clear();
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  pwm_freq Floating point frequency that we will attempt to match
 *  @param  oscillator_freq The internal chip frequency
 */
void PWMDriver::setPWMFreq(float pwm_freq, uint32_t oscillator_freq) {
  if (_i2c_mode) {
    // Range output modulation frequency is dependant on oscillator
    // Datasheet limit is 3052=50MHz/(4*4096)
    if (pwm_freq < 1) {
      pwm_freq = 1;
    } else if (pwm_freq > 3500) {
      pwm_freq = 3500;
    }
    
    float prescaleval = ((oscillator_freq / (pwm_freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN) {
      prescaleval = PCA9685_PRESCALE_MIN;
    }
    if (prescaleval > PCA9685_PRESCALE_MAX) {
      prescaleval = PCA9685_PRESCALE_MAX;
    }
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = i2c_read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep

    i2c_write8(PCA9685_MODE1, newmode);                             // go to sleep
    i2c_write8(PCA9685_PRESCALE, prescale); // set the prescaler
    i2c_write8(PCA9685_MODE1, oldmode);
    delay(5);

    // This sets the MODE1 register to turn on auto increment.
    i2c_write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
  }
}

void PWMDriver::setPWM(uint8_t num, uint16_t value) {
  if (_i2c_mode) {
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(PCA9685_LED0_ON_L + 4 * num);
    _i2c->write(0);
    _i2c->write(0 >> 8);
    _i2c->write(value);
    _i2c->write(value >> 8);
    _i2c->endTransmission();
  } else {
    #if defined(ESP32)
    ledcWrite(channelArray[num], value);
    #elif defined(ESP8266)
    analogWrite(ledArray[num], value);
    #endif    
  }
}


void PWMDriver::clear() {
  for (int x = 0; x <= 16; x++) {
    setPWM(x,0);
    
    prev1[x] = 0;
    prev2[x] = 0;
    prev3[x] = 0;
    prev4[x] = 0;
    prev5[x] = 0;
    
    c1Val[x] = 0;
    c2Val[x] = 0;
    c3Val[x] = 0;
    c4Val[x] = 0;
    c5Val[x] = 0;
  }
}

int PWMDriver::calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero, 
    step = 1020/step;              //   divide by 1020
  } 
  return step;
}


/* The next function is calculateVal. When the loop value, i,
*  reaches the step size appropriate for one of the
*  colors, it increases or decreases the value of that color by 1. 
*  (R, G, and B are each calculated separately.)
*/
int PWMDriver::calculateVal(int step, int val, int i) {
  if ((step) && i % step == 0) { // If step is non-zero and its time to change a     value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;           
    } 
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    } 
  }
  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  } 
  else if (val < 0) {
    val = 0;
  }
  return val;
}

void PWMDriver::checkFadeComplete(uint8_t chans) {
  if (i <= 1020) {
    complete[chans] = {true};
    
    prev1[chans] = c1Val[chans]; 
    prev2[chans] = c2Val[chans]; 
    prev3[chans] = c3Val[chans];
    prev4[chans] = c4Val[chans];
    prev5[chans] = c5Val[chans];

    i = 0;
  } else {
    i++;
  }
}


/*
 * 5 channel strip control
 */
void PWMDriver::colour(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5) {
  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1,0,255,0,4095);
  C2 = map(c2,0,255,0,4095);
  C3 = map(c3,0,255,0,4095);
  C4 = map(c4,0,255,0,4095);
  C5 = map(c5,0,255,0,4095);
 
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);
  setPWM((2+offsets),C3);
  setPWM((3+offsets),C4);
  setPWM((4+offsets),C5);

  prev1[chans] = c1;
  prev2[chans] = c2;
  prev3[chans] = c3;
  prev4[chans] = c4;
  prev5[chans] = c5;

  c1Val[chans] = c1;
  c2Val[chans] = c2;
  c3Val[chans] = c3;
  c4Val[chans] = c4;
  c5Val[chans] = c5;
}

void PWMDriver::crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5) {
  int step1 = calculateStep(prev1[chans], c1);
  int step2 = calculateStep(prev2[chans], c2); 
  int step3 = calculateStep(prev3[chans], c3);
  int step4 = calculateStep(prev4[chans], c4);
  int step5 = calculateStep(prev5[chans], c5);

  c1Val[chans] = calculateVal(step1, c1Val[chans], i);
  c2Val[chans] = calculateVal(step2, c2Val[chans], i);
  c3Val[chans] = calculateVal(step3, c3Val[chans], i);
  c4Val[chans] = calculateVal(step4, c4Val[chans], i);
  c5Val[chans] = calculateVal(step5, c5Val[chans], i);

  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1Val[chans],0,255,0,4095);
  C2 = map(c2Val[chans],0,255,0,4095);
  C3 = map(c3Val[chans],0,255,0,4095);
  C4 = map(c4Val[chans],0,255,0,4095);
  C5 = map(c5Val[chans],0,255,0,4095);
   
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);
  setPWM((2+offsets),C3);
  setPWM((3+offsets),C4);
  setPWM((4+offsets),C5);

  checkFadeComplete(chans);
}

/*
 * 4 channel strip control
 */
void PWMDriver::colour(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4) {
  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1,0,255,0,4095);
  C2 = map(c2,0,255,0,4095);
  C3 = map(c3,0,255,0,4095);
  C4 = map(c4,0,255,0,4095);
  
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);
  setPWM((2+offsets),C3);
  setPWM((3+offsets),C4);

  prev1[chans] = c1;
  prev2[chans] = c2;
  prev3[chans] = c3;
  prev4[chans] = c4;

  c1Val[chans] = c1;
  c2Val[chans] = c2;
  c3Val[chans] = c3;
  c4Val[chans] = c4;
}

void PWMDriver::crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4) {
  int step1 = calculateStep(prev1[chans], c1);
  int step2 = calculateStep(prev2[chans], c2); 
  int step3 = calculateStep(prev3[chans], c3);
  int step4 = calculateStep(prev4[chans], c4);

  c1Val[chans] = calculateVal(step1, c1Val[chans], i);
  c2Val[chans] = calculateVal(step2, c2Val[chans], i);
  c3Val[chans] = calculateVal(step3, c3Val[chans], i);
  c4Val[chans] = calculateVal(step4, c4Val[chans], i);

  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1Val[chans],0,255,0,4095);
  C2 = map(c2Val[chans],0,255,0,4095);
  C3 = map(c3Val[chans],0,255,0,4095);
  C4 = map(c4Val[chans],0,255,0,4095);
 
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);
  setPWM((2+offsets),C3);
  setPWM((3+offsets),C4);

  checkFadeComplete(chans);
}

/*
 * 3 channel strip control
 */
void PWMDriver::colour(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3) {
  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1,0,255,0,4095);
  C2 = map(c2,0,255,0,4095);
  C3 = map(c3,0,255,0,4095);
 
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);
  setPWM((2+offsets),C3);

  prev1[chans] = c1;
  prev2[chans] = c2;
  prev3[chans] = c3;
  
  c1Val[chans] = c1;
  c2Val[chans] = c2;
  c3Val[chans] = c3;
}

void PWMDriver::crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2, uint8_t c3) {
  int step1 = calculateStep(prev1[chans], c1);
  int step2 = calculateStep(prev2[chans], c2); 
  int step3 = calculateStep(prev3[chans], c3);

  c1Val[chans] = calculateVal(step1, c1Val[chans], i);
  c2Val[chans] = calculateVal(step2, c2Val[chans], i);
  c3Val[chans] = calculateVal(step3, c3Val[chans], i);

  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1Val[chans],0,255,0,4095);
  C2 = map(c2Val[chans],0,255,0,4095);
  C3 = map(c3Val[chans],0,255,0,4095);
 
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);
  setPWM((2+offsets),C3);

  checkFadeComplete(chans);
}

/*
 * 2 channel strip control
 */
void PWMDriver::colour(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2) {
  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1,0,255,0,4095);
  C2 = map(c2,0,255,0,4095);
 
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);

  prev1[chans] = c1;
  prev2[chans] = c2;
  
  c1Val[chans] = c1;
  c2Val[chans] = c2;
}

void PWMDriver::crossfade(uint8_t chans, uint8_t offsets, uint8_t c1, uint8_t c2) {
  int step1 = calculateStep(prev1[chans], c1);
  int step2 = calculateStep(prev2[chans], c2);

  c1Val[chans] = calculateVal(step1, c1Val[chans], i);
  c2Val[chans] = calculateVal(step2, c2Val[chans], i);

  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1Val[chans],0,255,0,4095);
  C2 = map(c2Val[chans],0,255,0,4095);
 
  setPWM((0+offsets),C1);
  setPWM((1+offsets),C2);

  checkFadeComplete(chans);
}

/*
 * single channel strip control
 */
void PWMDriver::colour(uint8_t chans, uint8_t offsets, uint8_t c1) {
  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1,0,255,0,4095);

  setPWM((0+offsets),C1);

  prev1[chans] = c1;

  c1Val[chans] = c1;
}

void PWMDriver::crossfade(uint8_t chans, uint8_t offsets, uint8_t c1) {
  int step1 = calculateStep(prev1[chans], c1);

  c1Val[chans] = calculateVal(step1, c1Val[chans], i);

  // changes from a 255 value to a 16 bit value as the PWM driver outputs a 16bit pulse
  C1 = map(c1Val[chans],0,255,0,4095);
 
  setPWM((0+offsets),C1);

  checkFadeComplete(chans);
}

/******************* Low level I2C interface */
uint8_t PWMDriver::i2c_read8(uint8_t addr) {
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(addr);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  return _i2c->read();
}

void PWMDriver::i2c_write8(uint8_t addr, uint8_t d) {
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(addr);
  _i2c->write(d);
  _i2c->endTransmission();
}
