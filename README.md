# ledPWM
Building on my i2cRGB library, this library can control multiple types of strips

2022-01-19 Updated to support both I2C and GPIO based PWM channels

```c++
#include <ledPWM.h>                   // For PWM LED controller library

PWMDriver pwmDriver;                  // For PWM control via PCA9685

void setup() {

  // PCA9685 - I2C expandable board
  pwmDriver.begin_i2c(0x40);

  // ESP32 - LilyGO Shield (GPIO in channel order)
  pwmDriver.begin_gpio(14,04,12,15,16);

  // ESP8266 - D1 Mini Shield - (GPIO in channel order)
  pwmDriver.begin_gpio(15,13,12,14,5);
}
```

2021-09-16 changes made to library to make it easier to declare multiple boards. used in the https://github.com/austinscreations/OXRS-AC-LedController-ESP32-FW

2021-08-13 added library files. to see examples of this in use see my PWM lighting controller: https://github.com/austinscreations/PWM-LED-Controller
