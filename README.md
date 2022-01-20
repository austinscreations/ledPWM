# ledPWM
Building on my i2cRGB library, this library can control multiple types of strips

2022-01-19 Now desigend to control esp32 and esp8266 pwm directly from GPIO up to 5ch of PWM

```c++
#include <ledPWM.h>                   // For PWM LED controller library

#define I2C_ADDRESS 0x40              // PCA9685 I2C address

//PWMDriver pwmDriver;                // For PWM control via PCA9685

//ESP32_PWMDriver pwmDriver;          // For PWM control via ESP32 GPIO

//ESP8266_PWMDriver pwmDriver;        // For PWM control VIA ESP8266 GPIO


void setup() {

//pwmDriver.begin(I2C_ADDRESS);       // PCA9685 - i2c expandable board

//pwmDriver.begin(14,04,12,15,16);    // ESP32   - lilyGO Shield - .begin(GPIO in channel order)

//pwmDriver.begin(15,13,12,14,5);     // ESP8266 - D1 Mini board - .begin(GPIO in channel order)

}
```

2021-09-16 changes made to library to make it easier to declare multiple boards. used in the https://github.com/austinscreations/OXRS-AC-LedController-ESP32-FW

2021-08-13 added library files. to see examples of this in use see my PWM lighting controller: https://github.com/austinscreations/PWM-LED-Controller
