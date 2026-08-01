#pragma once

#include <stdint.h>
#include <driver/gpio.h> 


#define PIN_TWAI_TX         GPIO_NUM_5
#define PIN_TWAI_RX         GPIO_NUM_4


#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22


#define PIN_POT_A           34   // ADC1_CH6
#define PIN_POT_B           35   // ADC1_CH7

#define PIN_BTN_BRAKE       25
#define PIN_BTN_START       26


#define PIN_PWM_TORQUE      18  
#define PIN_BUZZER          19
#define PIN_LED_STATUS      2    


#define LEDC_CH_TORQUE      0
#define LEDC_CH_BUZZER      1
