#ifndef _IO_H_
#define _IO_H_

#include <Arduino.h>

//1.OLED
#include "U8g2lib.h"

//2.WS2812B
// #include "WS2812FX.h"

#include <Adafruit_NeoPixel.h>

#define RGBDIN 19

#define WS_COUNT  3
#define WS_PIN    19

//3.LED
#define LEDpin 18

//4.Buzzer
#define Buzzerpin 23

//5.MITSUMI
#define EC_A 35
#define EC_K 34
#define EC_B 39

//UART2 - OPENMV4
#define OpenMV_TXD 25
#define OpenMV_RXD 26

void InitIOs();         // IO资源Initialize
void EC_Debug();        // Dial encoder debug output
void OLED_Debug();      // OLED debug

void BlinkLed(uint8_t lelpin,uint8_t blinknum); //LED blink

// PWM channels, total 16 (0-15), divided into high and low speed groups,
// High-speed channels (0-7): 80MHz clock, Low-speed channels (8-15): 1MHz clock
// 0-15都可以设置，只要不重复即可，参考上面的列表
// 如果有定时器的使用，要避开!!!
/*
 * LEDC Chan to Group/Channel/Timer Mapping
** ledc: 0  => Group: 0, Channel: 0, Timer: 0
** ledc: 1  => Group: 0, Channel: 1, Timer: 0
** ledc: 2  => Group: 0, Channel: 2, Timer: 1
** ledc: 3  => Group: 0, Channel: 3, Timer: 1
** ledc: 4  => Group: 0, Channel: 4, Timer: 2
** ledc: 5  => Group: 0, Channel: 5, Timer: 2
** ledc: 6  => Group: 0, Channel: 6, Timer: 3
** ledc: 7  => Group: 0, Channel: 7, Timer: 3
** ledc: 8  => Group: 1, Channel: 0, Timer: 0
** ledc: 9  => Group: 1, Channel: 1, Timer: 0
** ledc: 10 => Group: 1, Channel: 2, Timer: 1
** ledc: 11 => Group: 1, Channel: 3, Timer: 1
** ledc: 12 => Group: 1, Channel: 4, Timer: 2
** ledc: 13 => Group: 1, Channel: 5, Timer: 2
** ledc: 14 => Group: 1, Channel: 6, Timer: 3
** ledc: 15 => Group: 1, Channel: 7, Timer: 3
*/

void PWM_Init(int PWM_Channel, int PWM_IO);         // PWMInitialize
void PWM_Control(int PWM_Channel, int DutyA);       // PWM控制

#endif

