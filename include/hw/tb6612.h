//
// Created by 34575 on 25-6-8.
//

#ifndef TB6612_H
#define TB6612_H
#include "config.h"

using namespace System;
extern Dirver::GPIO tb6612;
extern Dirver::GPIO _tb6612;

//PWM接口
//#define PWMA 22
//#define PWMB 20

//方向使能接口 use the GPIOA
#define AIN_2 GPIO_Pin_1
#define AIN_1 GPIO_Pin_3
#define BIN_1 GPIO_Pin_4
#define BIN_2 GPIO_Pin_5

//PWM二次接口
//#define MotorLeft PWMA
//#define MotorRight PWMB

//方向控制
#define RightForward    do{tb6612.setPin(AIN_1); _tb6612.resetPin(AIN_2);}while(0)
#define RightBackward   do{tb6612.resetPin(AIN_1); _tb6612.setPin(AIN_2);}while(0)
#define LeftForward     do{tb6612.resetPin(BIN_1); tb6612.setPin(BIN_2);}while(0)
#define LeftBackward    do{tb6612.setPin(BIN_1); tb6612.resetPin(BIN_2);}while(0)
#define AllForward      do{LeftForward;RightForward;}while(0)
#define AllBackward     do{LeftBackward;RightBackward;}while(0)
#define AllStop         do{tb6612.resetPin(AIN_1); _tb6612.resetPin(AIN_2);tb6612.resetPin(BIN_1);tb6612.resetPin(BIN_2);}while(0)

#endif //TB6612_H
