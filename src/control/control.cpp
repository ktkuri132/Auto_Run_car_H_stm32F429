//
// Created by 34575 on 25-6-6.
//

#include "control.h"
#include "tb6612.h"
using namespace Control;
using namespace System;


extern SysData data;
extern Dirver::PWM Motor;
extern Dirver::Encoding leftAB;
extern Dirver::Encoding rightAB;




/**
 * @brief 小车速度环控制函数
 * @param target 速度环目标值
 */
void Speed_Control::control(float target) {
    float left_current_speed = 0;
    float right_current_speed = 0;
    left_current_speed = static_cast<float>(leftAB.GetSpeed());
    right_current_speed = static_cast<float>(rightAB.GetSpeed());
    Filter::LowPassFilter left_speed_filter(0.3f);
    Filter::LowPassFilter right_speed_filter(0.3f);
    left_current_speed = left_speed_filter.update(left_current_speed);
    right_current_speed = right_speed_filter.update(right_current_speed);
    update(target, left_current_speed - right_current_speed);
}

void Turn_Control::control(float target,float current) {
    update(target,current);

}
/**
 * @brief 小车直立环控制函数
 * @param target 目标角度,传入速度环输入
 * @param current 角度当前值
 */
void Upright_Control::control(float target,float current) {
    update(target,current);
    float tmp_out = output; // 获取输出值
    if (Chx == left_Motor) {
        if (tmp_out >= 0) {
            LeftForward;
        } else {
            tmp_out = -tmp_out;
            LeftBackward;
        }
    } else if (Chx == right_Motor) {
        if (tmp_out >= 0) {
            RightForward;
        } else {
            tmp_out = -tmp_out;
            RightBackward;
        }
    }
    uint32_t tmp = (uint32_t)tmp_out;
    if (data.pitch > 40 || data.pitch < -40) {
        Motor.setDutyCycle(Chx,9000);
    } else {
        Motor.setDutyCycle(Chx,tmp + (uint32_t)min_output);
    }
}
