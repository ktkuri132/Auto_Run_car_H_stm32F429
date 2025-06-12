//
// Created by 34575 on 25-6-2.
//
#include <cstdio>

#include "bsp/config.h"
#include "rtos/FreeRTOS.h"
#include "rtos/task.h"
#include "hw/OLED.h"
#include "hw/inv_mpu.h"
#include <shell.h>
#include "Serial.h"
#include "tb6612.h"
#include "control.h"
#include <env.h>
#include <iosfwd>
#include <streambuf>
using namespace System;

SysData data;
Bie_ShellTypeDef USART1_Deal;
Dirver::UART usart1(USART1, GPIOA, GPIO_Pin_9 | GPIO_Pin_10, 115200); /*串口的初始化不能放在SYstem::Init函数中不知道为什么*/
Dirver::UART uart4(UART4, GPIOC, GPIO_Pin_10 | GPIO_Pin_11, 115200);
Application::Button sw1(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);
Application::Button sw2(GPIOB, GPIO_Pin_13);
Application::LED led(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
Application::LED buzzer(GPIOA,GPIO_Pin_8);
Dirver::PWM Motor(GPIOA, TIM2, GPIO_Pin_0 | GPIO_Pin_1, 9000, 1); // 初始化PWM
Dirver::Encoding leftAB(GPIOD, TIM4,GPIO_Pin_12 | GPIO_Pin_13); // 初始化编码器
Dirver::Encoding rightAB(GPIOH, TIM5, GPIO_Pin_10 | GPIO_Pin_11); // 初始化编码器
Dirver::GPIO tb6612(GPIOA, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
Dirver::GPIO _tb6612(GPIOC, GPIO_Pin_1);
/**
 * 远离OLED侧为右轮,靠近侧为左轮,左轮转得快一些
 */
Control::Upright_Control left_Control(left_Motor,600,10.1,-1.2,9000,500,1000);
Control::Upright_Control right_Control(right_Motor,600,10.1,-1.2,9000,500,1000);
Control::Speed_Control Speed_Control(left_Motor, -2.9f, -0.02f, 0.0f, 20, 14);
Control::Filter::KalmanFilter gy_kalman(0.01f, 100.0f, 0.0f); // 创建卡尔曼滤波器实例
Control::Filter::LowPassFilter gy_lowpass(0.3f); // 创建低通滤波器实例

/*
 *今天出了一个傻逼的BUG,efp从RTOS出来以后的操作全没了,像tm的局部变量一样
 */
EFP efp; // 定义环境函数指针结构体实例

extern EnvVar MyEnvVar[20];
extern DeviceFamily default_log;


void Debug_log(void *pvParameters) {
    for (;;) {
        uart4.printf("%d,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f\n",
            leftAB.speed - rightAB.speed,
            left_Control.derivative_value,
            left_Control.output,
            Speed_Control.integral_value,
            Speed_Control.output,
            data.gyro[data.y],
            left_Control.integral_value,
            data.pitch,
            data.acc[data.y]
            );
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief 主线程函数
 * @param pvParameters 主线程参数
 */
void Main_Thread(void *pvParameters) {
    buzzer.off();
    Sys_Cmd_Init();
    MCU_Shell_Init(&USART1_Deal, &default_log); // 初始化Shell协议结构体

    for (;;) {
        if (efp.envpfunc != NULL) {
            efp.envpfunc(efp.argc, efp.Parameters); // 执行系统函数
            efp.envpfunc = NULL;
            printf(CURSOR_SHOW);
        }
        mpu_dmp_get_data(&data.pitch, &data.roll, &data.yaw);
        mpu_get_gyro_reg(data.gyro,NULL);
        mpu_get_accel_reg(data.acc,NULL);
    }
}

void TIM3_IRQHandler_CallBack(void *pvParameters) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Speed_Control.control(0);
        Control::Filter::LowPassFilter lowpass(0.3f);
        Control::Filter::KalmanFilter kalman(0.01f, 10.0f, 0.0f);
        left_Control.control(kalman.Update(lowpass.update(Speed_Control.output)),data.pitch);
        right_Control.control(kalman.Update(lowpass.update(Speed_Control.output)),data.pitch);
        if (data.pitch > 50 || data.pitch < -50) {
            AllStop; // 如果pitch大于50或小于-50,则停止
        }
    }
}

void USART1_IRQHandler_CallBack(void *pvParameters) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        taskENTER_CRITICAL();
        taskEXIT_CRITICAL();
    }
}

void System::Init() {
    // 在main或系统初始化时调用一次
    /*初始化DWT,以准备延迟函数*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    NVIC_SetPriority(TIM3_IRQn, 6);
    NVIC_SetPriority(USART1_IRQn, 7);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    /* 初始化OLED MPU6050  */
    mpu_dmp_init();
    OLED_Init();
}

TaskHandle_t TIM3_CallBackHandle;
TaskHandle_t USART1_CallBackHandle;

int main() {
    Init();
    Dirver::TIM tim3(TIM3, (10 * 1000), 27);
    xTaskCreate(Debug_log, "Debug_log", 1000, NULL, 3, NULL);
    xTaskCreate(Main_Thread, "Main_Thread", 1400, NULL, 3, NULL);
    xTaskCreate(TIM3_IRQHandler_CallBack, "TIM3_IRQHandler", 1000, NULL, 5, &TIM3_CallBackHandle);
    xTaskCreate(USART1_IRQHandler_CallBack, "USART1_IRQHandler", 1000, MyEnvVar, 4, &USART1_CallBackHandle);
    vTaskStartScheduler();
    for (;;);
    return 0;
}


void TIM3_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 用于指示是否需要切换任务
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        vTaskGenericNotifyGiveFromISR(TIM3_CallBackHandle,tskDEFAULT_INDEX_TO_NOTIFY, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // Clear the interrupt flag
    }
}


void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint16_t data = usart1.readData();
        BIE_UART(&data, &USART1_Deal, MyEnvVar, &default_log); // 处理串口数据
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); // 清除中断标志
    }
}

void vApplicationMallocFailedHook(void) {
    // 内存分配失败钩子函数
    printf("Memory allocation failed!\n");
    for (;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // 堆栈溢出钩子函数
    printf("Stack overflow in task: %s\n", pcTaskName);
    for (;;);
}

void vApplicationIdleHook(void) {
    // 空闲任务钩子函数
    // 可以在这里执行低功耗模式或其他空闲任务
    __WFI(); // 进入等待中断模式
}
