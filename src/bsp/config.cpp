//
// Created by 34575 on 25-6-2.
//

#include "spl/stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "../../include/bsp/config.h"
#include <cassert>
#include <stdarg.h>
using namespace System;

Dirver::GPIO::GPIO(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode ,
                 GPIOSpeed_TypeDef speed , GPIOOType_TypeDef otype ,
                 GPIOPuPd_TypeDef pupd, uint8_t af ,uint8_t pinSource ) :
    port(port), pin(pin), mode(mode), speed(speed), otype(otype), pupd(pupd), af(af), pinSource(pinSource) {
    /* 使能GPIO时钟 */
    if (port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (port == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else if (port == GPIOE) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    } else if (port == GPIOF) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    } else if (port == GPIOG) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    } else if (port == GPIOH) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    }else {
        assert(port);
        return;
    }
    /*   配置GPIO   */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_InitStructure.GPIO_Speed = speed;
    GPIO_InitStructure.GPIO_OType = otype;
    GPIO_InitStructure.GPIO_PuPd = pupd;
    GPIO_Init(port, &GPIO_InitStructure);
}

void Dirver::GPIO::init(GPIOMode_TypeDef mode) {
    /*重新配置GPIO*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_InitStructure.GPIO_Speed = speed;
    GPIO_InitStructure.GPIO_OType = otype;
    GPIO_InitStructure.GPIO_PuPd = pupd;
    GPIO_Init(port, &GPIO_InitStructure);
}


void Dirver::GPIO::setaf(uint16_t pin, uint8_t af) {
    for (uint8_t i = 0; i < 16; ++i) {
        if (pin & (1 << i)) {
            GPIO_PinAFConfig(port, i, af);
        }
    }
}


Dirver::UART::UART(USART_TypeDef *usart,GPIO_TypeDef *port,uint16_t pin, uint32_t baudRate) :
    GPIO(port, pin, GPIO_Mode_AF, GPIO_High_Speed, GPIO_OType_PP, GPIO_PuPd_NOPULL),
        usart(usart), baudRate(baudRate) {
    if (usart == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        af = GPIO_AF_USART1;
    } else if (usart == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        af = GPIO_AF_USART2;
    } else if (usart == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        af = GPIO_AF_USART3;
    } else if (usart == UART4) {
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
        af = GPIO_AF_UART4;
    } else if (usart == UART5) {
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
        af = GPIO_AF_UART5;
    } else if (usart == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        af = GPIO_AF_USART6;
    }
    setaf(pin, af);
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(usart, &USART_InitStructure);
    USART_ITConfig(usart, USART_IT_RXNE, ENABLE);
    USART_Cmd(usart, ENABLE);
}

void Dirver::UART::sendByte(uint8_t byte) {
    while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET);
    USART_SendData(usart, byte);
}

void Dirver::UART::sendData(const char *str) {
    while (*str) {
        sendByte((uint8_t)*str++);
    }
}

void Dirver::UART::sendData(const uint8_t *data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        sendByte(data[i]);
    }
}

uint16_t Dirver::UART::readData() {
    return USART_ReceiveData(usart);
}

int Dirver::UART::printf(const char *fmt, ...) {
    int j;
    char buf[256];

    va_list arg;
    va_start(arg, fmt);
    j = vsprintf(buf, fmt, arg);
    va_end(arg);

    for (int i = 0; i < j; i++) {
        while (!(usart->SR & USART_SR_TXE));
        usart->DR = buf[i];
    }
    return j;
}


extern "C" int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE)); // 等待发送缓冲区空闲
        USART1->DR = ptr[i]; // 发送数据
    }
    return len;
}

Dirver::TIM::TIM(TIM_TypeDef *tim, uint32_t period, uint16_t prescaler) :
    tim(tim), period(period), prescaler(prescaler) {
    if (tim == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    } else if (tim == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    } else if (tim == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    } else if (tim == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    } else if (tim == TIM5) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    } else if (tim == TIM6) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    } else if (tim == TIM7) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    } else if (tim == TIM8) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    }
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = period - 1; // ARR
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1; // PSC
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
    this->start();
}

Dirver::PWM::PWM(GPIO_TypeDef *port, TIM_TypeDef *tim, uint16_t pin, uint32_t period, uint16_t prescaler  ) :
    GPIO(port, pin, GPIO_Mode_AF, GPIO_High_Speed, GPIO_OType_PP, GPIO_PuPd_NOPULL),
        TIM(tim,period,prescaler){
    if (tim == TIM1) {
        af = GPIO_AF_TIM1;
    } else if (tim == TIM2) {
        af = GPIO_AF_TIM2;
    } else if (tim == TIM3) {
        af = GPIO_AF_TIM3;
    } else if (tim == TIM4) {
        af = GPIO_AF_TIM4;
    } else if (tim == TIM5) {
        af = GPIO_AF_TIM5;
    } else if (tim == TIM8) {
        af = GPIO_AF_TIM8;
    }
    setaf(pin,af);
    // 设置定时器的PWM模式
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(tim, &TIM_OCInitStructure);
    TIM_OC2Init(tim, &TIM_OCInitStructure);
    TIM_OC3Init(tim, &TIM_OCInitStructure);
    TIM_OC4Init(tim, &TIM_OCInitStructure);
    TIM_CtrlPWMOutputs(tim, ENABLE);
    TIM_Cmd(tim, ENABLE); // 启动定时器
}

void Dirver::PWM::setDutyCycle(Channel CCRx, uint32_t dutyCycle) {
    switch (CCRx) {
        case Channel_1:
            tim->CCR1 = dutyCycle;
            break;
        case Channel_2:
            tim->CCR2 = dutyCycle;
            break;
        case Channel_3:
            tim->CCR3 = dutyCycle;
            break;
        case Channel_4:
            tim->CCR4 = dutyCycle;
            break;
    }
}

void Application::Servos_Motor::setDutyCycle(Channel CCRx, uint32_t dutyCycle) {
    if (dutyCycle > 9600) {
        // Ensure duty cycle is within the range for servo motors
        printf("Servos Motor DutyCycle error: Duty cycle must be less than or equal to 9600.\n");
        return;
    }
    if (dutyCycle < 1600 || dutyCycle > 8000) {
        // Ensure duty cycle is within the range for servo motors
        printf("Servos Motor DutyCycle error: Duty cycle must be between 1600 and 8000.\n");
        return;
    }
    switch (CCRx) {
        case Channel_1:
            tim->CCR1 = dutyCycle;
            break;
        case Channel_2:
            tim->CCR2 = dutyCycle;
            break;
        case Channel_3:
            tim->CCR3 = dutyCycle;
            break;
        case Channel_4:
            tim->CCR4 = dutyCycle;
            break;
    }
}


Dirver::Encoding::Encoding(GPIO_TypeDef *port, TIM_TypeDef *tim, uint16_t pin) :
    GPIO(port, pin, GPIO_Mode_AF, GPIO_High_Speed, GPIO_OType_PP, GPIO_PuPd_NOPULL),
    TIM(tim, 0xffff, 1){
    if (tim == TIM1) {
        af = GPIO_AF_TIM1;
    } else if (tim == TIM2) {
        af = GPIO_AF_TIM2;
    } else if (tim == TIM3) {
        af = GPIO_AF_TIM3;
    } else if (tim == TIM4) {
        af = GPIO_AF_TIM4;
    } else if (tim == TIM5) {
        af = GPIO_AF_TIM5;
    } else if (tim == TIM8) {
        af = GPIO_AF_TIM8;
    }
    setaf(pin, af);
    TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(tim, ENABLE); // 启动定时器
}

