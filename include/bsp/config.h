//
// Created by 34575 on 25-6-3.
//

#ifndef INIT_H
#define INIT_H

#include "stm32f4xx_gpio.h"
#include "spl/stm32f4xx.h"
#include <cstddef>
#include <cstdio>
namespace System {
    inline void Init();
    inline void sleep(const uint32_t us) {
        //延时函数
        uint32_t cycles = us * (SystemCoreClock / 1000000);
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles);
    }
    inline void sleep_ms(const uint32_t ms) {
        //延时函数
        uint32_t cycles = ms * (SystemCoreClock / 1000);
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles);
    }
    namespace Dirver {
        class GPIO {
        public:
            GPIO_TypeDef *port;
            uint16_t pin;
            GPIOMode_TypeDef mode;
            GPIOSpeed_TypeDef speed = GPIO_High_Speed;
            GPIOOType_TypeDef otype = GPIO_OType_PP;
            GPIOPuPd_TypeDef pupd = GPIO_PuPd_NOPULL;
            uint8_t af;
            uint8_t pinSource;

            GPIO(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode = GPIO_Mode_OUT,
                 GPIOSpeed_TypeDef speed = GPIO_High_Speed, GPIOOType_TypeDef otype = GPIO_OType_PP,
                 GPIOPuPd_TypeDef pupd = GPIO_PuPd_NOPULL, uint8_t af = NULL, uint8_t pinSource = NULL);

            virtual ~GPIO() {
                GPIO_DeInit(port);
            }

            virtual void init(GPIOMode_TypeDef mode);

            void setPin(const uint16_t pin) const {
                GPIO_SetBits(port, pin);
            }

            void resetPin(const uint16_t pin) const {
                GPIO_ResetBits(port, pin);
            }

            [[nodiscard]] uint8_t readPin(const uint16_t pin) const {
                return GPIO_ReadInputDataBit(port, pin);
            }

            void setaf(uint16_t pin, uint8_t af);
        };

        class UART : public GPIO {
        public:
            USART_TypeDef *usart;
            uint32_t baudRate;

            UART(USART_TypeDef *usart, GPIO_TypeDef *port, uint16_t pin, uint32_t baudRate);

            ~UART() override {
                USART_DeInit(usart);
            }

            void sendByte(uint8_t byte);

            void sendData(const char *str);

            void sendData(const uint8_t *data, uint16_t size);

            int printf(const char *fmt, ...);

            uint16_t readData();
            static void sendByte(USART_TypeDef *usart, uint8_t byte) {
                while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET);
                usart->DR = (byte & static_cast<uint16_t>(0x01FF));
            }
            static uint16_t readData(const USART_TypeDef *usart) {
                return static_cast<uint16_t>(usart->DR & static_cast<uint16_t>(0x01FF));
            }
        };

        class TIM {
        public:
            virtual ~TIM() = default;

            TIM_TypeDef *tim;
            uint32_t period; // ARR
            uint16_t prescaler; // PSC
            TIM(TIM_TypeDef *tim, uint32_t period, uint16_t prescaler);
            virtual void start() {     // 作为定时器使用时,开启中断
                TIM_ITConfig(tim, TIM_IT_Update, ENABLE);
                TIM_Cmd(tim, ENABLE);
            }
        };

        class PWM : public GPIO, public TIM {
        public:
            enum Channel {
                Channel_1 = 1,
                Channel_2,
                Channel_3,
                Channel_4
            };
            PWM(GPIO_TypeDef *port, TIM_TypeDef *tim, uint16_t pin, uint32_t period, uint16_t prescaler = 1);
            virtual void setDutyCycle(Channel CCRx, uint32_t dutyCycle);
        };
        class Encoding : public GPIO, public TIM {
        public:
            int16_t speed;
            Encoding(GPIO_TypeDef *port, TIM_TypeDef *tim, uint16_t pin);
            virtual uint32_t getCount() {
                return tim->CNT; // 获取计数值
            }
            virtual int16_t GetSpeed() {
                speed = tim->CNT; // 获取速度
                tim->CNT = 0; // 清零计数器
                return speed;
            }
        };
        class Soft_IIC : public GPIO {
        protected:
            void init(GPIOMode_TypeDef mode) override {
                /*重新配置GPIO*/
                GPIO_InitTypeDef GPIO_InitStructure;
                GPIO_InitStructure.GPIO_Pin = IIC_SDA_Pin;
                GPIO_InitStructure.GPIO_Mode = mode;
                GPIO_InitStructure.GPIO_Speed = speed;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
                GPIO_Init(port, &GPIO_InitStructure);
            }
            virtual void Soft_IIC_SCL(uint8_t state) {//SCL线控制函数
                if(state){
                    setPin(IIC_SCL_Pin); // 设置SCL线为高电平
                }else{
                    resetPin(IIC_SCL_Pin); // 设置SCL线为低电平
                }
            }
            virtual void Soft_IIC_SDA(uint8_t state) {//SDA线控制函数
                if(state){
                    setPin(IIC_SDA_Pin); // 设置SDA线为高电平
                }else{
                    resetPin(IIC_SDA_Pin); // 设置SDA线为低电平
                }
            }
            virtual void Soft_SDA_IN() {//SDA线设置为输入
                init(GPIO_Mode_IN); // 设置SDA线为浮空输入
            }
            virtual void Soft_SDA_OUT() {//SDA线设置为输出
                init(GPIO_Mode_OUT); // 设置SDA线为开漏输出
            }
            virtual uint8_t Soft_READ_SDA() {//读取SDA线状态
                return readPin(IIC_SDA_Pin); // 读取SDA线状态
            }
        public:
            GPIO_TypeDef *IIC_Port;
            uint16_t IIC_SCL_Pin;
            uint16_t IIC_SDA_Pin;

            Soft_IIC(GPIO_TypeDef *iic_port, uint16_t scl, uint16_t sda) : GPIO(iic_port, scl | sda, GPIO_Mode_OUT,
                                                                               GPIO_High_Speed, GPIO_OType_OD,
                                                                               GPIO_PuPd_UP),
                                                                           IIC_Port(iic_port), IIC_SCL_Pin(scl),
                                                                           IIC_SDA_Pin(sda) {
            };
            virtual void delay_us(uint32_t nus) {
                sleep(nus);
            }
            virtual void delay_ms(uint32_t nms) {
                //延时函数
                while(nms--) {
                    sleep(1000);
                }
            }
            void GPIO_Port_Init(); //IIC GPIO端口初始化函数
            void Soft_IIC_Init(); //初始化IIC的IO口
            void Soft_IIC_Start(); //发送IIC开始信号
            void Soft_IIC_Stop(); //发送IIC停止信号
            void Soft_IIC_Send_Byte(uint8_t txd); //IIC发送一个字节
            uint8_t Soft_IIC_Receive_Byte(unsigned char ack); //IIC读取一个字节
            uint8_t Soft_IIC_Wait_Ack(); //IIC等待ACK信号
            void Soft_IIC_Ack(); //IIC发送ACK信号
            void Soft_IIC_NAck(); //IIC不发送ACK信号
            uint8_t Soft_IIC_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf); //IIC连续写
            uint8_t Soft_IIC_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf); //IIC连续读
            uint8_t Soft_IIC_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data); //IIC写一个字节
            uint8_t Soft_IIC_Read_Byte(uint8_t addr, uint8_t reg); //IIC读一个字节
        };
        class SD : public GPIO {
        public:
            SDIO_TypeDef *sdio;
        };
    }

    namespace Application {
        class LED : public Dirver::GPIO {
        public:
            LED(GPIO_TypeDef *port, uint16_t pin) : GPIO(port, pin, GPIO_Mode_OUT, GPIO_High_Speed, GPIO_OType_PP,
                                                         GPIO_PuPd_NOPULL) {
                setPin(pin);
            }

            void on() const {
                setPin(pin);
            }

            void on(const uint16_t _pin) const {
                setPin(_pin);
            }

            void off() const {
                resetPin(pin);
            }

            void off(const uint16_t _pin) const {
                resetPin(_pin);
            }

            void toggle() const {
                if (readPin(pin)) {
                    resetPin(pin);
                } else {
                    setPin(pin);
                }
            }

            void toggle(const uint16_t _pin) const {
                if (readPin(_pin)) {
                    resetPin(_pin);
                } else {
                    setPin(_pin);
                }
            }
        };

        class Button : public Dirver::GPIO {
        public:
            Button(GPIO_TypeDef *port, uint16_t pin, GPIOPuPd_TypeDef pupd = GPIO_PuPd_UP) : GPIO(
                port, pin, GPIO_Mode_IN, GPIO_High_Speed, GPIO_OType_PP, pupd) {
            }

            bool isOn() {
                return readPin(pin) == 0; // Assuming active low button
            }

            bool isOn(uint16_t _pin) {
                return readPin(_pin) == 0; // Assuming active low button
            }
        };

        class Servos_Motor : public Dirver::PWM {
        public:
            Servos_Motor(GPIO_TypeDef *port, TIM_TypeDef *tim, uint16_t pin, uint32_t period = 9600,
                         uint16_t prescaler = 5) : PWM(port, tim, pin, period, prescaler) {
            }
            void setDutyCycle(Channel CCRx, uint32_t dutyCycle) override;
            void setAugle(Channel CCRx, uint32_t angle) {
                uint32_t pulseWidth = (angle * period) / 100; // Convert percentage to pulse width
                setDutyCycle(CCRx, pulseWidth);
            }
        };


        class Shell : public Dirver::GPIO, Dirver::UART {

        };
    }
}

extern "C" {    /*中断要在C环境下执行*/
    void USART1_IRQHandler(void);
    void USART2_IRQHandler(void);
    void USART3_IRQHandler(void);
    void TIM3_IRQHandler(void);
    void TIM2_IRQHandler(void);
}


#endif //INIT_H
