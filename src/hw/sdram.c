#include "sdram.h"
#include "stm32f4xx.h"

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

uint32_t GetGPIOPeriphClock(GPIO_TypeDef * GPIOx) {
    if (GPIOx == GPIOA) {
        return RCC_AHB1Periph_GPIOA;
    }
    else if (GPIOx == GPIOB) {
        return RCC_AHB1Periph_GPIOB;
    }
    else if (GPIOx == GPIOC) {
        return RCC_AHB1Periph_GPIOC;
    }
    else if (GPIOx == GPIOD) {
        return RCC_AHB1Periph_GPIOD;
    }
    else if (GPIOx == GPIOE) {
        return RCC_AHB1Periph_GPIOE;
    }
    else if (GPIOx == GPIOF) {
        return RCC_AHB1Periph_GPIOF;
    }
    else if (GPIOx == GPIOG) {
        return RCC_AHB1Periph_GPIOG;
    }
    else if (GPIOx == GPIOH) {
        return RCC_AHB1Periph_GPIOH;
    }
    else if (GPIOx == GPIOI) {
        return RCC_AHB1Periph_GPIOI;
    }
    else if (GPIOx == GPIOJ) {
        return RCC_AHB1Periph_GPIOJ;
    }
    else if (GPIOx == GPIOK) {
        return RCC_AHB1Periph_GPIOK;
    }
}

/**
 * @brief       GPIO通用设置
 * @param       GPIOx: GPIOA~GPIOI, GPIO指针
 * @param       pin: 0X0000~0XFFFF, 引脚位置, 每个位代表一个IO, 第0位代表Px0, 第1位代表Px1, 依次类推. 比如0X0101, 代表同时设置Px0和Px8.
 *   @arg       SYS_GPIO_PIN0~SYS_GPIO_PIN15, 1<<0 ~ 1<<15
 *
 * @param       mode: 0~3; 模式选择, 设置如下:
 *   @arg       SYS_GPIO_MODE_IN,  0, 输入模式(系统复位默认状态)
 *   @arg       SYS_GPIO_MODE_OUT, 1, 输出模式
 *   @arg       SYS_GPIO_MODE_AF,  2, 复用功能模式
 *   @arg       SYS_GPIO_MODE_AIN, 3, 模拟输入模式
 *
 * @param       otype: 0 / 1; 输出类型选择, 设置如下:
 *   @arg       SYS_GPIO_OTYPE_PP, 0, 推挽输出
 *   @arg       SYS_GPIO_OTYPE_OD, 1, 开漏输出
 *
 * @param       ospeed: 0~3; 输出速度, 设置如下:
 *   @arg       SYS_GPIO_SPEED_LOW,  0, 低速
 *   @arg       SYS_GPIO_SPEED_MID,  1, 中速
 *   @arg       SYS_GPIO_SPEED_FAST, 2, 快速
 *   @arg       SYS_GPIO_SPEED_HIGH, 3, 高速
 *
 * @param       pupd: 0~3: 上下拉设置, 设置如下:
 *   @arg       SYS_GPIO_PUPD_NONE, 0, 不带上下拉
 *   @arg       SYS_GPIO_PUPD_PU,   1, 上拉
 *   @arg       SYS_GPIO_PUPD_PD,   2, 下拉
 *   @arg       SYS_GPIO_PUPD_RES,  3, 保留
 *
 * @note:       注意: 在输入模式(普通输入/模拟输入)下, OTYPE和OSPEED参数无效!!
 * @retval      无
 */
void bsp_gpio_init(GPIO_TypeDef *GPIOx, uint32_t PINx, uint32_t mode, uint32_t otype, uint32_t ospeed, uint32_t pupd)
{
    uint32_t pinpos = 0, pos = 0, curpin = 0;

    for (pinpos = 0; pinpos < 16; pinpos++)
    {
        pos = 1 << pinpos;      /* 一个个位检查 */
        curpin = PINx & pos;    /* 检查引脚是否要设置 */

        if (curpin == pos)      /* 需要设置 */
        {
            GPIOx->MODER &= ~(3 << (pinpos * 2)); /* 先清除原来的设置 */
            GPIOx->MODER |= mode << (pinpos * 2); /* 设置新的模式 */

            if ((mode == 0X01) || (mode == 0X02))   /* 如果是输出模式/复用功能模式 */
            {
                GPIOx->OSPEEDR &= ~(3 << (pinpos * 2));       /* 清除原来的设置 */
                GPIOx->OSPEEDR |= (ospeed << (pinpos * 2));   /* 设置新的速度值 */
                GPIOx->OTYPER &= ~(1 << pinpos) ;             /* 清除原来的设置 */
                GPIOx->OTYPER |= otype << pinpos;             /* 设置新的输出模式 */
            }

            GPIOx->PUPDR &= ~(3 << (pinpos * 2)); /* 先清除原来的设置 */
            GPIOx->PUPDR |= pupd << (pinpos * 2); /* 设置新的上下拉 */
        }
    }
}

/**
 * @brief       GPIO复用功能选择设置
 * @param       GPIOx: GPIOA~GPIOI, GPIO指针
 * @param       PINx: 0X0000~0XFFFF, 引脚位置, 每个位代表一个IO, 第0位代表Px0, 第1位代表Px1, 依次类推. 比如0X0101, 代表同时设置Px0和Px8.
 *   @arg       SYS_GPIO_PIN0~SYS_GPIO_PIN15, 1<<0 ~ 1<<15
 * @param       AFx:0~15, 代表AF0~AF15.
 *              AF0~15设置情况(这里仅是列出常用的,详细的请见429/746数据手册,Table 12):
 *   @arg       AF0:MCO/SWD/SWCLK/RTC;      AF1:TIM1/TIM2;                  AF2:TIM3~5;                     AF3:TIM8~11
 *   @arg       AF4:I2C1~I2C4;              AF5:SPI1~SPI6;                  AF6:SPI3/SAI1;                  AF7:SPI2/3/USART1~3/UART5/SPDIFRX;
 *   @arg       AF8:USART4~8/SPDIFRX/SAI2;  AF9;CAN1~2/TIM12~14/LCD/QSPI;   AF10:USB_OTG/USB_HS/SAI2/QSPI;  AF11:ETH;
 *   @arg       AF12:FMC/SDIO/OTG/HS;       AF13:DCIM;                      AF14:LCD;                       AF15:EVENTOUT;
 * @retval      无
 */
void bsp_gpio_af_set(GPIO_TypeDef *GPIOx, uint16_t PINx, uint8_t AFx)
{
    uint32_t pinpos = 0, pos = 0, curpin = 0;;

    for (pinpos = 0; pinpos < 16; pinpos++)
    {
        pos = 1 << pinpos;      /* 一个个位检查 */
        curpin = PINx & pos;    /* 检查引脚是否要设置 */

        if (curpin == pos)      /* 需要设置 */
        {
            GPIOx->AFR[pinpos >> 3] &= ~(0X0F << ((pinpos & 0X07) * 4));
            GPIOx->AFR[pinpos >> 3] |= (uint32_t)AFx << ((pinpos & 0X07) * 4);
        }
    }
}


/**
 * @brief       设置GPIO某个引脚的输出状态
 * @param       p_gpiox: GPIOA~GPIOI, GPIO指针
 * @param       pinx: 0X0000~0XFFFF, 引脚位置, 每个位代表一个IO, 第0位代表Px0, 第1位代表Px1, 依次类推. 比如0X0101, 代表同时设置Px0和Px8.
 *   @arg       SYS_GPIO_PIN0~SYS_GPIO_PIN15, 1<<0 ~ 1<<15
 * @param       status: 0/1, 引脚状态(仅最低位有效), 设置如下:
 *   @arg       0, 输出低电平
 *   @arg       1, 输出高电平
 * @retval      无
 */
void bsp_gpio_pin_set(GPIO_TypeDef *GPIOx, uint16_t PINx, uint8_t status)
{
    if (status & 0X01)
    {
        GPIOx->BSRR |= PINx;  /* 设置GPIOx的pinx为1 */
    }
    else
    {
        GPIOx->BSRR |= (uint32_t)PINx << 16;  /* 设置GPIOx的pinx为0 */
    }
}


/**
 * @brief       读取GPIO某个引脚的状态
 * @param       p_gpiox: GPIOA~GPIOG, GPIO指针
 * @param       pinx: 0X0000~0XFFFF, 引脚位置, 每个位代表一个IO, 第0位代表Px0, 第1位代表Px1, 依次类推. 一次只能读取一个GPIO！
 *   @arg       SYS_GPIO_PIN0~SYS_GPIO_PIN15, 1<<0 ~ 1<<15
 * @retval      返回引脚状态, 0, 低电平; 1, 高电平
 */
uint8_t bsp_gpio_pin_get(GPIO_TypeDef *GPIOx, uint16_t PINx)
{
    if (GPIOx->IDR & PINx)
    {
        return 1;   /* pinx的状态为1 */
    }
    else
    {
        return 0;   /* pinx的状态为0 */
    }
}

/*
    专门用来兼容SDRAM的GPIO操作
*/
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx)
{
	GPIOx->AFR[BITx>>3]&=~(0X0F<<((BITx&0X07)*4));
	GPIOx->AFR[BITx>>3]|=(u32)AFx<<((BITx&0X07)*4);
}

/// @brief systick delay time in us
/// @param nus
void bsp_systick_delay_us(uint32_t nus) {
	SysTick->LOAD = nus * 21;
	SysTick->VAL = 0x00;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
	}
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL = 0X00;
}

/// @brief systick delay time in ms
void bsp_systick_delay_ms(uint32_t nms) {
	uint32_t i;
	for (i = 0; i < nms; i++) {
		bsp_systick_delay_us(1000);
	}
}

//向SDRAM发送命令
//bankx:0,向BANK5上面的SDRAM发送指令
//      1,向BANK6上面的SDRAM发送指令
//cmd:指令(0,正常模式/1,时钟配置使能/2,预充电所有存储区/3,自动刷新/4,加载模式寄存器/5,自刷新/6,掉电)
//refresh:自刷新次数(cmd=3时有效)
//regval:模式寄存器的定义
//返回值:0,正常;1,失败.
u8 SDRAM_Send_Cmd(u8 bankx,u8 cmd,u8 refresh,u16 regval)
{
	u32 retry=0;
	u32 tempreg=0; 
	tempreg|=cmd<<0;			//设置指令
	tempreg|=1<<(4-bankx);		//设置发送指令到bank5还是6
	tempreg|=refresh<<5;		//设置自刷新次数
	tempreg|=regval<<9;			//设置模式寄存器的值
	FMC_Bank5_6->SDCMR=tempreg;	//配置寄存器
	while((FMC_Bank5_6->SDSR&(1<<5)))//等待指令发送完成 
	{
		retry++;
		if(retry>0X1FFFFF)return 1; 
	}
	return 0;	
} 

//SDRAM初始化
void SDRAM_Init(void)
{ 
	u32 sdctrlreg=0,sdtimereg=0;
	u16 mregval=0;
	
	RCC->AHB3ENR|=1<<0;     	//使能FMC时钟  
	RCC->AHB1ENR|=0X1F<<2;		//使能PC/PD/PE/PF/PG时钟  
	
	bsp_gpio_init(GPIOC,SYS_GPIO_PIN0|SYS_GPIO_PIN2|SYS_GPIO_PIN3,SYS_GPIO_MODE_AF,SYS_GPIO_OTYPE_PP,SYS_GPIO_SPEED_HIGH,SYS_GPIO_PUPD_PU);			//PC0/2/3			
	bsp_gpio_init(GPIOD,3<<0|7<<8|3<<14,SYS_GPIO_MODE_AF,SYS_GPIO_OTYPE_PP,SYS_GPIO_SPEED_HIGH,SYS_GPIO_PUPD_PU);		//PD0/1/8/9/10/14/15		
	bsp_gpio_init(GPIOE,3<<0|0X1FF<<7,SYS_GPIO_MODE_AF,SYS_GPIO_OTYPE_PP,SYS_GPIO_SPEED_HIGH,SYS_GPIO_PUPD_PU);			//PE0/1/7~15				
	bsp_gpio_init(GPIOF,0X3F<<0|0X1F<<11,SYS_GPIO_MODE_AF,SYS_GPIO_OTYPE_PP,SYS_GPIO_SPEED_HIGH,SYS_GPIO_PUPD_PU);		//PG0~5/11~15					
	bsp_gpio_init(GPIOG,7<<0|3<<4|SYS_GPIO_PIN8|SYS_GPIO_PIN15,SYS_GPIO_MODE_AF,SYS_GPIO_OTYPE_PP,SYS_GPIO_SPEED_HIGH,SYS_GPIO_PUPD_PU);	//PF0~2/4/5/8/15				
 
 	bsp_gpio_af_set(GPIOC,SYS_GPIO_PIN0,12);	//PC0,AF12
 	bsp_gpio_af_set(GPIOC,SYS_GPIO_PIN2,12);	//PC2,AF12
 	bsp_gpio_af_set(GPIOC,SYS_GPIO_PIN3,12);	//PC3,AF12
	
 	bsp_gpio_af_set(GPIOD,SYS_GPIO_PIN0,12);	//PD0,AF12 
 	bsp_gpio_af_set(GPIOD,SYS_GPIO_PIN1,12);	//PD1,AF12 
 	bsp_gpio_af_set(GPIOD,SYS_GPIO_PIN8,12);	//PD8,AF12
 	bsp_gpio_af_set(GPIOD,SYS_GPIO_PIN9,12);	//PD9,AF12
 	bsp_gpio_af_set(GPIOD,SYS_GPIO_PIN10,12);	//PD10,AF12  
 	bsp_gpio_af_set(GPIOD,SYS_GPIO_PIN14,12);	//PD14,AF12
 	bsp_gpio_af_set(GPIOD,SYS_GPIO_PIN15,12);	//PD15,AF12
	
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN0,12);	//PE0,AF12 
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN1,12);	//PE1,AF12 
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN7,12);	//PE7,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN8,12);	//PE8,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN9,12);	//PE9,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN10,12);	//PE10,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN11,12);	//PE11,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN12,12);	//PE12,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN13,12);	//PE13,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN14,12);	//PE14,AF12
 	bsp_gpio_af_set(GPIOE,SYS_GPIO_PIN15,12);	//PE15,AF12

 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN0,12);	//PF0,AF12 
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN1,12);	//PF1,AF12 
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN2,12);	//PF2,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN3,12);	//PF3,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN4,12);	//PF4,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN5,12);	//PF5,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN11,12);	//PF11,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN12,12);	//PF12,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN13,12);	//PF13,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN14,12);	//PF14,AF12
 	bsp_gpio_af_set(GPIOF,SYS_GPIO_PIN15,12);	//PF15,AF12
	
 	bsp_gpio_af_set(GPIOG,SYS_GPIO_PIN0,12);	//PG0,AF12 
 	bsp_gpio_af_set(GPIOG,SYS_GPIO_PIN1,12);	//PG1,AF12 
 	bsp_gpio_af_set(GPIOG,SYS_GPIO_PIN2,12);	//PG2,AF12
 	bsp_gpio_af_set(GPIOG,SYS_GPIO_PIN4,12);	//PG4,AF12
 	bsp_gpio_af_set(GPIOG,SYS_GPIO_PIN5,12);	//PG5,AF12  
 	bsp_gpio_af_set(GPIOG,SYS_GPIO_PIN8,12);	//PG8,AF12
 	bsp_gpio_af_set(GPIOG,SYS_GPIO_PIN15,12);	//PG15,AF12	
    
 	sdctrlreg|=1<<0;				//9位列地址
	sdctrlreg|=2<<2;				//13位行地址
	sdctrlreg|=1<<4;				//16位数据位宽
	sdctrlreg|=1<<6;				//4个内部存区(4 BANKS)
	sdctrlreg|=3<<7;				//3个CAS延迟
	sdctrlreg|=0<<9;				//允许写访问
	sdctrlreg|=2<<10;				//SDRAM时钟=HCLK/2=192M/2=96M=10.4ns
	sdctrlreg|=1<<12;				//使能突发访问 
	sdctrlreg|=0<<13;				//读通道延迟0个HCLK
 	FMC_Bank5_6->SDCR[0]=sdctrlreg;	//设置FMC BANK5 SDRAM控制寄存器(BANK5和6用于管理SDRAM).

	sdtimereg|=1<<0;				//加载模式寄存器到激活时间的延迟为2个时钟周期
	sdtimereg|=6<<4;				//退出自刷新延迟为7个时钟周期
	sdtimereg|=5<<8;				//自刷新时间为6个时钟周期
	sdtimereg|=5<<12;				//行循环延迟为6个时钟周期
	sdtimereg|=1<<16;				//恢复延迟为2个时钟周期
	sdtimereg|=1<<20;				//行预充电延迟为2个时钟周期
	sdtimereg|=1<<24;				//行到列延迟为2个时钟周期
 	FMC_Bank5_6->SDTR[0]=sdtimereg;	//设置FMC BANK5 SDRAM时序寄存器 

	SDRAM_Send_Cmd(0,1,0,0);		//时钟配置使能
	bsp_systick_delay_us(500);					//至少延迟200us.
	SDRAM_Send_Cmd(0,2,0,0);		//对所有存储区预充电
	SDRAM_Send_Cmd(0,3,8,0);		//设置自刷新次数 
	mregval|=3<<0;					//设置突发长度:8(可以是1/2/4/8)
	mregval|=0<<3;					//设置突发类型:连续(可以是连续/交错)
	mregval|=3<<4;					//设置CAS值:3(可以是2/3)
	mregval|=0<<7;					//设置操作模式:0,标准模式
	mregval|=1<<9;					//设置突发写模式:1,单点访问
	SDRAM_Send_Cmd(0,4,0,mregval);	//设置SDRAM的模式寄存器
	
	//刷新频率计数器(以SDCLK频率计数),计算方法:
	//COUNT=SDRAM刷新周期/行数-20=SDRAM刷新周期(us)*SDCLK频率(Mhz)/行数
	//我们使用的SDRAM刷新周期为64ms,SDCLK=192/2=96Mhz,行数为8192(2^13).
	//所以,COUNT=64*1000*96/8192-20=730 
	FMC_Bank5_6->SDRTR=730<<1;		//设置刷新频率计数器
} 

//在指定地址(WriteAddr+Bank5_SDRAM_ADDR)开始,连续写入n个字节.
//pBuffer:字节指针
//WriteAddr:要写入的地址
//n:要写入的字节数
void FMC_SDRAM_WriteBuffer(u8 *pBuffer,u32 WriteAddr,u32 n)
{
	for(;n!=0;n--)
	{
		*(vu8*)(Bank5_SDRAM_ADDR+WriteAddr)=*pBuffer;
		WriteAddr++;
		pBuffer++;
	}
}

//在指定地址((WriteAddr+Bank5_SDRAM_ADDR))开始,连续读出n个字节.
//pBuffer:字节指针
//ReadAddr:要读出的起始地址
//n:要写入的字节数
void FMC_SDRAM_ReadBuffer(u8 *pBuffer,u32 ReadAddr,u32 n)
{
	for(;n!=0;n--)
	{
		*pBuffer++=*(vu8*)(Bank5_SDRAM_ADDR+ReadAddr);
		ReadAddr++;
	}
}






























