#include "Serial.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef Basic_Serial

#endif

#ifdef ATOM_Serial  // 正点原子串口协议,用于兼容性交互

#endif

#ifdef STDE_Serial  // st,d,..e 串口协议,用于常规通信




// 初始化函数，模拟类的构造函数
void Stde_DataTypeDef_Init(struct Stde_DataTypeDef *data) {
    data->CMD_Callback            = NULL;
    data->DR_Eevet_Callback       = NULL;
    data->UART_DATA_TYPE_Callback = NULL;
    data->c                       = 0;
    data->UART_Strat              = 0;
    data->UART_End                = 0;
    data->Res_Data_type           = 0;  
    data->UART_DATA_TYPE          = 0;
    data->Res_note                = 0;
    data->Res_len                 = 0;
    data->UART_NOTE               = 0;
    data->UART_LEN                = 0;
    memset(data->UART_NOTE_LEN, 0, sizeof(data->UART_NOTE_LEN));
    memset(data->USART_array, 0, sizeof(data->USART_array));
    memset(data->Data, 0, sizeof(data->Data));
}


/// @brief STDE串口协议处理函数
uint8_t STDE_UART(void* Parameters,struct Stde_DataTypeDef *DataTypeStruct) {
    // usart.bsp_usart_x_receive(Parameters, &DataTypeStruct->c);  // 接收数据
    if (DataTypeStruct->c == 's')     // 如果是开始字符
    {
        DataTypeStruct->UART_Strat    = 1;  // 开始接收
        DataTypeStruct->UART_End      = 0;  // 关闭结束标志
        DataTypeStruct->Res_Data_type = 1;  // 开始检测数据格式
        DataTypeStruct->Res_len       = 0;  // 下标清零
        DataTypeStruct->Res_note      = 0;  // 数据节点清零
        return 0;
    } else if (DataTypeStruct->c == 'e')  // 如果是结束字符
    {
        DataTypeStruct->UART_NOTE_LEN[DataTypeStruct->Res_note] =
            DataTypeStruct->Res_len;  // 保存最后一次数据节点的长度
        DataTypeStruct->UART_Strat = 0;                             // 重新开始
        DataTypeStruct->UART_End   = 1;                             // 打开结束标志
        DataTypeStruct->UART_LEN   = DataTypeStruct->Res_len;       // 保存本次数据长度
        DataTypeStruct->UART_NOTE  = DataTypeStruct->Res_note + 1;  // 保存本次数据节点
        DataTypeStruct->Res_note = 0;  // 数据节点清零
        DataTypeStruct->Res_len  = 0;  // 下标清零
        memcpy(DataTypeStruct->Data, DataTypeStruct->USART_array,
               sizeof(DataTypeStruct->USART_array));  // 将数据拷贝到Data数组
        memset(DataTypeStruct->USART_array, 0,
               sizeof(DataTypeStruct->USART_array));  // 清空USART_array数组

        // 数据帧结束，产生回调事件

        return 0;
    } else if (DataTypeStruct->c == ',') {
        DataTypeStruct->UART_NOTE_LEN[DataTypeStruct->Res_note] =
            DataTypeStruct->Res_len;  // 保存本次数据节点的长度
        DataTypeStruct->Res_note++;   // 数据节点加1
        DataTypeStruct->Res_len = 0;  // 下标清零
    } else {
        if (DataTypeStruct->Res_Data_type)  // 先检测数据格式
        {
            switch (DataTypeStruct->c) {
                // 检测数据格式
                case '1':
                    DataTypeStruct->UART_DATA_TYPE = 1;
                    break;
                case '5':
                    DataTypeStruct->UART_DATA_TYPE = 5;
                    break;
                case '6':
                    DataTypeStruct->UART_DATA_TYPE = 6;
                    break;
                case '3':
                    DataTypeStruct->UART_DATA_TYPE = 3;
                    break;
                case '4':
                    DataTypeStruct->UART_DATA_TYPE = 4;
                    break;
                case '2':
                    DataTypeStruct->UART_DATA_TYPE = 2;
                    break;
                    // 检测：命令数据格式

                case 'p':
                    DataTypeStruct->UART_DATA_TYPE = 'p';
                    break;

                default:
                    break;
            }
            // (DataTypeStruct->UART_DATA_TYPE_Callback != NULL)
            //     ? DataTypeStruct->UART_DATA_TYPE_Callback(DataTypeStruct)
            //     : 0;
            DataTypeStruct->Res_Data_type = 0;  // 关闭数据格式检测
        }

        if (DataTypeStruct->UART_Strat)  // 开始接收后，数组的第一个位置存储的是数据格式，读数据要从第二个位置开始
        {
            DataTypeStruct->USART_array[DataTypeStruct->Res_note][DataTypeStruct->Res_len] =
                DataTypeStruct->c;      // 存储数据
            DataTypeStruct->Res_len++;  // 下标加1
        }
    }
}

/// @brief 处理串口数据针对于数字
/// @param point_note 待处理的指定节点
uint16_t StdeUSART_Deal(struct Stde_DataTypeDef *DataTypeStruct, uint8_t point_note) {
    // 检查是否越界
    if (point_note > DataTypeStruct->UART_NOTE) {
        return 0;
    }

    uint16_t sum = 0;
    uint8_t len  = DataTypeStruct->UART_NOTE_LEN[point_note];

    for (uint8_t i = 0; i < len; i++) {
        uint8_t temp_target = DataTypeStruct->Data[point_note][i];
        // 判断临时目标数据的ascll码值
        if (!(temp_target >= '0' && temp_target <= '9'))  // 如果不是数字字符
        {
            return temp_target;  // 直接返回
        }

        sum = sum * 10 + (temp_target - '0');
    }

    return sum;
}

#endif

#ifdef BIE_Serial  // 专用于模拟PC交互shell脚本协议





/****************************串口模拟外接屏幕API**************************** */
GraphicsChar_Unit Graphics_Memory[10][100]; //__attribute__((section(".ram_d1_data"))) ;  // 显存区，存储字符和坐标信息

/// @brief 写入显存区中的字符和坐标信息
void Wirte_Char(uint8_t x, uint8_t y, char c, uint8_t color) {
    // 在显存区中存储字符和坐标信息
    Graphics_Memory[x][y].c           = c;
    Graphics_Memory[x][y].sit_x       = x;
    Graphics_Memory[x][y].sit_y       = y;
    Graphics_Memory[x][y].color       = color;
    Graphics_Memory[x][y].fresh       = 1;  // 设置刷新标志位
    Graphics_Memory[x][y].color_fresh = 1;  // 设置颜色刷新标志位
    Graphics_Memory[x][y].sit_fresh   = 1;  // 设置坐标刷新标志位
}

/// @brief 读取显存区中的字符和坐标信息
void Read_Char(uint8_t x, uint8_t y, char *c, uint8_t *color) {
    // 从显存区中读取字符和坐标信息
    *c     = Graphics_Memory[x][y].c;
    *color = Graphics_Memory[x][y].color;
}

/// @brief 清除显存区中的字符和坐标信息
void Clear_Char(uint8_t x, uint8_t y) {
    // 清除显存区中的字符和坐标信息
    Graphics_Memory[x][y].c           = 0;
    Graphics_Memory[x][y].sit_x       = 0;
    Graphics_Memory[x][y].sit_y       = 0;
    Graphics_Memory[x][y].color       = 0;
    Graphics_Memory[x][y].fresh       = 1;  // 清除刷新标志位
    Graphics_Memory[x][y].color_fresh = 1;  // 清除颜色刷新标志位
    Graphics_Memory[x][y].sit_fresh   = 1;  // 清除坐标刷新标志位
    // 这里可以添加代码将字符从屏幕上清除，例如调用LCD或OLED的清除函数
}

void Clear_Screen() {
    // 清除显存区中的所有字符和坐标信息
    for (uint8_t i = 0; i < 50; i++) {
        for (uint8_t j = 0; j < 50; j++) {
            Clear_Char(i, j);
        }
    }
}

void __Wirte_String(uint8_t x, uint8_t y, uint8_t color, char *str) {
    // 在显存区中存储字符串和坐标信息
    for (uint8_t i = 0; str[i] != '\0'; i++) {
        Wirte_Char(x, y + i, str[i], color);
    }
}

void Wirte_String(uint8_t x, uint8_t y, uint8_t color, char *str, ...) {
    char String[100];                     // 定义字符数组
    va_list arg;                          // 定义可变参数列表数据类型的变量arg
    va_start(arg, str);                   // 从format开始，接收参数列表到arg变量
    vsprintf(String, str, arg);           // 使用vsprintf打印格式化字符串和参数列表到字符数组中
    va_end(arg);                          // 结束变量arg
    __Wirte_String(x, y, color, String);  // 调用Wirte_String函数将字符串写入显存区
}

/**
 * @brief 显存整体上移一格函数
 * @note 该函数将显存区中的字符和坐标信息整体上移指定的行数,并将最上行的字符清除
 * @param  None
 * @retval None
 */
void Graphics_UpMove() {
    // 清除最上行的字符和坐标信息
    for (uint8_t j = 0; j < 50; j++) {
        Clear_Char(j, 120 - 1);
    }
    // 整体上移指定的行数
    for (uint8_t j = 0; j <= 50; j++) {
        for (uint8_t i = 0; i <= 120; i++) {
            Graphics_Memory[j][i].c     = Graphics_Memory[j + 1][i].c;
            Graphics_Memory[j][i].color = Graphics_Memory[j + 1][i].color;
            Graphics_Memory[j][i].fresh = 1;  // 设置刷新标志位
        }
    }
}

/**
 * @brief 交换显存区中的字符和坐标信息
 * @param  None
 * @retval None
 */
void Graphics_Swap(uint8_t Dst_x, uint8_t Dst_y, uint8_t Src_x, uint8_t Src_y) {
    // 交换显存区中的字符和坐标信息
    char temp_c                         = Graphics_Memory[Dst_x][Dst_y].c;
    uint8_t temp_color                  = Graphics_Memory[Dst_x][Dst_y].color;
    Graphics_Memory[Dst_x][Dst_y].c     = Graphics_Memory[Src_x][Src_y].c;
    Graphics_Memory[Dst_x][Dst_y].color = Graphics_Memory[Src_x][Src_y].color;
    Graphics_Memory[Src_x][Src_y].c     = temp_c;
    Graphics_Memory[Src_x][Src_y].color = temp_color;
    Graphics_Memory[Dst_x][Dst_y].fresh = 1;  // 设置刷新标志位
}

void Read_String(uint8_t x, uint8_t y, char *str, uint8_t color) {
    // 从显存区中读取字符串和坐标信息
    for (uint8_t i = 0; i < 50; i++) {
        Read_Char(x, y + i, &str[i], &color);
    }
}

void refresh_Allscreen() {
    // 刷新屏幕，将显存区中的字符和坐标信息显示到屏幕上
    for (uint8_t i = 0; i < 50; i++) {
        for (uint8_t j = 0; j < 50; j++) {
            if (Graphics_Memory[i][j].c != 0) {
                // 显示字符
                printf("%c", Graphics_Memory[i][j].c);
            }
        }
    }
}

/// @brief 部分刷新,将刷新标志位的字符显示到屏幕上
/// @param x 刷新显存的起始坐标x
/// @param y 刷新显存的起始坐标y
/// @param Mode 1: 覆盖显示 0: 不覆盖,追尾显示
void refresh_Partscreen(uint8_t x, uint8_t y, uint8_t Mode) {
    int i, j;
    for (i = x; i < 20; i++) {
        for (j = y; j < 100; j++) {
            if (Graphics_Memory[i][j].fresh == 1)  // 如果需要刷新
            {
                // 显示字符
                if (Mode == 1) {
                    // 覆盖显示
                    printf("\033[%d;%dH", Graphics_Memory[i][j].sit_x + 1,
                           Graphics_Memory[i][j].sit_y);  // 设置光标位置
                } else if (Mode == 0) {
                    // 追尾显示
                    printf("\033[%d;%dH", i, j);  // 设置光标位置
                }
                if (Graphics_Memory[i][j].color_fresh == 1)  // 如果需要刷新颜色
                {
                    // 设置颜色
                    switch (Graphics_Memory[i][j].color) {
                        case 0:
                            printf("\033[0m");
                            break;  // 默认颜色
                        case 1:
                            printf("\033[31m");
                            break;  // 红色
                        case 2:
                            printf("\033[32m");
                            break;  // 绿色
                        case 3:
                            printf("\033[33m");
                            break;  // 黄色
                        case 4:
                            printf("\033[34m");
                            break;  // 蓝色
                        case 5:
                            printf("\033[35m");
                            break;  // 紫色
                        case 6:
                            printf("\033[36m");
                            break;  // 青色
                        default:
                            printf("\033[0m");
                            break;  // 默认颜色
                    }
                }
                printf("%c", Graphics_Memory[i][j].c);  // 显示字符
                Graphics_Memory[i][j].fresh = 0;        // 清除刷新标志位
            }
        }
        printf(CURSOR_RESTORE);  // 设置光标位置到下一行
    }
    printf(CURSOR_HOME);
    printf(CURSOR_HIDE);  // 隐藏光标
    fflush(stdout);
}

/***********************应用层***************************** */
/**
    @brief 进度条函数
    @param refe 参考值
    @param curr 当前值
*/
void loading_bar(uint8_t x,uint8_t y,uint8_t color,int sta,int end,int curr) {
    // 计算进度百分比
    int percentage = (curr * 100) / (end-sta);
    int bar_length = 20;  // 进度条长度
    int filled_length = (percentage * bar_length) / 100;

    // 绘制进度条
    char bar[bar_length + 1];
    for (int i = 0; i < bar_length; i++) {
        if (i < filled_length) {
            bar[i] = '#';  // 已完成部分
        } else {
            bar[i] = '-';  // 未完成部分
        }
    }
    bar[bar_length] = '\0';  // 添加字符串结束符

    // 显示进度条
    Wirte_String(x, y, color, "[%s] %d%%  ", bar, percentage);  // 使用绿色显示
}

#endif