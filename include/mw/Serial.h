#ifndef SERIAL_TOOL
#define SERIAL_TOOL


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// 光标控制
#define CURSOR_UP(n) "\033[" #n "A"                     // 光标上移 n 行
#define CURSOR_DOWN(n) "\033[" #n "B"                   // 光标下移 n 行
#define CURSOR_RIGHT(n) "\033[" #n "C"                  // 光标右移 n 列
#define CURSOR_LEFT(n) "\033[" #n "D"                   // 光标左移 n 列
#define CURSOR_POS(row, col) "\033[" #row ";" #col "H"  // 光标移动到 (row, col)
#define CURSOR_HOME "\033[H"                            // 光标移动到左上角 (1,1)
#define CURSOR_SAVE "\033[s"                            // 保存光标位置
#define CURSOR_RESTORE "\033[u"                         // 恢复光标位置
// 清屏和清除行
#define CLEAR_SCREEN "\033[2J"      // 清屏
#define CLEAR_LINE "\033[K"         // 清除从光标到行尾
#define CLEAR_LINE_START "\033[1K"  // 清除从行首到光标
#define CLEAR_LINE_ALL "\033[2K"    // 清除整行
// 文本样式
#define RESET_ALL "\033[0m"           // 重置所有样式
#define TEXT_BOLD "\033[1m"           // 粗体
#define TEXT_DIM "\033[2m"            // 暗色（通常无效）
#define TEXT_ITALIC "\033[3m"         // 斜体
#define TEXT_UNDERLINE "\033[4m"      // 下划线
#define TEXT_BLINK "\033[5m"          // 闪烁
#define TEXT_REVERSE "\033[7m"        // 反转前景色和背景色
#define TEXT_HIDDEN "\033[8m"         // 隐藏文本（通常无效）
#define TEXT_STRIKETHROUGH "\033[9m"  // 删除线
// 颜色控制
// 前景色
#define FG_BLACK "\033[30m"    // 黑色
#define FG_RED "\033[31m"      // 红色
#define FG_GREEN "\033[32m"    // 绿色
#define FG_YELLOW "\033[33m"   // 黄色
#define FG_BLUE "\033[34m"     // 蓝色
#define FG_MAGENTA "\033[35m"  // 洋红色
#define FG_CYAN "\033[36m"     // 青色
#define FG_WHITE "\033[37m"    // 白色

// 背景色
#define BG_BLACK "\033[40m"    // 黑色背景
#define BG_RED "\033[41m"      // 红色背景
#define BG_GREEN "\033[42m"    // 绿色背景
#define BG_YELLOW "\033[43m"   // 黄色背景
#define BG_BLUE "\033[44m"     // 蓝色背景
#define BG_MAGENTA "\033[45m"  // 洋红色背景
#define BG_CYAN "\033[46m"     // 青色背景
#define BG_WHITE "\033[47m"    // 白色背景

// 256 色模式
#define FG_COLOR_256(n) "\033[38;5;" #n "m"  // 前景色，n 为 0-255
#define BG_COLOR_256(n) "\033[48;5;" #n "m"  // 背景色，n 为 0-255

// RGB 颜色模式
#define FG_COLOR_RGB(r, g, b) "\033[38;2;" #r ";" #g ";" #b "m"  // 前景色
#define BG_COLOR_RGB(r, g, b) "\033[48;2;" #r ";" #g ";" #b "m"  // 背景色
// 其他控制码
#define CURSOR_HIDE "\033[?25l"   // 隐藏光标
#define CURSOR_SHOW "\033[?25h"   // 显示光标
#define CURSOR_GET_POS "\033[6n"  // 获取光标位置
#define TERM_IDENTIFY "\033[c"    // 查询终端类型

#define Warning(str) FG_YELLOW str RESET_ALL"\n"
#define Error(str) FG_RED str RESET_ALL"\n"

#define STDE_Serial
#define BIE_Serial

#ifdef Basic_Serial

#endif

#ifdef ATOM_Serial  // 正点原子串口协议,用于兼容性交互

#endif
#ifdef STDE_Serial  // St,d,..E 串口协议,用于常规通信

typedef struct Stde_DataTypeDef{
    void (*CMD_Callback)();       // 命令模式回调函数（可选）
    void (*DR_Eevet_Callback)();  // 结束帧回调函数（可选）
    void (*UART_DATA_TYPE_Callback)(
        struct Stde_DataTypeDef *DataTypeStruct);  // 数据类型回调函数（可选
    uint8_t c;                                     // 单次数据
    uint8_t UART_Strat;  // 数据传输起始标志位
    uint8_t UART_End;    // 数据传输结束标志位
    uint8_t Res_Data_type;   // 数据格式检测标志位
    uint8_t UART_DATA_TYPE;  // 真实数据格式
    uint8_t Res_note;   // 数据节点
    uint8_t Res_len;    // 接收数据的下标
    uint8_t UART_NOTE;  // 本次数据节点
    uint8_t UART_LEN;   // 本次接收数据的长度
    uint8_t UART_NOTE_LEN[20];  // 某次数据指定节点的数据长度
    uint8_t USART_array[10][10];  // 数据长度酌情调整
    uint8_t Data[10][10];  // 数据长度酌情调整

} ;


void Stde_DataTypeDef_Init(struct Stde_DataTypeDef *data);
uint8_t STDE_UART(void* Parameters,struct Stde_DataTypeDef *DataTypeStruct);
uint16_t StdeUSART_Deal(struct Stde_DataTypeDef *DataTypeStruct, uint8_t point_note);

#ifdef BIE_Serial  // 专用于串口命令行终端交互协议




typedef struct {
    char c;               // 单次数据
    int64_t data;         // 数据
    uint8_t color;        // 字体颜色
    uint8_t sit_x;        // 字体坐标x
    uint8_t sit_y;        // 字体坐标y
    uint8_t fresh;        // 刷新标志位,标志字符是否需要刷新
    uint8_t color_fresh;  // 刷新颜色标志位,标志字符颜色是否需要刷新
    uint8_t sit_fresh;    // 刷新坐标标志位,标志字符坐标是否需要刷新
} GraphicsChar_Unit;    // 字符结构体

void Wirte_Char(uint8_t x, uint8_t y, char c, uint8_t color);
void Read_Char(uint8_t x, uint8_t y, char *c, uint8_t *color);
void Clear_Char(uint8_t x, uint8_t y);
void Clear_Screen();
void Graphics_UpMove();
void Wirte_String(uint8_t x, uint8_t y, uint8_t color, char *str, ...);
void Read_String(uint8_t x, uint8_t y, char *str, uint8_t color);
void refresh_Allscreen();
void refresh_Partscreen(uint8_t x, uint8_t y, uint8_t Mode);

void loading_bar(uint8_t x,uint8_t y,uint8_t color,int sta,int end,int curr);

#endif

#endif

#ifdef __cplusplus
}
#endif

#endif