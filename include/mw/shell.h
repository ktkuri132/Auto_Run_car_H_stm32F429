#ifndef __SEHLL_H
#define __SEHLL_H
#include <stdint.h>

// 主版本号
#define SHELL_VERSION_MAIN 			1
// 确认发行版号
#define SHELL_VERSION_RE			0		
// 更新号
#define SHELL_VERSION_UPDATE		1

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 环境变量结构体
 */
typedef struct Env_Function_Point {
    void (*envpfunc)(int argc, void **argv);  // 环境函数指针
    int argc;
    void **Parameters;  // 参数列表
}EFP;


typedef struct{
    char *Architecture;  // 处理器架构
    char *User;         // 用户名
    char *Password;      // 密码
    char *DeviceName;   // 设备名称
    char *OS;           // 操作系统
    char *Device;        // 设备型号
    char *Version;       // 版本信息
}DeviceFamily;

typedef struct {
    uint8_t c;
    uint8_t Res_len;    // 接收数据的下标
    uint8_t UART_NOTE;  // 本次数据节点
    uint8_t
    RunStae;  // 运行状态   --> 这里出现了一个bug,下面的Data数组访问越界,导致RunStae的值被覆盖了
    uint8_t Data[20];                            // 数据长度酌情调整
} Bie_ShellTypeDef __attribute__((aligned(4)));  // Shell协议结构体

typedef struct {
    char *name;                                // 命令名称
    uint8_t RunStae;                           // 运行状态
    void **arg;                                // 命令参数指针
    int argc;                                  // 参数个数
    void (*callback)(int argc, void *argv[]);  // 命令回调函数
} EnvVar;                                      // 环境变量结构体

void MCU_Shell_Init(Bie_ShellTypeDef *ShellTypeStruct,DeviceFamily *log);
void Shell_Deal(Bie_ShellTypeDef *ShellTypeStruct, EnvVar *env_vars);
void BIE_UART(void *Parameters, Bie_ShellTypeDef *ShellTypeStruct, EnvVar *env,DeviceFamily *log);

typedef struct {
    void (*ls)(int argc, void *argv[]);        // ls命令回调函数
    void (*reset)(int argc, void *argv[]);     // reboot命令回调函数
    void (*poweroff)(int argc, void *argv[]);  // poweroff命令回调函数
    void (*help)(int argc, void *argv[]);      // help命令回调函数
    void (*clear)(int argc, void *argv[]);     // clear命令回调函数
    void (*test)(int argc, void *argv[]);      // test命令回调函数
} Cmd_PointerTypeDef;                          // 系统默认配置命令指针结构体

#ifdef __cplusplus
    }
#endif


#endif