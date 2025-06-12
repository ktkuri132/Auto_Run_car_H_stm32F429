//
// Created by 34575 on 25-6-8.
//
#include <stdio.h>
#include <shell.h>
#include <Serial.h>
#include <string>
#include "control/control.h"


extern Cmd_PointerTypeDef Cmd;

extern Control::Upright_Control left_Control;
extern Control::Upright_Control right_Control;
extern Control::Speed_Control Speed_Control;

extern "C" {
void _ls(int argc, void *argv[]) {
    if (argc == 2) {
        std::string cmd = static_cast<char *>(argv[0]);
        if (cmd == "pidu") {
            printf("\nleft: p:%.2f  i:%.2f d:%.2f\n", left_Control.p, left_Control.i, left_Control.d);
            printf("right: p:%.2f  i:%.2f d:%.2f\n", right_Control.p, right_Control.i, right_Control.d);
        } else if (cmd == "pids") {
            printf("\nspeed: p:%.2f  i:%.2f d:%.2f\n", Speed_Control.p, Speed_Control.i, Speed_Control.d);
        } else if (cmd == "pid") {
            printf("\nspeed: p:%.2f  i:%.2f d:%.2f\n", Speed_Control.p, Speed_Control.i, Speed_Control.d);
            printf("sleft: p:%.2f  i:%.2f d:%.2f\n", left_Control.p, left_Control.i, left_Control.d);
            printf("sright: p:%.2f  i:%.2f d:%.2f\n", right_Control.p, right_Control.i, right_Control.d);
        } else if (cmd == "die") {
            printf("left: die:%.2f\n", left_Control.min_output);
            printf("right: die:%.2f\n", right_Control.min_output);
        } else {
            printf(Error("arguments error:ls command need pid,die,test,reset"));
        }
    } else {
        printf(Error("arguments error:ls command need 2 arguments"));
    }
}


void _pids(int argc, void *argv[]) {
    if (argc == 4) {
        std::string lr = static_cast<char *>(argv[0]);
        std::string pid = static_cast<char *>(argv[1]);
        std::string data = static_cast<char *>(argv[2]);
        float arg_value_f = std::stof(data); // 将字符串转换为浮点数
        if (lr == "l") {
            if (pid == "p") {
                Speed_Control.p = arg_value_f;
            } else if (pid == "i") {
                Speed_Control.i = arg_value_f;
            } else if (pid == "d") {
                Speed_Control.d = arg_value_f;
            } else {
                printf(Error("arguments error:pid command need p,i,d"));
            }
        } else if (lr == "r") {
            if (pid == "p") {
                Speed_Control.p = arg_value_f;
            } else if (pid == "i") {
                Speed_Control.i = arg_value_f;
            } else if (pid == "d") {
                Speed_Control.d = arg_value_f;
            } else {
                printf(Error("arguments error:pid command need p,i,d"));
            }
        } else if (lr == "all") {
            if (pid == "p") {
                Speed_Control.p = arg_value_f;
            } else if (pid == "i") {
                Speed_Control.i = arg_value_f;
            } else if (pid == "d") {
                Speed_Control.d = arg_value_f;
            } else {
                printf(Error("arguments error:pid command need p,i,d"));
            }
        } else {
            printf(Error("arguments error:pid command need l,r,all"));

        }
        printf("\nleft: p:%.2f  i:%.2f d:%.2f\n", Speed_Control.p, Speed_Control.i, Speed_Control.d);
    } else {
        printf(Error("arguments error:pid command need 4 arguments"));
    }
}

void _pidu(int argc, void *argv[]) {
    if (argc == 4) {
        std::string lr = static_cast<char *>(argv[0]);
        std::string pid = static_cast<char *>(argv[1]);
        std::string data = static_cast<char *>(argv[2]);
        float arg_value_f = std::stof(data); // 将字符串转换为浮点数
        if (lr == "l") {
            if (pid == "p") {
                left_Control.p = arg_value_f;
            } else if (pid == "i") {
                left_Control.i = arg_value_f;
            } else if (pid == "d") {
                left_Control.d = arg_value_f;
            } else {
                printf(Error("arguments error:pid command need p,i,d"));
            }
        } else if (lr == "r") {
            if (pid == "p") {
                right_Control.p = arg_value_f;
            } else if (pid == "i") {
                right_Control.i = arg_value_f;
            } else if (pid == "d") {
                right_Control.d = arg_value_f;
            } else {
                printf(Error("arguments error:pid command need p,i,d"));
            }
        } else if (lr == "all") {
            if (pid == "p") {
                left_Control.p = arg_value_f;
                right_Control.p = arg_value_f;
            } else if (pid == "i") {
                left_Control.i = arg_value_f;
                right_Control.i = arg_value_f;
            } else if (pid == "d") {
                left_Control.d = arg_value_f;
                right_Control.d = arg_value_f;
            } else {
                printf(Error("arguments error:pid command need p,i,d"));
            }
        } else {
            printf(Error("arguments error:pid command need l,r,all"));

        }
        printf("left: p:%.2f  i:%.2f d:%.2f\n", left_Control.p, left_Control.i, left_Control.d);
        printf("right: p:%.2f  i:%.2f d:%.2f\n", right_Control.p, right_Control.i, right_Control.d);
    } else {
        printf(Error("arguments error:pid command need 4 arguments"));
    }
}

void _die(int argc, void *argv[]) {
    if (argc == 3) {
        std::string lr = static_cast<char *>(argv[0]);
        std::string data = static_cast<char *>(argv[1]);
        float arg_value_f = std::stof(data); // 将字符串转换为浮点数
        if (lr == "l") {
            left_Control.min_output = arg_value_f;
        } else if (lr == "r") {
            right_Control.min_output = arg_value_f;
        } else if (lr == "all") {
            left_Control.min_output = arg_value_f;
            right_Control.min_output = arg_value_f;
        } else {
            printf(Error("arguments error:die command need l,r,all"));
        }
        printf("die left: %.2f\n", left_Control.min_output);
        printf("die right: %.2f\n", right_Control.min_output);
    }
}

void _test(int argc, void *argv[]) {
    printf("cmd:%s argc:%d argv1:%s argv2:%s argv3:%s\n", "test", argc, argv[0], argv[1], argv[2]);
}

void _reset(int argc, void *argv[]) {
    NVIC_SystemReset(); // 重启系统
}


EnvVar MyEnvVar[20] = {
    {
        .name = "pidu",
        .callback = _pidu,
    },
    {
        .name = "pids",
        .callback = _pids,
    },
    {
        .name = "die",
        .callback = _die,
    },
    {
        .name = NULL,
        .callback = NULL
    }
};
}


void Sys_Cmd_Init() {
    // 初始化系统命令
    Cmd.ls = _ls;
    Cmd.reset = _reset;
    Cmd.test = _test;
}
