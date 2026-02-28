/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_shell.h
**文件描述:        Shell 命令行交互模块头文件（基于 RTT）
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        基于 Zephyr Shell 子系统实现人机交互
**                 1. 通过 RTT 接收命令并输出日志
**                 2. 注册自定义命令，支持蓝牙控制、系统诊断等
**                 3. 支持历史记录、自动补全等高级功能
*********************************************************************/

#ifndef _MY_SHELL_H_
#define _MY_SHELL_H_

#define MAX_CMD_LEN     128 // 暂定单条指令长度最长128个字节,请根据实际需求修改
#define MAX_ARGS        10

#define CMD_MATCHED(data,header)  (strncmp(data,header,strlen(header))==0)
#define CMD_EQUAL(data,header)  (strcmp(data,header)==0)

typedef int (*cmdproc)(int argc, char *argv[]);
typedef struct {
    bool bSee;   // 是否可见
    char *cmd;
    char *hlp;
    cmdproc proc;
}CMD_STRUC;

/********************************************************************
**函数名称:  my_shell_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 Shell 模块，注册自定义命令集
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_shell_init(void);

#endif /* _MY_SHELL_H_ */
