#ifndef USER_MOTOR_H_
#define USER_MOTOR_H_

#include <stdint.h>

#define motor_num 5
#define lead_screw 10
// 大丝杠导程,单位:mm
#define Max_tray_num 100 // 最大层数
#define Max_Length 875   // 最大行程 也就是机器人取料点

typedef struct
{
    uint8_t id; // 1,2,3,4,5
    uint8_t is_reach;
} MotorStatus;

extern MotorStatus motor_statuses[motor_num]; // 电机状态
extern uint8_t inputs[motor_num];             // 输入状态
extern uint8_t stop_state;                    // 急停状态机 0:不停机 1:停机
extern uint8_t sys_state;                     // 系统状态机
extern uint8_t home_button;                   // 归零按钮 1:按钮归零

extern int8_t tray_num;
extern int16_t tray_offset;
extern int16_t PutDown_offset;
extern int16_t heigh;

extern uint8_t updateFlag; // 一轮开始标志 1开始
extern int32_t tray_position[Max_tray_num];
extern int32_t PutDown_position[Max_tray_num];

extern int64_t fifthPosition;

void motorGoHome(uint8_t slaveAddr);                                               // 电机回零
uint8_t motor_AllGoHome(void);                                                        // 所有电机回零
void motorGoPosition(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t pos); // 指定电机运行到绝对位置
void motorReadPosition(uint8_t slaveAddr);                                         // 读取电机的绝对位置
void motor_read(void);                                                             // 读取输入
void Tray_posInit(uint32_t tray_offset, uint32_t heigh);                           // 料盘位置初始化     tray_offset:第一层位置 (单位mm)   heigh:每层高度(单位 mm)
void PutDown_posInit(uint32_t PutDown_offset, uint32_t heigh);                     // 料盘放置位置初始化 PutDown_offset:第一层位置(单位mm) heigh:每层高度(单位 mm)

void MoveFirstGMotors(uint8_t state, uint16_t speed, uint8_t acc);  // 移动第一组电机
void MoveSecondGMotors(uint8_t state, uint16_t speed, uint8_t acc,uint8_t num); // 移动第二组电机
void MoveFifthMotor(uint16_t speed, uint8_t acc, int32_t pos);      // 移动第五个电机
uint8_t GetMaterial(uint8_t num);                                   // 去取num盘料  返回非0 失败
uint8_t PutDownMaterial(uint8_t num);                               // 放第num盘料  返回非0 失败
uint8_t GetMaterialBack(uint8_t num);
uint8_t PutDownMaterialBack(uint8_t num); 

void IO_Tran(uint8_t state);                                        // 输出IO信号 料仓告知机器人料盘到位 state:1 到位 state:0 结束
uint8_t IO_Read(void);                                              // 输入IO 机器人告知料仓已近取完

void Motor_Init(void);                                                             // 初始化电机
void Position_Init(uint32_t tray_offset, uint32_t PutDown_offset, uint32_t heigh); // 取料放料位置初始化
uint8_t Run(uint8_t statrnum);                                                     // 机器人加工一盘 流程:取盘、完成、放盘 ， 返回-1 错误
uint8_t Sys_Run(uint8_t stratnum);                                                 // 跑完循环

void motor_Stop(uint16_t id); // 电机急停

void LED_Green(void);  // 绿灯
void LED_Yellow(void); // 黄灯
void LED_Red(void);    // 红灯

void motor_Sstop(uint8_t slaveAddr, uint8_t acc); // slaveAddr电机停止

#endif /* USER_MOTOR_H_ */
