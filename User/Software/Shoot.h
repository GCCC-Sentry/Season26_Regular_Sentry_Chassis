/*
 * @Date: 2025-10-27 21:03:17
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-13 04:31:18
 * @FilePath: \Regular_Sentry_Chassis\User\Software\Shoot.h
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "CAN_receive_send.h"
#include "pid.h"

#include "motor.h"
/*发射机构参数*/
enum shoot_speed_e // 摩擦轮速度
{
    SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,// 开始反转值
    SHOOT_SPEED_CLOSE = 0,    // 停止速度值
    SHOOT_SPEED_READY = FRIC_SPEED_REDAY,  // 正常工作值
    SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG, // 退弹低速值
};

enum trigger_speed_e // 拨弹电机速度
{
    TRIGGER_SPEED_CLOSE = 0,
    TRIGGER_SPEED_HIGH  = TRIGGER_SPEED_H,
    TRIGGER_SPEED_MID   = TRIGGER_SPEED_M,
    TRIGGER_SPEED_LOW   = TRIGGER_SPEED_L,
    TRIGGER_SPEED_DEBUG = 3000,
    TRIGGER_SPEED_AIM_HIGH = 4800,
    TRIGGER_SPEED_AIM_MID = 2400,
    TRIGGER_SPEED_AIM_LOW = 1200,
};

/* 单发限位参数 */
#define TRIGGER_GEAR_RATIO       36        // M2006减速比
#define TRIGGER_HOLE_NUM         9         // 拨弹盘孔位数
#define SINGLE_BULLET_ECD        ((TRIGGER_GEAR_RATIO * 8192) / TRIGGER_HOLE_NUM)
#define SINGLE_SHOT_THRESHOLD    500       // 到位判定阈值
#define SINGLE_SHOT_SLOWDOWN_ECD 3000      // 接近目标时降速阈值
#define SINGLE_SHOT_SLOW_SPEED   1200      // 单发末段低速拨弹

/* 单发状态机 */
typedef enum {
    SINGLE_IDLE,
    SINGLE_FIRING,
    SINGLE_WAITING, // 新增：出弹后的低速维持状态
    SINGLE_DONE,
} single_shot_state_e;

// 定义低速待机的转速（建议设为一个很小但足以维持转动的数值）
#define SINGLE_WAIT_SPEED  500

/* 内部调用 */
#define SHOOTMotor_init(type, id)    DJIMotor_Init(type, id)
#define SHOOTMotor_set(val, id)      DJIMotor_Set(val, id)
#define TriggerMotor_init(type, id)  DJIMotor_Init(type, id)
#define TriggerMotor_set(val, id)    DJIMotor_Set(val, id)
#define TriggerMotor_get_data(id)    DJIMotor_GetData(id)
#define SHOOTMotor_get_data(id)      DJIMotor_GetData(id)

/* 电机参数 */
#define SHOOTMOTOR_MAX_CURRENT MAX_CURRENT

/*内部数据类型*/
typedef struct
{
    /*-------PID-------*/
    pid_t shoot_L_speed_pid;
    pid_t shoot_R_speed_pid;
    pid_t trigger_speed_pid;

    /*-------状态量-------*/
    float shoot_speed_L_now;
    float shoot_speed_R_now;
    float trigger_speed_now;
    float trigger_current_now;

    /*-------目标量-------*/
    enum trigger_speed_e trigger_speed_set;
    enum shoot_speed_e shoot_speed_set;
    float current[3];

    /*-------单发限位-------*/
    single_shot_state_e single_shot_state;
    int32_t trigger_ecd_now;
    int32_t single_shot_start_ecd;
    uint8_t last_trigger_mode;

}Shoot_t;

/* 外部调用 */
void Shoot_Init();
void Shoot_Tasks();
void Receive_from_Gimbal_6(uint8_t data[8]);
#endif 
