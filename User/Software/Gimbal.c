/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-24 04:19:11
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Gimbal.c
 */
/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       gimbal.c/h
  * @brief      云台控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建云台代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/
#include "Gimbal.h"
#include "Chassis.h"
#include "Global_status.h"

#include "User_math.h"
#include "remote_control.h"
#include "IMU_updata.h"
#include "dm_imu.h"
#include "motor.h"
#include "Auto_control.h"
Gimbal_t Gimbal;



/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    // 云台电机初始化
    DMMotor_Init(DM_4310, BIGYAWMotor);


    /*PID速度环初始化*/ 
    // 遥控
    PID_Set(&Gimbal.big_yaw.big_yaw_speed_pid, 200.0f, 0.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    //自瞄
    PID_Set(&Gimbal.big_yaw.big_yaw_auto_speed_pid, 200.0f, 0.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.big_yaw.big_yaw_location_pid, 2.0f, 0.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 100);
    //自瞄
    PID_Set(&Gimbal.big_yaw.big_yaw_auto_location_pid, 2.0f, 0.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 100);
    // 云台零点初始化
    DMMotor_SetZero(BIG_YAW_ZERO, BIGYAWMotor); 
    }



/*-------------------- Update --------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */
void Gimbal_Updater()
{
    static int last_chassis_mode = FLOW_Chassis;
    static float yaw_offset = 0.0f;
    static float last_Global_Control_mode = 0.0f;
    uint8_t navigation_use_location_ctrl = (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 1);

    /*------状态量更新------*/
    //速度
    Gimbal.big_yaw.big_yaw_speed_now = (cos(imu.pitch / RAD_TO_DEG) * RAD_TO_DEG * imu.gyro[2] - sin(imu.pitch / RAD_TO_DEG) * RAD_TO_DEG * imu.gyro[0]);
    //位置
    Gimbal.big_yaw.big_yaw_location_now = imu.yaw_cnt; 

    /*------模式切换处理------*/
    if ((last_chassis_mode == SPIN_P || last_chassis_mode == SPIN_N) &&
    (Global.Chassis.mode != SPIN_P && Global.Chassis.mode != SPIN_N)) 
    {
    // 记录当前偏移，避免跳变
    // yaw_offset = Gimbal.big_yaw.big_yaw_location_now - Global.Gimbal.input.yaw;

    // 重置云台目标位置为当前位置
    Global.Gimbal.input.yaw = Gimbal.big_yaw.big_yaw_location_now;
    }
    // 从 Global.Control.mode==2（泄力）退出时，记录位置偏移量，防止大yaw回转
    if (last_Global_Control_mode == 2 && Global.Control.mode != 2)
    {
        yaw_offset = Gimbal.big_yaw.big_yaw_location_now - Global.Gimbal.input.yaw;
    }
    last_Global_Control_mode = Global.Control.mode;

    // 从 Navigation 退出到其他模式时，记录位置偏移量，防止回转
    // 此时 Global.Gimbal.input.yaw 是 CAN 原始值（未被 Navigation 覆写），offset 才正确
    if (last_chassis_mode == Navigation && Global.Chassis.mode != Navigation)
    {
        yaw_offset = Gimbal.big_yaw.big_yaw_location_now - Global.Gimbal.input.yaw;
    }
    // 进入 Navigation 时清除偏移量（Navigation 使用速度控制，不需要偏移）
    if (last_chassis_mode != Navigation && Global.Chassis.mode == Navigation)
    {
        yaw_offset = 0.0f;
    }
    last_chassis_mode = Global.Chassis.mode;

    /*------目标量更新------*/
    // flag_lock==2 时泄力状态，目标位置同步到当前位置，防止PID积分饱和
    if (Global.Control.mode == 2)
    {
        Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.big_yaw_location_now;
    }
    else if (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 0)
    {
        // ===== 自瞄锁定：上板发的是编码器坐标系目标，直接转成IMU世界角 =====
        // Global.Gimbal.input.yaw 现在 = relative_angle + encoder_relative（编码器系）
        // 需要转换到IMU世界角：big_yaw_world = big_yaw_imu_now + (target_encoder - current_encoder)
        float current_encoder = Chassis.relative_angle;  // 大yaw当前编码器位置
        float target_encoder  = Global.Gimbal.input.yaw; // 上板发来的编码器坐标系目标
        float delta_encoder   = target_encoder - current_encoder; // 还需要转过的角度

        Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.big_yaw_location_now + delta_encoder;
    }
    else switch(Global.Chassis.mode)
    {
        case Navigation :
        /* case FLOW_Chassis :  */
            Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.big_yaw_location_now;
            break;
        default :
            // 位置控制，叠加 offset 防止退出 Navigation 时回转
            Gimbal.big_yaw.big_yaw_location_set = Global.Gimbal.input.yaw + yaw_offset;
            break;
    }

}


/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
void Gimbal_Calculator()
{
    uint8_t nav_auto_lock = (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 0);
    if (nav_auto_lock)
    {
        // 导航 + 自瞄锁定：大yaw位置环追目标（目标在Updater中已设好）
        Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set);
    }
    else if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        // 非自瞄（含导航扫描模式 is_scaning==1）
        switch(Global.Chassis.mode)
        {
            case Navigation :
                // is_scaning==1：速度控制（扫描匀速旋转由上板导航速度控制）
            /* case FLOW_Chassis : */ 
                Gimbal.big_yaw.big_yaw_speed_set = 180.0f;
                if(Chassis.is_aligning == 1)
                {
                    Gimbal.big_yaw.big_yaw_speed_set = Gimbal.big_yaw.planning_speed;
                }
            break;
            default :
                Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        }
        Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set); 
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    { 
        // 自瞄
        Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set);
        Global.Auto.input.Auto_control_online--;
    }
}


/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none 
 * @retval         none
 */
void Gimbal_Controller()
{
    if(Global.Control.mode != LOCK)
    {
        DMMotor_Set(BIGYAWMotor,
                 0,
                 0,
                 Gimbal.big_yaw.current / 10000,
                 0,
                 0);
    }
    else
    {
        DMMotor_Set(BIGYAWMotor,
                 0,
                 0,
                 0,
                 0,
                 0);
    }
       
}


/*-------------------- Task --------------------*/

/**
 * @brief          云台任务
 * @param          none
 * @retval         none
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)
	// 云台数据更新
	Gimbal_Updater();
    // 云台控制解算
    Gimbal_Calculator();
	// 云台电机控制
	Gimbal_Controller();
#endif
}


/*-------------------- Set --------------------*/
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_SetPitchAngle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**          
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台eYAW轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

