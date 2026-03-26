/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       shoot.c/h
  * @brief      发射机构控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  * Version    Date            Author          Modification
  * V1.0.0   2025.10.31       Wang Zihao       1.重新构建发射机构代码结构
  * V1.1.0   2026.02.16       Gemini           2.优化单发限位逻辑，增加闭环锁止与防卡弹
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/

#include <stdlib.h>
#include "Shoot.h"
#include "Global_status.h"
#include "Gimbal.h"
#include "referee_system.h"

Shoot_t Shoot;

void Shoot_Init()
{
    TriggerMotor_init(DJI_M2006, TRIGGER_MOTOR);
    PID_Set(&Shoot.trigger_speed_pid, 8, 0, 3, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
    Shoot.single_shot_state = SINGLE_IDLE;
    Shoot.last_trigger_mode = TRIGGER_CLOSE;
}

void Shoot_Updater()
{
    Shoot.trigger_speed_now = TriggerMotor_get_data(TRIGGER_MOTOR).speed_rpm;
    Shoot.trigger_current_now = TriggerMotor_get_data(TRIGGER_MOTOR).given_current;
    Shoot.trigger_ecd_now = (int32_t)TriggerMotor_get_data(TRIGGER_MOTOR).ecd_cnt;

    if (Global.Shoot.tigger_mode == SINGLE && Shoot.last_trigger_mode != SINGLE)
    {
        Shoot.single_shot_start_ecd = Shoot.trigger_ecd_now;
        Shoot.single_shot_state = SINGLE_FIRING;
    }
    else if (Global.Shoot.tigger_mode != SINGLE && Shoot.single_shot_state == SINGLE_DONE)
    {
        Shoot.single_shot_state = SINGLE_IDLE;
    }

    Shoot.last_trigger_mode = Global.Shoot.tigger_mode;

    switch (Global.Shoot.tigger_mode)
    {
    case TRIGGER_CLOSE:
        Shoot.trigger_speed_set = TRIGGER_SPEED_CLOSE;
        break;
    case HIGH:
    case MID:
    case LOW:
    case DEBUG_TRIGGER:
        Shoot.trigger_speed_set = TRIGGER_SPEED_HIGH;
        break;
    case ONLY_AIM_HIGH:
        Shoot.trigger_speed_set = TRIGGER_SPEED_AIM_HIGH;
        break;
    case ONLY_AIM_MID:
        Shoot.trigger_speed_set = TRIGGER_SPEED_AIM_MID;
        break;
    case ONLY_AIM_LOW:
        Shoot.trigger_speed_set = TRIGGER_SPEED_AIM_LOW;
        break;
    case SINGLE:
        Shoot.trigger_speed_set = TRIGGER_SPEED_CLOSE;
        break;
    }
}

void Shoot_Calculator()
{
    if (Referee_data.Barrel_Heat < (Referee_data.Heat_Limit - 20)) // 180
	{
		if (Referee_data.Barrel_Heat < (Referee_data.Heat_Limit - 100)) // 100
			Shoot.trigger_speed_set = Shoot.trigger_speed_set / 1.2f;
		else if ((Referee_data.Barrel_Heat > (Referee_data.Heat_Limit - 100)) && (Referee_data.Barrel_Heat < (Referee_data.Heat_Limit - 90))) // 80--110
			Shoot.trigger_speed_set = Shoot.trigger_speed_set / 2.0f;
		else
			Shoot.trigger_speed_set = Shoot.trigger_speed_set / 3.0f; // 150-180
	}
	else
		Shoot.trigger_speed_set = TRIGGER_SPEED_CLOSE;
    // 1. 优先级最高：执行单发任务
    if (Shoot.single_shot_state == SINGLE_FIRING)
    {
        // 计算自触发时刻起的相对位移
        int32_t moved_ecd = Shoot.trigger_ecd_now - Shoot.single_shot_start_ecd;
        int32_t ecd_err = (int32_t)SINGLE_BULLET_ECD - moved_ecd;

        // 到位判定
        if (ecd_err <= SINGLE_SHOT_THRESHOLD)
        {
            Shoot.single_shot_state = SINGLE_DONE;
            Shoot.current[2] = 0;
            return;
        }

        // 速度规划
        float speed_set = (ecd_err <= SINGLE_SHOT_SLOWDOWN_ECD) ? 
                          (float)SINGLE_SHOT_SLOW_SPEED : (float)TRIGGER_SPEED_HIGH;

        // PID 计算
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, speed_set);

        /**
         * 【关键优化：强力启动补偿】
         * 如果在单发启动初期（转动还没超过总行程的 10%），且发现速度没起来，
         * 直接给一个足以克服摩擦力的最小保底电流（例如 1000）。
         */
        if (moved_ecd < (SINGLE_BULLET_ECD / 10)) 
        {
            if (abs((int32_t)Shoot.trigger_speed_now) < 100) 
            {
                // 给一个方向正确的额外推力
                Shoot.current[2] += (speed_set > 0) ? 1000.0f : -1000.0f;
            }
        }

        // 防卡弹逻辑
        static uint16_t block_timer = 0;
        if (abs((int32_t)Shoot.trigger_speed_now) < 100 && abs((int32_t)Shoot.trigger_current_now) > 8000)
            block_timer++;
        else
            block_timer = 0;

        if (block_timer > 300) 
        {
            Shoot.single_shot_state = SINGLE_DONE;
            Shoot.current[2] = 0;
            block_timer = 0;
        }
        return; 
    }

    // 2. 单发停机锁止
    if (Global.Shoot.tigger_mode == SINGLE)
    {
        Shoot.current[2] = 0;
        return;
    }

    // 3. 常规模式逻辑 (HIGH/MID/LOW/AIM 等)
    static int trigger_kill_cnt = 0;
    if (Shoot.trigger_current_now > 10000) trigger_kill_cnt = 50;

    if (trigger_kill_cnt > 1)
    {
        // 堵转反转脱困
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, -3000.0f);
        trigger_kill_cnt--;
    }
    else
    {
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, (float)Shoot.trigger_speed_set);
    }
}

void Shoot_Controller()
{
    SHOOTMotor_set(Shoot.current[2], TRIGGER_MOTOR);
}

void Shoot_Tasks(void)
{
#if (USE_SHOOT != 0)
    Shoot_Updater();
    Shoot_Calculator();
    Shoot_Controller();
#endif
}


