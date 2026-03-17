
#include "ramp_generator.h"
#include <math.h>

void RampGenerator_Init(RampGenerator *ramp,
                        unsigned int interval_ms,
                        float accel,
                        float decel,
                        float max_limit)
{
    ramp->current_value = 0.0f;
    ramp->target_value = 0.0f;
    ramp->interval_ms = (interval_ms > 0) ? interval_ms : 1;
    ramp->accel = fmaxf(accel, 0.0f);
    ramp->decel = fmaxf(decel, 0.0f);
    ramp->max_limit = fabsf(max_limit);
    ramp->last_update_time = 0;
}

void RampGenerator_SetTarget(RampGenerator *ramp, float target)
{
    ramp->target_value = target;
}

float RampGenerator_GetCurrent(const RampGenerator *ramp)
{
    return ramp->current_value;
}

void RampGenerator_Update(RampGenerator *ramp, unsigned long current_time_ms) {
    if (ramp->last_update_time == 0) {
        ramp->last_update_time = current_time_ms;
        return;
    }

    unsigned long elapsed = current_time_ms - ramp->last_update_time;
    if (elapsed < ramp->interval_ms) {
        return;
    }

    // 更新时间戳，保留余数以防时间漂移（可选，这里直接对齐到 interval）
    // 为了高性能，我们一次性计算所有变化，而不是循环
    // 但为了保持 interval_ms 的语义，我们只处理整数倍的时间
    unsigned int num_steps = elapsed / ramp->interval_ms;
    ramp->last_update_time += num_steps * ramp->interval_ms;
    
    float dt = (float)(num_steps * ramp->interval_ms) / 1000.0f;
    if (dt <= 0.0f) return;

    // 检查是否已经达到目标
    if (fabsf(ramp->current_value - ramp->target_value) < 1e-6f) {
        ramp->current_value = ramp->target_value;
        return;
    }

    // 动态适配性优化：分段处理跨零点的情况
    // 逻辑：
    // 1. 如果 current 和 target 异号（或 target 为 0），说明路径包含“归零”过程。
    //    归零过程应视为“减速”（绝对值减小），使用 decel。
    // 2. 归零后，如果还有剩余时间，从 0 向 target 移动，视为“加速”（绝对值增大），使用 accel。
    // 3. 如果同号，直接根据绝对值大小判断是加速还是减速。

    int cross_zero = (ramp->current_value > 0 && ramp->target_value < 0) || 
                     (ramp->current_value < 0 && ramp->target_value > 0);
    
    // 特殊情况：如果 target 是 0，也视为“归零”过程，但不跨越
    int to_zero = (ramp->target_value == 0.0f && ramp->current_value != 0.0f);

    if (cross_zero || to_zero) {
        // 计算归零所需时间
        // time = |current| / decel
        float time_to_zero = 0.0f;
        if (ramp->decel > 1e-6f) {
            time_to_zero = fabsf(ramp->current_value) / ramp->decel;
        } else {
            // 减速度极小，几乎无法归零，设为极大值
            time_to_zero = 1e9f; 
        }

        if (dt <= time_to_zero) {
            // 阶段 1：全程减速，未到 0
            float change = ramp->decel * dt;
            if (ramp->current_value > 0) ramp->current_value -= change;
            else ramp->current_value += change;
        } else {
            // 阶段 2：先减速到 0，再反向加速
            float remaining_dt = dt - time_to_zero;
            ramp->current_value = 0.0f; // 到达 0

            if (cross_zero) {
                // 只有跨越 0 点才需要反向加速
                float change = ramp->accel * remaining_dt;
                // 施加变化并限幅到 target
                if (ramp->target_value > 0) {
                    ramp->current_value += change;
                    if (ramp->current_value > ramp->target_value) ramp->current_value = ramp->target_value;
                } else {
                    ramp->current_value -= change;
                    if (ramp->current_value < ramp->target_value) ramp->current_value = ramp->target_value;
                }
            }
        }
    } else {
        // 同号情况（不跨零）
        // 判断是加速（远离0）还是减速（靠近0）
        float abs_current = fabsf(ramp->current_value);
        float abs_target = fabsf(ramp->target_value);
        
        // 如果 |target| > |current| -> 加速
        // 如果 |target| < |current| -> 减速
        float rate = (abs_target > abs_current) ? ramp->accel : ramp->decel;
        float change = rate * dt;

        if (ramp->current_value < ramp->target_value) {
            ramp->current_value += change;
            if (ramp->current_value > ramp->target_value) ramp->current_value = ramp->target_value;
        } else {
            ramp->current_value -= change;
            if (ramp->current_value < ramp->target_value) ramp->current_value = ramp->target_value;
        }
    }

    // 全局限幅保护
    if (ramp->current_value > ramp->max_limit) ramp->current_value = ramp->max_limit;
    else if (ramp->current_value < -ramp->max_limit) ramp->current_value = -ramp->max_limit;
}

void RampGenerator_SetInterval(RampGenerator *ramp, unsigned int interval_ms)
{
    ramp->interval_ms = (interval_ms > 0) ? interval_ms : 1;
}

void RampGenerator_SetAccel(RampGenerator *ramp, float accel)
{
    ramp->accel = fmaxf(accel, 0.0f);
}

void RampGenerator_SetDecel(RampGenerator *ramp, float decel)
{
    ramp->decel = fmaxf(decel, 0.0f);
}

void RampGenerator_SetMaxLimit(RampGenerator *ramp, float max_limit)
{
    ramp->max_limit = fabsf(max_limit);
    ramp->current_value = fminf(ramp->current_value, ramp->max_limit);
    ramp->current_value = fmaxf(ramp->current_value, -ramp->max_limit);
}

void RampGenerator_SetCurrent(RampGenerator *ramp, float current_value)
{
    ramp->current_value = current_value;
}

void RampGenerator_Reset(RampGenerator *ramp)
{
    ramp->current_value = 0.0f;
    ramp->target_value = 0.0f;
    ramp->last_update_time = 0;
}

uint8_t RampGenerator_IsFinished(const RampGenerator *ramp)
{
    return (fabsf(ramp->current_value - ramp->target_value) < 1e-6f) ? 1 : 0;
}