
#include "mecanum.h"
/*底盘混控 ：归一化目标输入   ，得到各轮归一化指令 */
#if (MECANUM_WHEEL_NUM == 4)
//麦轮方向结构体，设定各轮在各运动状态的旋转方向

/*
 *   列序列 ：fwd  strafe  spin
 *    LF = vx - vy - wz
 *   RF = vx + vy + wz
 *   RB = vx - vy + wz
 *   LB = vx + vy - wz
 *
 *   
 */
const Mecanum_Dir_t MECANUM_DIR[MECANUM_WHEEL_NUM] =
{
    { +1.0f, -1.0f, -1.0f },    /* [0] 左前 LF： */
    { +1.0f, +1.0f, +1.0f },    /* [1] 右前 RF： */
    { +1.0f, -1.0f, +1.0f },    /* [2] 右后 RB： */
    { +1.0f, +1.0f, -1.0f },    /* [3] 左后 LB： */
};
#endif

/* 斜坡内部状态：上一拍实际输出的归一化指令 */
static float mec_prev[MECANUM_WHEEL_NUM] = {0};

/* ------------------------------------------------------------------ */
/* 单轮斜坡：把 cur 朝 tgt 移动，每拍最多走 max_step                     */
/* ------------------------------------------------------------------ */
static float mec_slew(float cur, float tgt, float max_step)
{
    float d = tgt - cur;

    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;

    return cur + d;
}

/* ------------------------------------------------------------------ */
void Mecanum_Reset(void)
{
    for (uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
    {
        mec_prev[i] = 0.0f;
    }
}

/* ------------------------------------------------------------------ */
/* 混控主体                                                            */
/*   fwd/strafe/spin 已归一化到 [-1,1]，内部再做死区与比例缩放           */
/*   out[] 返回各轮归一化目标（未乘最大电流，由协议层决定电流档位）       */
/*   dt_s 固定按 1kHz 设计 → 每拍步进 = 速率 / 1000       
 * 根据摇杆值在xyz的值进行运算得到各轮速度归一化值，轮速由每1ms爬升
*/
/* ------------------------------------------------------------------ */
void Mecanum_Mix(float fwd, float strafe, float spin, uint8_t enabled,
                 float *out)
{
    float vx, vy, wz;
    float step_up   = MEC_ACCEL_NORM / 1000.0f;     /* 1kHz 控制周期 */
    float step_down = MEC_STOP_NORM  / 1000.0f;
    float step;
     /*逆解算摇杆值得到各轮速度*//*将摇杆值归一化后的值进行比例缩放，优化控制体验 */
    /* 1) 速度指令合成：摇杆比例缩放 + 归一化后残留死区清零 */
    vx = fwd    * MEC_MAX_LINEAR;
    vy = strafe * MEC_MAX_LINEAR * MEC_STRAFE_RATIO;
    wz = spin   * MEC_MAX_SPIN   * MEC_SPIN_RATIO;
//进行速度归一化死区判断
    if ((vx > -MEC_DEADBAND_NORM) && (vx < MEC_DEADBAND_NORM)) vx = 0.0f;
    if ((vy > -MEC_DEADBAND_NORM) && (vy < MEC_DEADBAND_NORM)) vy = 0.0f;
    if ((wz > -MEC_DEADBAND_NORM) && (wz < MEC_DEADBAND_NORM)) wz = 0.0f;

    /* 2) 未使能（失联/拨杆没到位）：目标强制 0，且用更快的回零速度 */
    if (!enabled)
    {
        vx = 0.0f;
        vy = 0.0f;
        wz = 0.0f;
        step = step_down;
    }
    else
    {
        step = step_up;
    }

    /* 3) 逐轮解算 + 斜坡限速 */
    for (uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
    {//得到各轮各方向符号进而解算各轮获得的速度
        const Mecanum_Dir_t *d = &MECANUM_DIR[i];
        float tgt = d->fwd * vx + d->strafe * vy + d->spin * wz;

        /* 限幅到 ±1：斜推摇杆时不会越界 */
        if (tgt >  1.0f) tgt =  1.0f;
        if (tgt < -1.0f) tgt = -1.0f;

        /* 松杆回零比加速更快，保证“松手就停”的手感 *//*进行每次归一化值变化速度的设定*/
        float s = (tgt == 0.0f || (mec_prev[i] * tgt < 0.0f)) ? step_down : step;

        //将本次变化完的归一化值传到out输出
        mec_prev[i] = mec_slew(mec_prev[i], tgt, s);
        out[i] = mec_prev[i];
    }
}
