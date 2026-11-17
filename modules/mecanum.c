
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
    { +1.0f, +1.0f, -1.0f },    /* [0] 左前 LF： */
    { +1.0f, -1.0f, +1.0f },    /* [1] 右前 RF： */
    { +1.0f, +1.0f, +1.0f },    /* [2] 右后 RB： */
    { +1.0f, -1.0f, -1.0f },    /* [3] 左后 LB： */
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

static float mec_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* ------------------------------------------------------------------ */
/* 混控主体                                                            */
/*   fwd/strafe/spin 已归一化到 [-1,1]，内部再做死区与比例缩放           */
/*   out[] 返回各轮归一化目标（未乘最大电流，由协议层决定电流档位）       */
/*   dt_s 固定按 1kHz 设计 → 每拍步进 = 速率 / 1000       
 * 根据摇杆值在xyz的值进行运算得到各轮速度归一化值，轮速由每1ms爬升
*/
/* ------------------------------------------------------------------ */
/* 自转通道死区旁路：跟随/小陀螺的自转指令是闭环算出来的，本来就很小，
 * 再被归一化死区（0.02）吃掉就等于"指令发了、车不动"。 */
static uint8_t s_spin_db_bypass = 0u;

void Mecanum_SetSpinDeadbandBypass(uint8_t on)
{
    s_spin_db_bypass = (on != 0u) ? 1u : 0u;
}

uint8_t Mecanum_Mix(float fwd, float strafe, float spin, uint8_t enabled,
                 float *out)
{ 
    uint8_t i;
    float tgt[MECANUM_WHEEL_NUM];
    float vx, vy, wz;
    float step, stop_step, dmax, k ;
    uint8_t accel_only;


    step    = MEC_ACCEL_NORM * 0.001f;      /* 每拍归一化变化量 */
    stop_step = MEC_STOP_NORM * 0.001f;     /* 松杆


     /*逆解算摇杆值得到各轮归一化的值*//*将摇杆值归一化后的值进行比例缩放，优化控制体验 */
    /* 1) 速度指令合成：摇杆比例缩放 + 归一化后残留死区清零 */
 
    if(!enabled)
{
    Mecanum_Reset();
    for(i = 0; i < MECANUM_WHEEL_NUM; i++)
    {
        out[i] = 0.0f;
    }
    return 2u;
}                

//进行速度归一化死区判断
    if (mec_abs(fwd)    < MEC_DEADBAND_NORM) fwd    = 0.0f;
    if (mec_abs(strafe) < MEC_DEADBAND_NORM) strafe = 0.0f;
    /* 自转：跟随/小陀螺下旁路，否则 3.1° 内指令全被清零 → 起转门槛 + 一顿一顿 */
    if ((s_spin_db_bypass == 0u) && (mec_abs(spin) < MEC_DEADBAND_NORM)) spin = 0.0f;

    vx = fwd    * MEC_MAX_LINEAR;
    vy = strafe * MEC_MAX_LINEAR * MEC_STRAFE_RATIO;
    wz = spin   * MEC_MAX_SPIN   * MEC_SPIN_RATIO;

      if(vx == 0.0f && vy == 0.0f && wz == 0.0f)
   {  
        for(i = 0; i < MECANUM_WHEEL_NUM; i++)
        {
            mec_prev[i] = mec_slew(mec_prev[i], 0.0f, stop_step);
            out[i] = mec_prev[i];
        }
        return 1u;
    }
    
    for(i = 0; i < MECANUM_WHEEL_NUM; i++)
    {
        const Mecanum_Dir_t *d = &MECANUM_DIR[i];
        tgt[i] = d->fwd * vx + d->strafe * vy + d->spin * wz;

         if (tgt[i] >  1.0f) tgt[i] =  1.0f;
         if (tgt[i] < -1.0f) tgt[i] = -1.0f;

    }

     dmax   = 0.0f;
     accel_only = 1u;
   

    /* 3) 逐轮解算 + 斜坡限速 */
    for (uint8_t i = 0; i < MECANUM_WHEEL_NUM; i++)
    {
 
        float d = mec_abs(tgt[i] - mec_prev[i]);
        if (d > dmax) dmax = d;
        if(tgt[i] * mec_prev[i] < 0.0f) accel_only = 0u; /*反向运动*/

    }

    step = (accel_only) ? step : stop_step;
    k    = (dmax <= step) ? 1.0f : (step / dmax);

    for(i = 0; i < MECANUM_WHEEL_NUM; i++)
    {
        mec_prev[i] += (tgt[i] - mec_prev[i]) * k;
        out[i] = mec_prev[i];
    }

     return 0u;

}
