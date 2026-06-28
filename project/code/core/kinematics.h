/**
 * kinematics.h — 自行车模型 + 车体坐标系变换
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "car_state.h"
#include "config.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MCU 单步推进 (200Hz, 3-clamp)
// 参数说明     s           小车状态 (输入/输出)
// 参数说明     a_long      纵向加速度 [m/s²] (已clamp)
// 参数说明     omega       前轮转角角速度 [rad/s] (已clamp)
// 返回参数     void
// 使用示例     mcu_kinematics_step(&vst, a_long, omega);
// 备注信息     Euler积分 dt=CTRL_DT=5ms, 3-clamp内联
//-------------------------------------------------------------------------------------------------------------------
void mcu_kinematics_step(CarState *s, f32 a_long, f32 omega);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     世界→车体 8D变换 (3航点)
// 参数说明     out         输出 [v,δ,dx1,dy1,dx2,dy2,dx3,dy3]
// 参数说明     s           小车状态
// 参数说明     g1,g2,g3    3个航点 (世界系)
// 返回参数     void
// 使用示例     f32 inp[8]; world_to_body_8d(inp, &s, &wp[0], &wp[1], &wp[2]);
// 备注信息     与训练完全一致的变换: dx'=dx·cos+dy·sin, dy'=-dx·sin+dy·cos
//-------------------------------------------------------------------------------------------------------------------
void world_to_body_8d(f32 out[8], const CarState *s, const Waypoint *g1, const Waypoint *g2, const Waypoint *g3);

#endif
