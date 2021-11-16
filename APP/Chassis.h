//
// Created by HLiamso on 2021-11-14.
//

#ifndef CHASSIS_H
#define CHASSIS_H

#endif //CHASSIS_H
#include <cmsis_os.h>
#include "controller.h"
#include "motor.h"
#include "Transmission.h"
#include "bsp_can.h"
#define CHASSIS_CAN hcan1
extern rc_type_t rc;
/* 底盘控制周期 (ms) */
#define CHASSIS_PERIOD 10
/* 底盘扭腰角度 (degree)*/
#define TWIST_ANGLE    50
/* 底盘扭腰周期 (ms) */
#define TWIST_PERIOD   1500

PID_t RotateFollow;
chassis_t chassis;
Motor_t ChassisMotor[4];
/**
  * @brief     底盘控制任务函数
  */
void chassis_task(const void* argu);
/**
  * @brief     底盘速度分解，计算底盘每个轮子速度
  * @param     vx: 底盘前后速度
  * @param     vy: 底盘左右速度
  * @param     vw: 底盘旋转速度
  * @param     speed[]: 4 个轮子速度数组
  */
void chassis_moto_speed_calc(float vx, float vy, float vw, int16_t speed[]);
/**
  * @brief     获取底盘控制模式
  */
void get_chassis_mode(void);
/**
  * @brief     底盘控制参数初始化
  */
void chassis_pid_param_init(void);
/**
  * @brief     底盘自定义控制代码接口
  */
void chassis_custom_control(void);
/**
  * @brief     底盘控制信息获取
  */
void chassis_control_information_get(void);
/**
  * @brief     底盘速度闭环处理计算函数
  */
void chassis_close_loop_calculate(void);
/**
  * @brief     底盘速度开环处理计算函数
  */
void chassis_open_loop_calculate(void);
/**
  * @brief     底盘陀螺处理函数
  */
void chassis_top_handle(void);
void send_chassis_moto_zero_current(void);
extern chassis_t chassis;
extern int16_t   chassis_moto_current[];
