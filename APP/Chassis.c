//
// Created by HLiamso on 2021-11-14.
//



#include "Chassis.h"
extern osThreadId ChassisHandle;

//底盘陀螺转速系数
char spin_flag=0;
void chassis_task(void const * argument)
{
    //初始化底盘控制PID参数
    chassis_pid_param_init();

    //底盘控制任务循环
    uint32_t chassis_wake_time = osKernelSysTick();
    /* USER CODE BEGIN chassis_task */
    /* Infinite loop */
    for(;;)
    {
        //切换底盘状态
        get_chassis_mode();

        switch (chassis.mode)
        {
            //底盘跟随云台模式，右侧拨杆在上面
            case CHASSIS_FOLLOW_GIMBAL:
            {
                chassis_control_information_get();

                //底盘跟随云台旋转控制，覆盖前面计算出来的值
                if ((gim.ctrl_mode == GIMBAL_CLOSE_LOOP_ZGYRO))
//         || ((gim.ctrl_mode == GIMBAL_NO_ACTION) && (gim.no_action_flag == 1)))
                {
                    chassis.vw = PID_Calculate(&RotateFollow, yaw_relative_angle, 0);
                    chassis_top_handle();
                }
                else
                    chassis.vw = 0;
            }break;

                //底盘开环模式，右侧拨杆在中间
                //此模式适用于不装云台情况下单独控制底盘使用
            case CHASSIS_OPEN_LOOP:
            {
                chassis_control_information_get();
            }break;

            case CHASSIS_SPIN:
            {
                chassis_control_information_get();
                chassis.vw=spin_flag*10;
                chassis_top_handle();
//
//        //底盘扭腰旋转速度处理，覆盖前面计算出来的值
//        chassis_twist_handle();
            }break;

                //底盘保持静止锁死不动
            default:
            {
                chassis.vy = 0;
                chassis.vx = 0;
                chassis.vw = 0;
            }break;
        }
        if (gim.ctrl_mode==GIMBAL_AUTO)
            chassis_top_handle();
        if (chassis.mode == CHASSIS_RELAX || glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist)
        {
            send_chassis_moto_zero_current();
        }
        else
        {
            chassis_custom_control();
        }

        //底盘任务周期控制 10ms
        osDelayUntil(&chassis_wake_time, CHASSIS_PERIOD);
    }
    /* USER CODE END chassis_task */
}

void chassis_pid_param_init() {
    osThreadSuspend(ChassisHandle);
    for (int i = 0; i < 4; i++)
    {
        PID_Init(
                &ChassisMotor[i].PID_Velocity, 16000, 8000, 1, 8, 15, 0, 500, 100, 0.001, 0, 1, Integral_Limit | OutputFilter);
        ChassisMotor[i].Max_Out = 8000;
    }
    //底盘跟随PID参数设置
    PID_Init(&RotateFollow, 500, 300, 0, 5, 0, 0, 300,
             100, 0, 0.01, 5,
             Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | DerivativeFilter);
}

void get_chassis_mode() {
    chassis.last_mode = chassis.mode;
    if(rc.kb.bit.E)
        spin_flag++;
    if(rc.kb.bit.E && rc.kb.bit.SHIFT)
        spin_flag=0;
    switch (rc.sw2)
    {
        case RC_UP:
            chassis.mode = CHASSIS_FOLLOW_GIMBAL;
            break;

        case RC_MI:
            chassis.mode = CHASSIS_STOP;
            break;

        case RC_DN:
            chassis.mode = CHASSIS_STOP;
            break;
    }

    switch (rc.sw1)
    {
        case RC_UP: //user custom function
            break;

        case RC_MI: //user custom function
            break;

        case RC_DN:
        {
            if (chassis.mode == CHASSIS_FOLLOW_GIMBAL)
                chassis.mode = CHASSIS_SPIN;
        }
            break;
    }
    if(spin_flag&&(chassis.mode == CHASSIS_FOLLOW_GIMBAL||gim.ctrl_mode==GIMBAL_AUTO))
        chassis.mode=CHASSIS_SPIN;

    if (rc.sw2 == RC_DN)
        chassis.mode = CHASSIS_STOP;

//  if (chassis.mode != CHASSIS_SPIN)
//    twist_count = 0;
}

/* 底盘电机期望转速(rpm) */
int16_t chassis_moto_speed_ref[4];
/* 底盘电机电流 */
int16_t chassis_moto_current[4];

/* 底盘控制信号获取 */
void chassis_control_information_get(void)
{
    //遥控器以及鼠标对底盘的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    chassis.vx = rc.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis.vy = rc.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;

    chassis.vw = rc.ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc.mouse.x * CHASSIS_PC_MOVE_RATIO_R;
}


/* 底盘运动的速度分解，以及电机转速的闭环控制 */
void chassis_custom_control(void)
{
    //底盘速度分解，计算底盘电机转速
    chassis_moto_speed_calc(chassis.vx, chassis.vy, chassis.vw, chassis_moto_speed_ref);

    //开环计算底盘轮子电机电流
    //chassis_open_loop_calculate();

    //闭环计算底盘轮子电机电流
    chassis_close_loop_calculate();

    //将计算好的电流值发送给电调
    send_chassis_moto_current(chassis_moto_current);
}

/* 底盘的运动分解处理 */
/**
  * @param 输入参数1: vx左右平移速度(mm/s)，右为正方向
  *        输入参数2: vy前后平移速度(mm/s)，前为正方向
  *        输入参数3: vw底盘旋转速度(degree/s)，逆时针为正方向
  *        输入参数4: speed[] 4个电机转速(rpm)
  * @note  下面是电机轮子编号，左上角为0号
  * @map   1 %++++++% 0
               ++++
               ++++
           2 %++++++% 3
  */
void chassis_moto_speed_calc(float vx, float vy, float vw, int16_t speed[])
{
    //
    static float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2.0f - GIMBAL_OFFSET)/RADIAN_COEF;
    static float rotate_ratio_b = ((WHEELBASE+WHEELTRACK)/2.0f + GIMBAL_OFFSET)/RADIAN_COEF;
    static float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    float max = 0;


    //限制底盘各方向速度
    VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = (+vx - vy + vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = (+vx + vy + vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (-vx + vy + vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (-vx - vy + vw * rotate_ratio_b) * wheel_rpm_ratio;

    //限制每个轮的转速，找出最大值
    for (uint8_t i = 0; i < 4; i++)
    {
        if (abs(wheel_rpm[i]) > max)
            max = abs(wheel_rpm[i]);
    }
    //若超速，给每个轮的速度乘以相同比例

    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
            wheel_rpm[i] *= rate;
    }

#ifdef POWER_CONTORL
    //功率限制
	//按缓冲功率减小的程度给每个轮的速度乘以相同比例
	if(chassis_power > POWER_LIMIT)
	{
		float r = power_buffer * power_buffer/3600;
		for(int i=0; i<4; i++){
			wheel_rpm[i] *= r;
		}
	}
#endif

    memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}

void chassis_close_loop_calculate(void)
{
    for (int i = 0; i < 4; i++)
    {
        chassis_moto_current[i]=Motor_Speed_Calculate(&ChassisMotor[i], ChassisMotor[i].Velocity_RPM,chassis_moto_speed_ref[i]);
    }
}

void chassis_open_loop_calculate(void)
{
    for (int i = 0; i < 4; i++)
    {
        chassis_moto_current[i] = chassis_moto_speed_ref[i] * 2;
    }
}

/* 底盘陀螺处理 */
void chassis_top_handle(void)
{
    float rad=-(yaw_relative_angle/RADIAN_COEF);
    float a=cosf(rad),b=sinf(rad);
    float temp_x=a*chassis.vx+b*chassis.vy;
    float temp_y=-b*chassis.vx+a*chassis.vy;
    chassis.vx=temp_x;
    chassis.vy=temp_y;
}
void send_chassis_moto_zero_current(void)
{
    static uint8_t data[8];

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}