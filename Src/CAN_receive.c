/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include <Detect.h>
#include <bsp_can.h>
#include <Chassis.h>
#include "CAN_receive.h"
#include "main.h"
uint8_t mmp[16];
rc_type_t rc;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff)
{
    for(int i=0;i<16;i++)
        mmp[i]=buff[i];
    /* 下面是正常遥控器数据的处理 */
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;(buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    /* 防止遥控器零点有偏差 */
    if(rc->ch1 <= 5 && rc->ch1 >= -5)
        rc->ch1 = 0;
    if(rc->ch2 <= 5 && rc->ch2 >= -5)
        rc->ch2 = 0;
    if(rc->ch3 <= 5 && rc->ch3 >= -5)
        rc->ch3 = 0;
    if(rc->ch4 <= 5 && rc->ch4 >= -5)
        rc->ch4 = 0;

    /* 拨杆值获取 */
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    /* 遥控器异常值处理，函数直接返回 */
    if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
    {
        memset(rc, 0, sizeof(rc_type_t));
        return ;
    }

    /* 鼠标移动速度获取 */
    rc->mouse.x = buff[6] | (buff[7] << 8);
    rc->mouse.y = buff[8] | (buff[9] << 8);

    /* 鼠标左右按键键值获取 */
    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    /* 键盘按键键值获取 */
    rc->kb.key_code = buff[14] | buff[15] << 8;

    /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
    rc->wheel = buff[16] | buff[17] << 8;
    rc->wheel -= 1024;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    static uint8_t RC_Data_Buf[16];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan==&hcan2){
        switch (rx_header.StdId){
            case CAN_RC_DATA_Frame_0:
                RC_Data_Buf[0] = rx_data[0];
                RC_Data_Buf[1] = rx_data[1];
                RC_Data_Buf[3] = rx_data[3];
                RC_Data_Buf[4] = rx_data[4];
                RC_Data_Buf[5] = rx_data[5];
                RC_Data_Buf[6] = rx_data[6];
                RC_Data_Buf[7] = rx_data[7];
                break;
                RC_Data_Buf[2] = rx_data[2];
            case CAN_RC_DATA_Frame_1:
                RC_Data_Buf[8] = rx_data[0];
                RC_Data_Buf[9] = rx_data[1];
                RC_Data_Buf[10] = rx_data[2];
                RC_Data_Buf[11] = rx_data[3];
                RC_Data_Buf[12] = rx_data[4];
                RC_Data_Buf[13] = rx_data[5];
                RC_Data_Buf[14] = rx_data[6];
                RC_Data_Buf[15] = rx_data[7];
                remote_data_handle(&rc, RC_Data_Buf);
                err_detector_hook(REMOTE_CTRL_OFFLINE);
                break;
        }
    }
    if(hcan==&hcan1){
        switch (rx_header.StdId){
            case CAN_3508_M1_ID:
            {
                ChassisMotor[0].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[0], rx_data) : \
            get_moto_info(&ChassisMotor[0], rx_data);
                err_detector_hook(CHASSIS_M1_OFFLINE);
            }
                break;
            case CAN_3508_M2_ID:
            {
                ChassisMotor[1].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[1], rx_data) : \
            get_moto_info(&ChassisMotor[1], rx_data);
                err_detector_hook(CHASSIS_M2_OFFLINE);
            }
                break;

            case CAN_3508_M3_ID:
            {
                ChassisMotor[2].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[2], rx_data) : \
            get_moto_info(&ChassisMotor[2], rx_data);
                err_detector_hook(CHASSIS_M3_OFFLINE);
            }
                break;
            case CAN_3508_M4_ID:
            {
                ChassisMotor[3].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[3], rx_data) : \
            get_moto_info(&ChassisMotor[3], rx_data);
                err_detector_hook(CHASSIS_M4_OFFLINE);
            }
                break;

            case CAN_SUPERCAP_RECV:
                //PowerDataResolve(rx_data);
            default:
            {
            }
                break;
        }
    }
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */


/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
