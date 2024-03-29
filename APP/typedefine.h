//
// Created by HLiamso on 2021-11-17.
//

#ifndef TEST_TYPEDEFINE_H
#define TEST_TYPEDEFINE_H
typedef struct
{
    /* 遥控器的通道数据，数值范围：-660 ~ 660 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧左右
    int16_t ch4;   //左侧上下

    /* 遥控器的拨杆数据，上中下分别为：1、3、2 */
    uint8_t sw1;   //左侧拨杆
    uint8_t sw2;   //右侧拨杆

    /* PC 鼠标数据 */
    struct
    {
        /* 鼠标移动相关 */
        int16_t x;   //鼠标平移
        int16_t y;   //鼠标上下
        /* 鼠标按键相关，1为按下，0为松开 */
        uint8_t l;   //左侧按键
        uint8_t r;   //右侧按键
    }mouse;

    /* PC 键盘按键数据 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        }bit;
    }kb;

    /* 遥控器左侧拨轮数据 */
    int16_t wheel;
}rc_type_t;
//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;
/**
  * @brief     CAN 设备发送和接收 ID 枚举
  */

/**
  * @brief     解析后的遥控器数据结构体
  */

/**
  * @brief     遥控器拨杆数据枚举
  */
enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};
/**
  * @brief     云台控制信号输入状态枚举
  */
typedef enum
{
    NO_ACTION = 0,           //无控制信号输入
    IS_ACTION,               //有控制信号输入
} action_mode_e;
/**
  * @brief     云台控制模式枚举
  */
typedef enum
{
    GIMBAL_INIT = 0,         //云台初始化
    GIMBAL_RELAX = 1,            //云台断电
    GIMBAL_CLOSE_LOOP_ZGYRO = 2, //云台跟随imu z轴角度
    GIMBAL_NO_ACTION = 3,        //无手动信号输入状态
    GIMBAL_AUTO	= 4						 //云台自瞄模式
} gimbal_mode_e;
/**
  * @brief     云台控制数据结构体
  */
typedef struct
{
    gimbal_mode_e ctrl_mode; //云台当前控制模式
    gimbal_mode_e last_mode; //云台上次控制模式

    action_mode_e ac_mode;   //云台控制信号输入模式

    uint8_t  no_action_flag; //无控制信号标志
    uint32_t no_action_time; //无控制信号时间

    float ecd_offset_angle;  //云台初始编码器值
    float yaw_offset_angle;  //云台初始 yaw 轴角度
    float pit_offset_angle;  //云台初始 yaw 轴角度
} gimbal_yaw_t;
/**
  * @brief     底盘控制模式枚举
  */
typedef enum
{
    CHASSIS_STOP,          //底盘停止
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_OPEN_LOOP,     //底盘开环
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_FIXED_ROUTE,   //底盘固定路线
    CHASSIS_SPIN,          //底盘陀螺模式
} chassis_mode_e;
/**
  * @brief     底盘控制数据结构体
  */

typedef struct
{
    /* 底盘控制模式相关 */
    chassis_mode_e  mode;       //当前底盘控制模式
    chassis_mode_e  last_mode;  //上次底盘控制模式

    /* 底盘移动速度相关数据 */
    float           vx;         //底盘前后速度
    float           vy;         //底盘左右速度
    float           vw;         //底盘旋转速度


    uint8_t         last_sw2;
} chassis_t;

/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/

/**
  * @brief     底盘运动速度快慢模式
  */
typedef enum
{
    NORMAL_MODE = 0,    //正常模式
    FAST_MODE,          //快速模式
    SLOW_MODE,          //慢速模式
} kb_move_e;

/**
  * @brief     鼠标按键状态类型枚举
  */
typedef enum
{
    KEY_RELEASE = 0,    //没有按键按下
    KEY_WAIT_EFFECTIVE, //等待按键按下有效，防抖
    KEY_PRESS_ONCE,     //按键按下一次的状态
    KEY_PRESS_DOWN,     //按键已经被按下
    KEY_PRESS_LONG,     //按键长按状态
} kb_state_e;
/**
  * @brief     键盘鼠标数据结构体
  */
typedef struct
{
    /* 键盘模式使能标志 */
    uint8_t kb_enable;

    /* 鼠标键盘控制模式下的底盘移动速度目标值 */
    float vx;          //底盘前进后退目标速度
    float vy;          //底盘左右平移目标速度
    float vw;          //底盘旋转速度
    float max_spd;     //运动最大速度

    /* 左右按键状态 */
    kb_state_e lk_sta; //左侧按键状态
    kb_state_e rk_sta; //右侧按键状态

    uint16_t lk_cnt;
    uint16_t rk_cnt;

    /* 运动模式，键盘控制底盘运动快慢 */
    kb_move_e move_mode;

} km_control_t;
typedef enum
{
    //接收ID
    CAN_3508_M1_ID       = 0x201,
    CAN_3508_M2_ID       = 0x202,
    CAN_3508_M3_ID       = 0x203,
    CAN_3508_M4_ID       = 0x204,
    CAN_YAW_MOTOR_ID     = 0x205,//ID 1 001
    CAN_PIT_MOTOR_ID     = 0x206,//ID 2 010
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_SUPERCAP_RECV    = 0x211,
    //发送ID
    CAN_CHASSIS_ID       = 0x200,
    CAN_SUPER_CAP_ID      = 0X210,
    CAN_GIMBAL_ID        = 0x1ff,

} can_msg_id_e;
typedef enum
{
    DEVICE_NORMAL = 0,
    CHASSIS_M1_OFFLINE,
    CHASSIS_M2_OFFLINE,
    CHASSIS_M3_OFFLINE,
    CHASSIS_M4_OFFLINE,
    REMOTE_CTRL_OFFLINE,
    GIMBAL_YAW_OFFLINE,
    GIMBAL_PIT_OFFLINE,
    AMMO_BOOSTER1_OFFLINE,
    AMMO_BOOSTER2_OFFLINE,
    TRIGGER_MOTO_OFFLINE,
    ERROR_LIST_LENGTH,
} err_id_e;
typedef struct
{
    volatile uint32_t last_time;
    volatile uint32_t err_exist : 1;   //1 = err_exist, 0 = everything ok
    volatile uint32_t enable : 1;
    volatile uint32_t warn_pri : 6;    //priority
    volatile uint32_t delta_time : 16; //time interval last
    volatile uint32_t set_timeout : 16;
} __attribute__((__packed__)) offline_dev_t;
typedef struct
{
    volatile offline_dev_t *err_now;
    volatile offline_dev_t  err_list[ERROR_LIST_LENGTH];
    err_id_e err_id;
} __attribute__((__packed__)) glb_err_type_t;
#endif //TEST_TYPEDEFINE_H
