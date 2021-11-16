//
// Created by HLiamso on 2021-11-14.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "stm32f4xx_hal_def.h"
#endif //BSP_CAN_H
#define CAN_RC_DATA_Frame_0 0x131
#define CAN_RC_DATA_Frame_1 0x132
void CAN_Device_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
/**
  * @brief     发送 CAN 数据
  * @param     can_id: CAN 设备 ID，只有 CAN1 或者 CAN2
  * @param     send_id: 发送数据 ID
  * @param     send_data: 发送数据指针，大小为 8 位
  */
void write_can(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]);