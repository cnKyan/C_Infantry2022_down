//
// Created by HLiamso on 2021-11-14.
//


#include <CAN_receive.h>
#include "Transmission.h"


void transmit_task(void const * argument)
{
    /* USER CODE BEGIN transmit_task */
    /* Infinite loop */
    for(;;) {
        HAL_GPIO_TogglePin(GPIOH, LED_G_Pin);
        //CAN_cmd_chassis(4000, 4000, 4000, 4000);
        osDelay(500);

    }
    /* USER CODE END transmit_task */
}
