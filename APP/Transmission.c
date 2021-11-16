//
// Created by HLiamso on 2021-11-14.
//


#include "Transmission.h"


void transmit_task(void const * argument)
{
    /* USER CODE BEGIN transmit_task */
    /* Infinite loop */
    for(;;) {
        osDelay(500);
        HAL_GPIO_TogglePin(GPIOH, LED_G_Pin);
    }
    /* USER CODE END transmit_task */
}
