#include "tim.h"
#include "comment.h"

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0); // set the counter value a 0
    uint16_t cur_us = 0;
    while (__HAL_TIM_GET_COUNTER(&htim3) < us); // wait for the counter to reach the us input in the parameter
}
