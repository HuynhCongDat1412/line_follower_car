#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f1xx_hal.h"

#define MAX_SPEED 100
#define MIN_SPEED 0 // cho toc do tu 0 den 100 thoi


typedef struct
{
	
	GPIO_TypeDef *DIR_GPIOPORT;
	uint16_t MOTORPIN_LEFT[2];
	uint16_t MOTORPIN_RIGHT[2];
	
	//PWM
	TIM_HandleTypeDef *htim;
	uint32_t PWM_CHANNEL_LEFT;//define nen kh biet kieu du lieu gi nen cho la uint16_t
	uint32_t PWM_CHANNEL_RIGHT;
} MOTOR;	

void motor_init (MOTOR *motor, TIM_HandleTypeDef *htim_pwm, GPIO_TypeDef *MOTOR_GPIOport, uint16_t MOTOR_pins[]);
void motor_set_speed(MOTOR *motor, int* left_speed, int* right_speed);

#endif