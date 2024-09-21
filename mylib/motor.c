#include "motor.h"
#include "user.h"
void motor_init (MOTOR *motor, TIM_HandleTypeDef *htim_pwm, GPIO_TypeDef *MOTOR_GPIOport, uint16_t MOTOR_pins[])
{
	//dong co
	motor->DIR_GPIOPORT = MOTOR_GPIOport; //mang GPIO PORT cho PWM va cac chan in cua dong co, 0 la cua dir, 1 la cua pwm
	motor->MOTORPIN_LEFT[0] = MOTOR_pins[0];// mang MOTOR_pins[] se co 6 chan theo thu tu 3 chan cho dong co trai, 3 chan cho phai
	motor->MOTORPIN_LEFT[1] = MOTOR_pins[1];
 
	motor->MOTORPIN_RIGHT[0] = MOTOR_pins[3];
	motor->MOTORPIN_RIGHT[1] = MOTOR_pins[4];
 	motor->PWM_CHANNEL_LEFT = (uint32_t) MOTOR_pins[2];
	motor->PWM_CHANNEL_RIGHT = (uint32_t) MOTOR_pins[5];
	//timer cua PWM
	motor->htim = htim_pwm;
	motor->htim->Instance->ARR = 999;//nguong de timer reset
	motor->htim->Instance->PSC = 71;// Bo chia tan so
	HAL_TIM_PWM_Start(motor->htim, motor->PWM_CHANNEL_LEFT);
	HAL_TIM_PWM_Start(motor->htim, motor->PWM_CHANNEL_RIGHT);
}

void motor_set_speed(MOTOR *motor, int *left_speed, int *right_speed)
{
	if(*left_speed>=0)
	{
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_LEFT[0],0); //o day MOTORPIN_LEFT[0] la so nguyen uint16_t
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_LEFT[1],1);
	}
		else
	{
		*left_speed = -(*left_speed);
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_LEFT[0],1); //o day MOTORPIN_LEFT[0] la so nguyen uint16_t
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_LEFT[1],0);
	}
	
	if(*right_speed>=0)
	{
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_RIGHT[0],0); //o day MOTORPIN_LEFT[0] la so nguyen uint16_t
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_RIGHT[1],1);
	}
	else
	{
		*right_speed = -*right_speed;
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_RIGHT[0],1); //o day MOTORPIN_LEFT[0] la so nguyen uint16_t
		HAL_GPIO_WritePin(motor->DIR_GPIOPORT,motor->MOTORPIN_RIGHT[1],0);
	}
	
	if (*left_speed>MAX_SPEED) *left_speed = MAX_SPEED;
	if (*left_speed<MIN_SPEED) *left_speed = MIN_SPEED;
	if (*right_speed>MAX_SPEED) *right_speed = MAX_SPEED;
	if (*right_speed<MIN_SPEED) *right_speed = MIN_SPEED;// luc nay toc do se trong khoang 0 den 100
	
	*left_speed = *left_speed*(motor->htim->Instance->ARR)/100; 
	*right_speed = *right_speed*(motor->htim->Instance->ARR)/100;
	motor->htim->Instance->CCR4 = *left_speed;
	motor->htim->Instance->CCR3 = *right_speed;
}

