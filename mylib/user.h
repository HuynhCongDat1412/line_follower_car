#ifndef USER_H
#define USER_H
#include "motor.h"
#include "adc.h"


#define ROBOT_SPEED 60
#define         MASK2_2         0x66    /* xooxxoox   01100110                  */
#define         MASK2_0         0xc0    /* ooxxxxxx   01100000                  */
#define         MASK0_2         0x03    /* xxxxxxoo   00000110                  */
#define         MASK3_3         0xe7    /* oooxxooo   11100111                  */
#define         MASK0_3         0x07    /* xxxxxooo   00000111                  */
#define         MASK3_0         0xe0    /* oooxxxxx   11100000                  */
#define         MASK4_0         0xf0    /* ooooxxxx   11110000                  */
#define         MASK0_4         0x0f    /* xxxxoooo   00001111                  */
#define         MASK1_1         0x81    /* oxxxxxxo   01111110                  */
#define					MASK4_4					0xff    /* oooooooo	  11111111									*/

typedef enum
{
	NORMAL,
	CROSS_LINE_1,
	CROSS_LINE_2,
	TRACE_AFTER_CROSS_LINE,
	PREPARE_TURN,
	TURN_LEFT_90,
	TURN_RIGHT_90,
	END_TURN_RIGHT_90,
	END_TURN_LEFT_90
} ROBOT_STATE;
typedef struct
{

	ADC_HandleTypeDef *linesensor_hadc;
	DMA_HandleTypeDef *linesensor_hdma_adc;
	TIM_HandleTypeDef *htim_motor_pwm;
	GPIO_TypeDef *MOTOR_GPIOport;
	int16_t error;
	float adjustment;
	uint8_t left_speed;
	uint8_t right_speed;
	uint8_t standard_speed;
	uint16_t line_position;
	uint16_t last_value;
	uint16_t current_value;
	uint32_t cnt1;
	
	ROBOT_STATE state;
	uint8_t detect_turn;//0 trai, 1 phai
} ROBOT;

void robot_init(ROBOT *robot,ADC_HandleTypeDef *line_hadc, DMA_HandleTypeDef *line_hdma_adc, TIM_HandleTypeDef *htim_motor_pwm);
void robot_speed(int left_speed, int right_speed);
void control_pid_speed(ROBOT* robot, uint8_t base_speed);
void control_loop(ROBOT *robot);
void timer_handle (ROBOT *robot);
void turn_left();
void turn_right();
void check_leftright (ROBOT *robot);
void my_delay(uint32_t delay_time, ROBOT* robot);
void timer_handle(ROBOT *robot);
uint8_t sensor_mask(uint8_t sensor, uint8_t MASK);
#endif