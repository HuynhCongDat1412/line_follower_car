#include "user.h"
#include "pid.h"
extern uint16_t white_detect; 
extern uint16_t black_detect; 
uint16_t LEFT = 2000;
uint16_t RIGHT = 6000;
//khai bao dong co (2 chan direction dong co 1, pwm1, 2 chan direction dong co 2, pwm 2
uint16_t MOTOR_pins[] = {GPIO_PIN_8,GPIO_PIN_9,TIM_CHANNEL_3,GPIO_PIN_10,GPIO_PIN_12,TIM_CHANNEL_4};
GPIO_TypeDef *MOTOR_GPIOport = GPIOA;
	
ADC ROBOT_linesensor;
MOTOR ROBOT_motor;

//PID_CONTROL_t pid_ctrl;
uint8_t sensor_mask(uint8_t sensor, uint8_t MASK){// HAM MAT NA CHO CAM BIEN		
	return ( sensor & MASK);
}

void timer_handle(ROBOT *robot)
{
	robot->cnt1++;
}

void robot_init(ROBOT *robot,ADC_HandleTypeDef *line_hadc, DMA_HandleTypeDef *line_hdma_adc, TIM_HandleTypeDef *htim_motor_pwm)
{
	// khoi tao line sensor
	robot->linesensor_hadc = line_hadc;
	robot->linesensor_hdma_adc = line_hdma_adc;

	
	// khoi tao dong co
robot->htim_motor_pwm = htim_motor_pwm;
	robot->MOTOR_GPIOport = MOTOR_GPIOport;
	adc_init(&ROBOT_linesensor,robot->linesensor_hadc,robot->linesensor_hdma_adc);
	motor_init(&ROBOT_motor, robot->htim_motor_pwm, robot->MOTOR_GPIOport,MOTOR_pins);
	//pid_init(&pid_ctrl, 1.0, 0.1, 0.05, MAX_SPEED, MIN_SPEED, 0.01);  // Kp, Ki, Kd, Max, Min, dt
	robot->standard_speed = ROBOT_SPEED;
	ROBOT_linesensor.on_line = 0;
	robot->cnt1=0;
	
	robot->state = NORMAL;
}


void robot_speed(int left_speed, int right_speed)
{
	motor_set_speed(&ROBOT_motor,&left_speed,&right_speed);
}

// Gi? s? b?n dã kh?i t?o PID



// Gi? s? b?n mu?n di?u khi?n t?c d? d?a trên giá tr? t? c?m bi?n
void control_loop(ROBOT *robot) {
   robot->current_value = readLine(&ROBOT_linesensor,1);  // Giá tr? h?i ti?p t? c?m bi?n line

		
    // Tính toán giá tr? di?u khi?n t? PID
    //float pid_output = pid_compute(&pid_ctrl, setpoint, (float)last_value);

    // Ği?u ch?nh t?c d? d?ng co d?a trên giá tr? PID
    //int left_speed = BASE_SPEED + (int)pid_output;
    //int right_speed = BASE_SPEED - (int)pid_output;
	
		
		
		switch (robot->state){
			case NORMAL: 
				{
					
					check_allSensorsDetectWhite (&ROBOT_linesensor);
					if (ROBOT_linesensor.allSensorsDetectWhite ==1) 
					{robot->state = CROSS_LINE_1;}
					// Ği?u khi?n d?ng co v?i t?c d? dã tính toán
					control_pid_speed(robot, robot->standard_speed);
					break;
				}
			case CROSS_LINE_1:
				{
					robot->cnt1=0;
					ROBOT_linesensor.allSensorsDetectWhite = 0;
					robot->state = CROSS_LINE_2;
				
					break;
				}
				case CROSS_LINE_2:
				{
					control_pid_speed(robot,0.6*robot->standard_speed);
					if (robot -> cnt1 >= 100)
				{
					if(d_check_crossline(&ROBOT_linesensor) == 0)
					{
						if(d_check_leftline(&ROBOT_linesensor)== 0 && d_check_rightline(&ROBOT_linesensor) ==0){
							ROBOT_linesensor.turn_right_filter=0;
							ROBOT_linesensor.turn_left_filter=0;
						robot -> state = TRACE_AFTER_CROSS_LINE;
						robot -> cnt1 = 0;}
						break;
					}
					
				}
				break;
			}
				case TRACE_AFTER_CROSS_LINE:
				{
					control_pid_speed(robot, 0.6*robot->standard_speed);
					check_turn(&ROBOT_linesensor);
							if(sensor_mask(readline_2(&ROBOT_linesensor), 0xff)    == 0x00){
								robot->state = PREPARE_TURN;
								}						
							/*if(robot->cnt1>100 && ROBOT_linesensor.on_line==1) {
								ROBOT_linesensor.prepare_turn_left=0;
								ROBOT_linesensor.prepare_turn_right=0;
								robot->cnt1 = 0;*/
												
							
							
							break;
				}
				case PREPARE_TURN:
				{
						
						if(ROBOT_linesensor.turn_right_filter > ROBOT_linesensor.turn_left_filter)
						{
							robot_speed(100,0);
							robot->state = TURN_RIGHT_90;
						}
						else if(ROBOT_linesensor.turn_left_filter > ROBOT_linesensor.turn_right_filter)
						{
							//robot_speed(0,100);
							robot->state = TURN_LEFT_90;
						}
					robot->cnt1 = 0;
					break;
					
				}
			
			case TURN_RIGHT_90:
				{   //
					robot_speed(100,0);
					if (robot -> cnt1  > 150)
					{
						//HAL_Delay(100);
						robot -> state = END_TURN_RIGHT_90;
						robot -> cnt1 = 0;
					}
					break;
					}
			case END_TURN_RIGHT_90:
				{   // K?t thúc r? ph?i
						
						if(ROBOT_linesensor.on_line == 1 && d_check_crossline(&ROBOT_linesensor) == 0) //0011 1100
					{	
						robot -> state  = NORMAL;
						break;
					}
					break;
				}
		}
}
void control_pid_speed(ROBOT* robot, uint8_t base_speed)
{
		const double KP = 0.028;
		const double KD = 0.0;
		double lastError = 0;
		const uint16_t GOAL = 3500;  // Giá tr? mong mu?n là gi?a du?ng

		// Tính toán l?i
		robot->error = GOAL - robot->current_value;
		robot->adjustment = KP * robot->error + KD * (robot->error - lastError);

		// Gi?i h?n giá tr? di?u ch?nh
		if (robot->adjustment > ROBOT_SPEED) robot->adjustment = ROBOT_SPEED;
		if (robot->adjustment < -ROBOT_SPEED) robot->adjustment = -ROBOT_SPEED;

		// Tính toán t?c d? d?ng co d?a trên giá tr? di?u ch?nh t? PID
		robot->left_speed = (int16_t)(base_speed + robot->adjustment);
		robot->right_speed = (int16_t)(base_speed - robot->adjustment);

		// Ği?u khi?n d?ng co
	robot_speed(robot->left_speed, robot->right_speed);
}



/*
			case PREPARE_TURN:
			{
				 if (detectLeftTurn()) {
                turnLeft();  // Th?c hi?n qu?o trái
                robot->state = NORMAL;  // Quay l?i tr?ng thái NORMAL sau khi qu?o
            } else if (detectRightTurn()) {
                turnRight();  // Th?c hi?n qu?o ph?i
                robot->state = NORMAL;  // Quay l?i tr?ng thái NORMAL sau khi qu?o
            }
            break;
			}
//

         // Hi?n th? màu s?c d? báo hi?u quá trình hi?u ch?nh hoàn t?t
    }
	}		
		
		void check_turn_leftright (ROBOT *robot)
		{
			if (robot->)
		}
		*/
    

