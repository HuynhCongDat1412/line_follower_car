#ifndef ADC_H
#define ADC_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>


#define NUM_SENSORS 8
#define INT_MAX 4095
#define INT_MIN 0
// Ð?nh nghia s? lu?ng c?m bi?n
typedef struct
{
	
	ADC_HandleTypeDef *hadc;
	DMA_HandleTypeDef *hdma_adc;
	

	    uint16_t sensor_value[8];
    uint16_t sensorMin[NUM_SENSORS];
    uint16_t sensorMax[NUM_SENSORS];
	  uint16_t calibratedValues[NUM_SENSORS];
    uint16_t last_value;
	

	uint8_t  sensor;
	uint8_t on_line;
	uint8_t prepare_turn_right;
	uint8_t prepare_turn_left;
	uint8_t turn_right_filter;
	uint8_t turn_left_filter;
	uint8_t turn_right;
	uint8_t turn_left ;
	int16_t value[8];
	    uint32_t avg ;
    uint32_t sum ;
	uint8_t allSensorsDetectWhite;
	uint8_t check;
} ADC;

void adc_init(ADC *adc, ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc);

void readCalibrated(ADC *adc);
uint16_t readLine(ADC *adc, uint8_t white_line);

void check_black_white(ADC *adc);
void check_allSensorsDetectWhite (ADC *adc);
void check_on_line (ADC *adc);
void check_turn(ADC *adc);
uint8_t readline_2(ADC *adc);
bool d_check_crossline(ADC *adc);
bool d_check_rightline(ADC *adc);
bool d_check_leftline(ADC *adc);
#endif
