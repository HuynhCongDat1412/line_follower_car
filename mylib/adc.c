#include "adc.h"
#include "user.h"
#define ADC_PAGE_ADDRESS 0x0801F800  // Page 126
uint16_t white_detect= 300;
uint16_t black_detect =600;

void adc_init(ADC *adc, ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc)
{
	
	adc -> hadc            = hadc ;
	adc -> hdma_adc  			 = hdma_adc ;
	adc->last_value = 0;
	HAL_ADC_Start_DMA(hadc,  (uint32_t *) adc -> sensor_value, 8);

//	uint16_t weight[8] = {500,500,500,500,500,500,500,500};
	
	//calibrate(adc);
}

void check_black_white(ADC *adc)
{
	
	for ( int i = 0; i < 8; i++)
  {
    adc->sensorMax[i] = 0;
    adc->sensorMin[i] = 4095;
  }
  uint32_t time = HAL_GetTick();
  while(HAL_GetTick()-time <= 4000)
  {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
    //motor1.drive(50);
    //motor2.drive(-50);

    for ( int i = 0; i < 8; i++)
    {
      if (adc ->sensor_value[i] < adc->sensorMin[i])
      {
        adc->sensorMin[i] = adc ->sensor_value[i];
      }
      if (adc ->sensor_value[i] > adc->sensorMax[i])
      {
       adc->sensorMax[i] = adc ->sensor_value[i];
      }
    }
  }
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
	//time = HAL_GetTick();
}




void readCalibrated(ADC *adc) {

     for (int i = 0; i < NUM_SENSORS; i++) {
        if (adc->sensorMax[i] != adc->sensorMin[i]) {
					if(adc->sensor_value[i]>=adc->sensorMax[i]) adc->sensor_value[i]=adc->sensorMax[i];
					else if(adc->sensor_value[i]<=adc->sensorMin[i]) adc->sensor_value[i]=adc->sensorMin[i];
          adc->calibratedValues[i] = 1000 * (adc->sensor_value[i] - adc->sensorMin[i]) / (float)(adc->sensorMax[i] - adc->sensorMin[i]);
        } else {
            adc->calibratedValues[i] = 0;
        }
    }
}


uint16_t readLine(ADC *adc, uint8_t white_line) {
    
		
		
adc->avg =0 ;
adc->sum = 0;

    readCalibrated(adc); // chuan hoa gia tri adc tu 0 den 1000

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        adc->value[i] = adc->calibratedValues[i];// chuan hoa gia tri adc tu 0 den 1000
        if (white_line==1) {
            adc->value[i] = 1000 - adc->value[i];
        }

        check_on_line(adc);

        if (adc->value[i] > 50) {
            adc->avg += (long)(adc->value[i]) * (i * 1000);
            adc->sum += adc->value[i];
        }
    }

    /*if (adc->on_line==0) {
        if (adc->last_value < (NUM_SENSORS - 1) * 1000 / 2) {
            return (NUM_SENSORS - 1) * 1000;
        } else {
            return 0;
        }
    }*/
		readline_2(adc);
    adc->last_value = (uint16_t)(adc->avg / adc->sum);
    return adc->last_value;
    }
/*
    if (adc->on_line==0) {
        if (adc->last_value < (NUM_SENSORS - 1) * 1000 / 2) {
            return 0;
        } else {
            return (NUM_SENSORS - 1) * 1000;
        }
    }*/

		
		
uint8_t readline_2(ADC *adc)
{
	adc->check = 0;
	for (uint8_t i = 0; i < NUM_SENSORS; i++){
		adc->check = adc->check<<1;
		if (adc->value[i] >white_detect)
		{
			adc->check = adc->check | 0x01; //0b00000001
		}
		else {
			adc->check = adc->check & 0xfe; //0b11111110
		}
	}
	return adc->check;
}
void check_on_line (ADC *adc) {
	for (int i = 0; i < NUM_SENSORS; i++) {
        if (adc->value[i] > white_detect) 
					{
					adc->on_line = 1;
					return;
					}					
    }
	adc->on_line = 0;
		return;
}
	
void check_allSensorsDetectWhite (ADC *adc) {
    // 
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (adc->value[i] < black_detect) {  // neu 1 mat thay mau den thi khong tinh nua
            adc->allSensorsDetectWhite = 0;  // 
					return;
        }
    }
    adc->allSensorsDetectWhite = 1;  // N?u t?t c? c?m bi?n d?u phát hi?n "tr?ng", tr? v? true
}

void check_turn(ADC *adc) {
	if (d_check_crossline(adc) == false) {
		if(  
				   sensor_mask(readline_2(adc), 0xf0) == 0xf0  //11110000
			  || sensor_mask(readline_2(adc), 0xf8) == 0xf8  //11111000
				|| sensor_mask(readline_2(adc), 0xfc) == 0xfc  //11111100
			  )
					{
						adc->turn_right_filter++;
						
						
						return;
					}
					
		else if(  
				   sensor_mask(readline_2(adc), 0x0f) == 0x0f //00001111
			  || sensor_mask(readline_2(adc), 0x1f) == 0x1f //00011111
				|| sensor_mask(readline_2(adc), 0x3f) == 0x3f //00111111
			  )
					{
						adc->turn_left_filter++;
						
						return;
					}
		
	}
}
	bool d_check_crossline(ADC *adc){
	if( 
	      sensor_mask(readline_2(adc), 0xe7)   == 0xe7    //0b1110 0111
	  ||  sensor_mask(readline_2(adc), 0xc3)   == 0xc3    //0b1100 0011
		||  sensor_mask(readline_2(adc), 0xff)    == 0xff    //1111 1111
	  ||  sensor_mask(readline_2(adc), 0x81)   == 0x81
	 )
	{
	
		//beep_long(&buzzer,100);
		return true;
	}
	else return false;
}
bool d_check_rightline(ADC *adc){
		if (d_check_crossline(adc) == false) {
	  if(sensor_mask(readline_2(adc), 0x1f) == 0x1f //00011111
	  // ||sensor_mask(get_sensor_mask(&line_sensor), 0x0f) == 0x0f //00001111
		)
		{		

			return true;
		}
	}
	return false;
}
bool d_check_leftline(ADC* adc){
	if(d_check_crossline(adc) == false) {
	if(sensor_mask(readline_2(adc), 0xf8) == 0xf8		//11111000
//	 ||sensor_mask(get_sensor_mask(&line_sensor), 0xf0) == 0xf0		//11110000
	)
	{

		return true;
	}
	}
	return false;
}