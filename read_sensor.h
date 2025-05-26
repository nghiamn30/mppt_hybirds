#ifndef __READ_SENSOR_H
#define __READ_SENSOR_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"     
#include "stm32f10x_adc.h"              
#include "adc.h"


typedef struct
{	
	uint32_t SVI;
	uint32_t SCI;
	uint32_t DCI;
	uint32_t LVO;
	uint32_t LCO;
	uint32_t BVI;
	uint32_t BVO;
	uint32_t TSV;
	
	uint32_t solarVoltageInput; 
	uint32_t solarCurrentInput; 
	uint32_t dcVoltageInput; 
	uint32_t loadVoltageOutput; 
	uint32_t loadCurrentOutput; 
	uint32_t batVoltageInput; 
	uint32_t batVoltageOutput; 
	uint32_t temperature; 
	
} system_parameter;
extern system_parameter sensorValue;

void readAllSensor(void);

#endif /* __READ_SENSOR_H */
