#include "read_sensor.h"
#include "adc.h"

system_parameter sensorValue;

void readAllSensor(void) 
{
	sensorValue.SVI = 0;
	sensorValue.SCI = 0;
	sensorValue.DCI = 0;
	sensorValue.LVO = 0;
	sensorValue.LCO = 0;
	sensorValue.BVI = 0;
	sensorValue.BVO = 0;
	sensorValue.TSV = 0;
	
	int i = 0;
	for(i = 0; i < 10; i++) 
	{
		sensorValue.SVI += RD_ADC_GetSolarVoltage();
		sensorValue.SCI += 0;
		sensorValue.DCI += RD_ADC_GetDCVoltage();
		sensorValue.LVO += RD_ADC_GetLoadVoltage();
		sensorValue.LCO += RD_ADC_GetLoadCurrent();
		sensorValue.BVI += RD_ADC_GetBatteryVoltage();
		sensorValue.BVO += 0;
		sensorValue.TSV += RD_ADC_GetTemperature;
	}
	
	sensorValue.solarVoltageInput = sensorValue.SVI/10;
	sensorValue.solarCurrentInput = sensorValue.SCI/10;
	sensorValue.dcVoltageInput = sensorValue.DCI/10;
	sensorValue.loadVoltageOutput = sensorValue.LVO/10;
	sensorValue.loadCurrentOutput = sensorValue.LCO/10;
	sensorValue.batVoltageInput = sensorValue.BVI/10;
	sensorValue.batVoltageOutput = sensorValue.BVO/10;
	sensorValue.temperature = sensorValue.TSV/10;
	
}
