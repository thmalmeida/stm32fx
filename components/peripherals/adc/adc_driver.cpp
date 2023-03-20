/*
 * adc.cpp
 *
 *  Created on: 15 de mar de 2023
 *      Author: titi
 */

#include "adc_driver.hpp"

// void ADC::adc_begin()
// {
// 	// Enable the ADC1 clock
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

// 	ADC_InitTypeDef ADC_InitStructure;
// 	//ADC1 configuration
// 	//select independent conversion mode (single)
// 	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
// 	//We will convert single channel only
// 	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
// 	//we will convert one time
// 	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
// 	//select no external triggering
// 	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
// 	//right 12-bit data alignment in ADC data register
// 	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
// 	//single channel conversion
// 	ADC_InitStructure.ADC_NbrOfChannel = 1;
// 	//load structure values to control and status registers
// 	ADC_Init(ADC1, &ADC_InitStructure);
// 	//wake up temperature sensor
// 	ADC_TempSensorVrefintCmd(ENABLE);
// 	//ADC1 channel16 configuration
// 	//we select 41.5 cycles conversion for channel16
// 	//and rank=1 which doesn't matter in single mode
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_41Cycles5);
// 	//Enable ADC1
// 	ADC_Cmd(ADC1, ENABLE);
// 	//Enable ADC1 reset calibration register
// 	ADC_ResetCalibration(ADC1);
// 	//Check the end of ADC1 reset calibration register
// 	while(ADC_GetResetCalibrationStatus(ADC1));
// 	//Start ADC1 calibration
// 	ADC_StartCalibration(ADC1);
// 	//Check the end of ADC1 calibration
// 	while(ADC_GetCalibrationStatus(ADC1));
// }
// int ADC::adc_readChannel(uint8_t channel)
// {
// 	uint16_t AD_value;

// 	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_41Cycles5);

// 	//Start ADC1 Software Conversion
// 	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
// 	//wait for conversion complete
// 	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)){}
// 	//read ADC value
// 	AD_value=ADC_GetConversionValue(ADC1);
// 	//clear EOC flag
// 	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

// 	return (AD_value >> 2);
// }
