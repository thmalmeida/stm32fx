#include "system_main.h"
#include "stdlib.h"
// includes for aht10 sensor
#include "i2c_master.hpp"
#include "aht10.hpp"

// includes for adc test
#include "adc_driver.hpp"

// includes for pcy8575
#include "pcy8575.hpp"
#include "tim.h"
#include "iwdg.h"
#include "backup.hpp"
#include "reset_reason.hpp"

void aht10_test(void);
void i2c_slave_pcy8575(void);
void adc_test(void);
void bkp_test(void);

int main(void)
{
	init_system();
	printf("\nSystem reset...\n");

	// aht10_test();
	// i2c_slave_pcy8575();
	// adc_test();
	// adc_test();
	bkp_test();

    return 0;
}

// Example to declare an array of GPIO_driver class
// #include "gpio"
// GPIO_driver pin0[] = {
// 	GPIO_driver{3, 1}, GPIO_driver{4, 1}, GPIO_driver{5, 1}, GPIO_driver{6, 1},
// 	GPIO_driver{7, 1}, GPIO_driver{8, 1}, GPIO_driver{9, 1}, GPIO_driver{10,1},
// 	GPIO_driver{11,1}, GPIO_driver{12,1}, GPIO_driver{13,1}, GPIO_driver{14,1},
// 	GPIO_driver{15,1}, GPIO_driver{16,1}, GPIO_driver{17,1}};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	printf("ADC cplt!\n");
	adc_dma_flag = 1;
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	printf("ADC half!\n");
	adc_dma_flag = 1;
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	printf("ADC: callback dma error\n");
}
void bkp_test(void) {

	uint16_t value = backup_DR1_get();
	printf("bkp DR1: 0x%04x, reset_reason:%d\n", value, static_cast<int>(reset_reason()));

	backup_DR1_set(value+1);
	printf("Restarting system...\n");
	HAL_Delay(4000);
	HAL_NVIC_SystemReset();

	while(1) {

	}
}
void adc_test(void) {
	// ADC_driver adc0;
	// adc0.init();
	tim3_init();
	adc_gpioa_config();
	adc_init();
	adc_dma_init();
	adc_dma_config_it();
	adc_dma_config((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
	// tim2_init();

	// adc_set_CR2_EXTSEL_bits(0x04);
	// adc_set_CR2_DMA_bit();

	adc_read_SR_reg();
	adc_print_SR_reg();
	adc_read_CR1_reg();
	adc_print_CR1_reg();
	adc_read_CR2_reg();
	adc_print_CR2_reg();

	memset(adc_buffer, 0, sizeof(adc_buffer));
	for(int j=0; j<ADC_BUFLEN; j++) {
		printf("ADC[%d]= %u\n", j, adc_buffer[j]);
	}
	// HAL_ADC_Start_DMA(&hadc1, &adc_buffer[0], ADC_BUFLEN); //Link DMA to ADC1
	// adc0.read_stream(&adc_buffer[0], ADC_BUFLEN);

	volatile int i = 0;
	// adc_start_conversion();

	while(1) {

		// 1 second flag to refresh watchdog timer
		if(tim3_flag_1sec) {
			tim3_flag_1sec = 0;
			// printf("TIM3\n");
			// adc_read_SR_reg();
			// adc_print_SR_reg();
			// adc_read_CR1_reg();
			// adc_print_CR1_reg();
			// adc_read_CR2_reg();
			// adc_print_CR2_reg();

			if(adc_read_SR_EOC_bit()) {
				printf("ADC1->DR: %lu\n", ADC1->DR);
			}

			i++;
			for(int j=0; j<ADC_BUFLEN; j++) {
				printf("ADC[%d]= %u\n", j, adc_buffer[j]);
			}
			printf("i: %d\n", i);

			if(i == ADC_BUFLEN) {
				i=0;
			}

			// adc_start_conversion();


			// HAL_ADC_Start(&hadc1);
			// printf("read value: %u\n", adc0.read(0));
			// printf("read stream:\n");
			// adc0.read_stream(&adc_raw_data[0], 3);
			// for(int i=0; i<3; i++) {
			// 	printf("%lu, ", adc_raw_data[i]);
			// }
		}

		// if(tim2_flag) {
		// 	tim2_flag = 0;
		// 	printf("TIM2!\n");
		// }

		if (adc_dma_flag) {
			adc_dma_flag = 0;
			for(int j=0; j<ADC_BUFLEN; j++) {
				printf("ADC[%d]= %u\n", j, adc_buffer[j]);
			}

			if(adc_dma_tc_flag) {
				adc_dma_tc_flag = 0;
				memset(adc_buffer, 0, sizeof(adc_buffer));
			}

		}
	}
}
void i2c_slave_pcy8575(void) {
	pcy8575 extender0;
	extender0.init();

	tim3_init();
	iwdg_init();

	while(1) {

		extender0.handle_message();
	
		// 1 second flag to refresh watchdog timer
		if(tim3_flag_1sec) {
			tim3_flag_1sec = 0;
			// printf("TIM3\n");
		}
	}
}
void aht10_test(void) {
	
	I2C_Master i2c;
	i2c.init(I2C_NORMAL_SPEED_HZ);

	aht10 sensor0(&i2c);
	sensor0.init(0);

	int count = 0;

	while (1)
	{
		// AHT10 test sensor
		if(sensor0.probe())
		{
			// printf("Sensor found!\n");
			sensor0.trig_meas();
			// sensor0.print_raw_data();
			printf("Count: %d, Humidity: %.2f %%, Temperature: %.2f C\n", count++, sensor0.get_humidity(), sensor0.get_temperature());
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		}
		else {
			i2c.deinit();
			HAL_Delay(100);
			i2c.init(I2C_NORMAL_SPEED_HZ);
		}
		// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(30000);
	}
}
