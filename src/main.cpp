#include "system_main.h"
#include "stdlib.h"
// includes for aht10 sensor
#include "i2c_master.hpp"
#include "aht10.hpp"
#include "gpio.hpp"

// includes for adc test
#include "adc_driver.hpp"

#include "tim_driver.hpp"

// includes for pcy8575
#include "pcy8575.hpp"
#include "tim.h"
#include "iwdg.h"
#include "backup.hpp"
#include "reset_reason.hpp"


void i2c_slave_pcy8575(void);

void test_adc_dma(void);
void test_adc_dma_tim_class(void);
void test_adc_single_read(void);
void test_aht10(void);
void test_bkp(void);
void test_pjb20(void);
void test_timer_pwm(void);

int main(void)
{
	init_system();
	printf("\nSystem reset...\n");

	// i2c_slave_pcy8575();
	// test_adc_dma();
	// test_timer_pwm();
	test_pjb20();
	// test_aht10();
	// test_bkp();
	// test_pwm();

    return 0;
}

void i2c_slave_pcy8575(void) {
	pcy8575 extender0;
	extender0.init();

	tim3_init();
	iwdg_init();

	while(1) {

		extender0.handle_message();
	
		// 1 second flag to refresh watchdog timer
		if(tim3_flag) {
			tim3_flag = 0;
			// printf("TIM3\n");
		}
	}
}
void test_timer_pwm(void) {
	TIM_driver tim0(2, 800, timer_mode::pwm_output, 4);

	tim0.set_duty_cycle(50);

	int i = 0;
	while(1) {
		if(i>99) {
			i=0;
		}
		tim0.set_duty_cycle(i);
		HAL_Delay(1000);
		printf("Count:%d\n",i++);
	}
	
}
void test_pjb20(void) {
	// This repoduces the behavior or PJB-20 electric panel
	// TODO list:
	// - implement PID controller. Slow response.
	
	// Using PB0
	TIM_driver tim0(3, 800, timer_mode::pwm_output, 3);
	ADC_driver adc3(2, adc_read_mode::single_read);

	uint16_t adc_value = 0;				// instant adc read input value

	int n_bits = 12;
	uint16_t min_adc_value = 1;			// minimum adc value to shutdown pwm signal;
	int max = 100*0.78;					// maximum duty cycle pwm operation value;
	int min = 0.42*max;					// minimum duty cycle pwm operation value;

	enum class states {
		off = 0,
		on
	};

	states fsm0 = states::off;			// Finite state machine state initialize;
	uint16_t pwm_set_point = 0, pwm_pid = 0;
	int error = 0;
	tim0.set_duty_cycle(0);

	while(1) {
		adc_value = adc3.read(3);

		if(fsm0 == states::off) {
			if(adc_value > min_adc_value) {
				fsm0 = states::on;
			}
		} else if(fsm0 == states::on) {

			pwm_set_point = (max-min)*adc_value/(4095)+min;
			error = pwm_set_point - pwm_pid;

			if(error) {
				if(error > 0) {
					pwm_pid++;
				} else if(error < 0) {
					pwm_pid--;
				}
				tim0.set_duty_cycle(pwm_pid);
				// HAL_Delay(1);
				delay_us(2000);		
			}

			if(adc_value < min_adc_value) {
				fsm0 = states::off;
				pwm_pid = 0;
				tim0.set_duty_cycle(pwm_pid);
			}
		// printf("ADC3_: %u\n", adc_value);
		}
	}
}
void test_adc_dma_tim_class(void) {

	TIM_driver tim1(1, 1, timer_mode::timer_interrupt);
	TIM_driver tim2(2, 1, timer_mode::timer_interrupt);
	TIM_driver tim3(3, 1, timer_mode::timer_interrupt);
	TIM_driver tim4(4, 1, timer_mode::timer_interrupt);

	// ADC_driver adc0;
	// adc0.init();	
	adc_dma_begin((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
	memset(adc_buffer, 0, sizeof(adc_buffer));

	int i = 0;
	int flag0 = 0;
	printf("test_adc_dma_tim_class!\n");
	// adc_start_conversion();

	while(1) {

		// 1 second flag to refresh watchdog timer
		if(tim1_flag_) {
			tim1_flag_ = 0;
			printf("TIM%d\n", tim1.get_tim_number());
		}

		if(tim2_flag_) {
			tim2_flag_ = 0;
			printf("TIM%d\n", tim2.get_tim_number());
		}

		if(tim4_flag_) {
			tim4_flag_ = 0;
			printf("TIM%d\n", tim4.get_tim_number());
		}

		if(tim3_flag_) {
			tim3_flag_ = 0;
			printf("TIM%d\n", tim3.get_tim_number());

			// adc_read_SR_reg();
			// adc_read_CR1_reg();
			// adc_read_CR2_reg();

			// adc_print_SR_reg();
			// adc_print_CR1_reg();
			// adc_print_CR2_reg();
	
			// dma1_read_ISR_reg();
			// dma1_read_CNDTR_reg();
			// dma1_read_CPAR_reg();
			// dma1_read_CMAR_reg();

			// dma1_print_ISR_reg();
			// dma1_print_CNDTR_reg();
			// dma1_print_CPAR_reg();
			// dma1_print_CMAR_reg();

			if(i>3) {
				
				// DMA1_Channel1->CCR &= ~(1<<0);
				// DMA1_Channel1->CNDTR = ADC_BUFLEN;
				// DMA1_Channel1->CCR |= (1<<0);
				printf("Start conversion!\n");
				adc_start_conversion();
			}
			// else {
				// adc_stop_conversion();
				// DMA1_Channel1->CCR &= ~(1<<0);
				// DMA1_Channel1->CNDTR = ADC_BUFLEN;
				// DMA1_Channel1->CCR |= (1<<0);
				// ADC1->SR = 0;
				// ADC1->CR2 |= (1<<0);
				// i = 0;
				// if(!flag0) {
				// 	flag0 = 1;
					// printf("ADC OFF!\n");
					// ADC1->SR = 0;
					// // ADC1->CR2 &= ~(1<<)
					// adc_power_off();
					// delay_ms(5);
					// ADC1->SR = 0;
					// adc_power_on();
					// ADC1->SR = 0;
					// delay_ms(5);
					// i = 0;
				// }
			// }

			// adc_read_SR_reg();
			// adc_print_SR_reg();
			// adc_read_CR1_reg();
			// adc_print_CR1_reg();
			// adc_read_CR2_reg();
			// adc_print_CR2_reg();

			// if(adc_read_SR_EOC_bit()) {
			// 	printf("ADC1->DR: %lu\n", ADC1->DR);
			// }

			i++;
			// for(int j=0; j<ADC_BUFLEN; j++) {
			// 	printf("ADC[%d]= %u\n", j, adc_buffer[j]);
			// }
			// printf("i: %d\n", i);

			// if(i == ADC_BUFLEN) {
			// 	i=0;
			// }

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

			if(adc_dma_te_flag) {
				printf("DMA1 TE flag!\n");
				adc_dma_te_flag = 0;
			}

			if(adc_dma_ht_flag) {
				printf("DMA1 HT flag!\n");
				adc_dma_ht_flag = 0;
			}

			if(adc_dma_tc_flag) {
				adc_dma_tc_flag = 0;
				memset(adc_buffer, 0, sizeof(adc_buffer));
				printf("DMA1 TC flag!\n");

				adc_dma_reset_cnt();	
				// adc_dma_config_addr((uint32_t*)(&adc_buffer[0]), adc_buffer);
				// adc_dma_begin((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
				// ADC1->CR2 |= ~(7<<17);
				// adc_stop_conversion();
				// dma1_read_CNDTR_reg();
				// dma1_print_CNDTR_reg();

				// ADC1->CR1 &= ~(1<<2);
				// ADC1->SR &= ~(1<<4);
				// ADC1->SR &= ~(1<<1);
			}

			if(adc_dma_gi_flag) {
				printf("DMA1 GI flag!\n");
				adc_dma_gi_flag = 0;
			}		
		}
	}

}
void test_adc_dma(void) {

	tim3_init();

	// ADC_driver adc0;
	// adc0.init();	
	adc_dma_begin((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
	memset(adc_buffer, 0, sizeof(adc_buffer));



	// enum class adc_states {
	// 	waiting = 0,
	// 	ready
	// }

	// adc_states adc_state adc_states::waiting;
	// adc_gpioa_config();
	// adc_init();
	// adc_dma_init();
	// adc_dma_config_it();
	// adc_dma_config((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
	// tim2_init();

	// adc_set_CR2_EXTSEL_bits(0x04);
	// adc_set_CR2_DMA_bit();

	// adc_read_SR_reg();
	// adc_print_SR_reg();
	// adc_read_CR1_reg();
	// adc_print_CR1_reg();
	// adc_read_CR2_reg();
	// adc_print_CR2_reg();

	// for(int j=0; j<ADC_BUFLEN; j++) {
	// 	printf("ADC[%d]= %u\n", j, adc_buffer[j]);
	// }
	// HAL_ADC_Start_DMA(&hadc1, &adc_buffer[0], ADC_BUFLEN); //Link DMA to ADC1
	// adc0.read_stream(&adc_buffer[0], ADC_BUFLEN);

	int i = 0;
	int flag0 = 0;
	// adc_start_conversion();

	while(1) {

		// 1 second flag to refresh watchdog timer
		if(tim3_flag) {
			tim3_flag = 0;
			printf("TIM3\n");

			// adc_read_SR_reg();
			// adc_read_CR1_reg();
			// adc_read_CR2_reg();

			// adc_print_SR_reg();
			// adc_print_CR1_reg();
			// adc_print_CR2_reg();
	
			dma1_read_ISR_reg();
			dma1_read_CNDTR_reg();
			dma1_read_CPAR_reg();
			dma1_read_CMAR_reg();

			dma1_print_ISR_reg();
			dma1_print_CNDTR_reg();
			dma1_print_CPAR_reg();
			dma1_print_CMAR_reg();

			if(i>3) {
				
				// DMA1_Channel1->CCR &= ~(1<<0);
				// DMA1_Channel1->CNDTR = ADC_BUFLEN;
				// DMA1_Channel1->CCR |= (1<<0);
				printf("Start conversion!\n");
				adc_start_conversion();
			}
			// else {
				// adc_stop_conversion();
				// DMA1_Channel1->CCR &= ~(1<<0);
				// DMA1_Channel1->CNDTR = ADC_BUFLEN;
				// DMA1_Channel1->CCR |= (1<<0);
				// ADC1->SR = 0;
				// ADC1->CR2 |= (1<<0);
				// i = 0;
				// if(!flag0) {
				// 	flag0 = 1;
					// printf("ADC OFF!\n");
					// ADC1->SR = 0;
					// // ADC1->CR2 &= ~(1<<)
					// adc_power_off();
					// delay_ms(5);
					// ADC1->SR = 0;
					// adc_power_on();
					// ADC1->SR = 0;
					// delay_ms(5);
					// i = 0;
				// }
			// }

			// adc_read_SR_reg();
			// adc_print_SR_reg();
			// adc_read_CR1_reg();
			// adc_print_CR1_reg();
			// adc_read_CR2_reg();
			// adc_print_CR2_reg();

			// if(adc_read_SR_EOC_bit()) {
			// 	printf("ADC1->DR: %lu\n", ADC1->DR);
			// }

			i++;
			// for(int j=0; j<ADC_BUFLEN; j++) {
			// 	printf("ADC[%d]= %u\n", j, adc_buffer[j]);
			// }
			// printf("i: %d\n", i);

			// if(i == ADC_BUFLEN) {
			// 	i=0;
			// }

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

			if(adc_dma_te_flag) {
				printf("DMA1 TE flag!\n");
				adc_dma_te_flag = 0;
			}

			if(adc_dma_ht_flag) {
				printf("DMA1 HT flag!\n");
				adc_dma_ht_flag = 0;
			}

			if(adc_dma_tc_flag) {
				adc_dma_tc_flag = 0;
				memset(adc_buffer, 0, sizeof(adc_buffer));
				printf("DMA1 TC flag!\n");
				
				// adc_dma_config_addr((uint32_t*)(&adc_buffer[0]), adc_buffer);
				// adc_dma_begin((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
				// ADC1->CR2 |= ~(7<<17);
				// adc_stop_conversion();
				// dma1_read_CNDTR_reg();
				// dma1_print_CNDTR_reg();

				// ADC1->CR1 &= ~(1<<2);
				// ADC1->SR &= ~(1<<4);
				// ADC1->SR &= ~(1<<1);
			}

			if(adc_dma_gi_flag) {
				printf("DMA1 GI flag!\n");
				adc_dma_gi_flag = 0;
			}		
		}
	}
}
void test_adc_single_read(void) {
	ADC_driver adc3(3, adc_read_mode::single_read);

	while(1) {
		printf("ADC3: %d\n", adc3.read(3));
		HAL_Delay(1000);
	}
}
void test_aht10(void) {
	
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
void test_bkp(void) {

	uint16_t value = backup_DR1_get();
	printf("bkp DR1: 0x%04x, reset_reason:%d\n", value, static_cast<int>(reset_reason()));

	backup_DR1_set(value+1);
	printf("Restarting system...\n");

	while(1) {

	}
}

// Example to declare an array of GPIO_driver class
// #include "gpio"
// GPIO_driver pin0[] = {
// 	GPIO_driver{3, 1}, GPIO_driver{4, 1}, GPIO_driver{5, 1}, GPIO_driver{6, 1},
// 	GPIO_driver{7, 1}, GPIO_driver{8, 1}, GPIO_driver{9, 1}, GPIO_driver{10,1},
// 	GPIO_driver{11,1}, GPIO_driver{12,1}, GPIO_driver{13,1}, GPIO_driver{14,1},
// 	GPIO_driver{15,1}, GPIO_driver{16,1}, GPIO_driver{17,1}};
