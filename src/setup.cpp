#include "setup.hpp"

void i2c_slave_pcy8575(void) {
	// Extender machine. Composed init of Timer 3 initialized with 1 Hz on interrupt mode. tim3_flag_ ISR variable;
	pcy8575 extender0;

	while(1) {
		extender0.run();
	}
}
void i2c_slave_pcy8575_adc(void) {

	// ---- ADC signal parameters
	// 1- just for find the number of points and it's sampling rate;
	// 2- another reason to use adc0 class outside pcy8575 is to keep adc_array_raw vector allocated.
	// 
	// Sampling parameters for STM32F103C8T6 device
	float F_clk = 72000000;							// Main clock system [Hz]
	float div_1 = 1;								// Advanced High-performance Bus (AHB) prescale;
	float div_2 = 2;								// Advanced Peripheral Bus (APB2) prescale;
	float div_3 = 8;								// ADC prescale
	float adc_clk = F_clk/div_1/div_2/div_3;		// ADC clock after all prescalers
	float T_conv = 12.5 + 239.5;					// Number of clock cycles to make one conversion
	float Fs_adc = adc_clk/T_conv;					// Sample rate calculation [Samples/s];

	float f_signal = 60.0;							// signal frequency [Hz]
	float n_points_cycle = Fs_adc/f_signal;			// number of points per cycle or period time

	int n_cycles = 2;								// Number of cycles desired to analyze
	int n_points = ceil(n_cycles*n_points_cycle);	// The number of points is calculated based on the ADC sample rate.
	// uint16_t adc_array_raw[n_points];				// Array allocation to receive converted points
	// uint16_t *adc_array_raw = new uint16_t[n_points];
	// uint16_t *adc_array_raw;

	printf("Sample rate: %f\n", Fs_adc);
	printf("points/cycle: %f\n", n_points_cycle);
	printf("n cycles: %d\n", n_cycles);
	printf("total points: %d\n", n_points);

	ADC_Driver adc0(adc_mode::stream);
	adc0.stream_init();
	adc0.channel_config(3);
	// adc0.stream_addr_config(adc_array_raw);
	// adc0.stream_length_config(n_points);
	// memset(adc_array_raw, 0, sizeof(adc_array_raw));
	// ---- end	

	// Extender machine. Composed init of Timer 3 initialized with 1 Hz on interrupt mode. tim3_flag_ ISR variable;
	pcy8575 extender0;
	extender0.init();
	// extender0.adc_config(&adc0);					// send class reference point to pcy8575;

	while(1) {
		extender0.run();
	}
}
void i2c_slave_pcy8575_debug(void) {
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

void weighing_scale(void) {

	I2C_Driver i2c(2);
	Weighing_Scale ws(&i2c);

	double Tp = 0.500;	// time period [s]
	TIM_Driver timer0(2, 1/Tp, timer_mode::timer_interrupt);

	while(1) {

		ws.run();

		if(timer0.isr_flag()) {
			ws.update();
		}
	}
}

void test_adc_class(void) {
	// Configure timer number 3 to 1 Hertz frequency update into interrupt mode.
	TIM_Driver tim3(3, 1, timer_mode::timer_interrupt);

	// Configure adc channel to using dma mode;
	ADC_Driver adc0(adc_mode::stream);
	adc0.channel_config(3);

	while(1) {

	}
}
void test_adc_dma(void) {

	tim3_init();

	// ADC_Driver adc0;
	// adc0.init();
	// adc_dma_begin((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
	adc_dma_begin(&adc_buffer[0], ADC_BUFLEN);
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
void test_adc_dma_tim_class(void) {

	TIM_Driver tim1(1, 1, timer_mode::timer_interrupt);
	TIM_Driver tim2(2, 1, timer_mode::timer_interrupt);
	TIM_Driver tim3(3, 1, timer_mode::timer_interrupt);
	TIM_Driver tim4(4, 1, timer_mode::timer_interrupt);

	// ADC_Driver adc0;
	// adc0.init();
	// adc_dma_begin((uint32_t*)&adc_buffer[0], ADC_BUFLEN);
	adc_dma_begin(&adc_buffer[0], ADC_BUFLEN);
	memset(adc_buffer, 0, sizeof(adc_buffer));

	int i = 0;
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

				adc_dma_set_addr_cnt(ADC_BUFLEN);	
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
void test_adc_oneshot(void) {
	ADC_Driver adc0(adc_mode::oneshot);

	adc0.channel_config(3);
	// adc0.oneshot_channel_config(3);
	// adc0.oneshot_channel_config(4);
	// adc0.oneshot_channel_config(16);
	// adc0.oneshot_channel_config(17);

	// adc0.ptable_print();

	int n_channels = 1;
	int adc_data_raw[n_channels];

	// adc_print_DISCNUM();
	// adc_print_L_regular();

	while(1) {

		adc_data_raw[0] = adc0.read(3);
		// adc_data_raw[1] = adc0.read(3);
		// adc_data_raw[2]  = adc0.read(4);
		// adc_data_raw[3] = adc0.read(16);
		// adc_data_raw[4] = adc0.read(17);
		// adc0.read_all();
		// adc0.ptable_print_();
		// printf("ch2:%d\n", adc0.get_raw(2));
		// printf("ch17:%d\n", adc0.get_raw(17));
		// printf("\nADC1: \n");
		for(auto i=0; i<n_channels; i++) {
			printf("adc_data_raw[%d]:%d\n", i, adc_data_raw[i]);
		}
		delay_ms(100);
	}
}
void test_adc_stream(void) {
	printf("test_adc_stream\n");
	// Sampling parameters for STM32F103C8T6 device
	float F_clk = 48000000;							// Main clock system [Hz]
	float div_1 = 1;								// Advanced High-performance Bus (AHB) prescale;
	float div_2 = 2;								// Advanced Peripheral Bus (APB2) prescale;
	float div_3 = 8;								// ADC prescale
	float adc_clk = F_clk/div_1/div_2/div_3;		// ADC clock after all prescalers
	float T_conv = 12.5 + 239.5;					// Number of clock cycles to make one conversion
	float Fs_adc = adc_clk/T_conv;

	float f_signal = 60.0;
	float n_points_cycle = Fs_adc/f_signal;
	
	int n_cycles = 1;
	int n_points = ceil(n_cycles*n_points_cycle);	// The number of points is calculated based on the ADC sample rate.
	uint16_t adc_array_raw[n_points];				// Array allocation to receive converted points

	printf("Sample rate: %f\n", Fs_adc);
	printf("points/cycle: %f\n", n_points_cycle);
	printf("n cycles: %d\n", n_cycles);
	printf("total points: %d\n", n_points);

	ADC_Driver adc0(adc_mode::stream);
	adc0.channel_config(3);
	adc0.stream_addr_config(&adc_array_raw[0]);
	adc0.stream_length_config(n_points);
	memset(adc_array_raw, 0, sizeof(adc_array_raw));

	printf("\nadc_array_raw[0] addr %p:: ", &adc_array_raw[0]);
	for(auto i=0; i<n_points; i++) {
		printf("%u, ", adc_array_raw[i]);
	}
	printf("\n");

	while(1) {
		adc0.stream_read();

		printf("\nadc_array_raw[0] addr %p:: ", &adc_array_raw[0]);
		for(auto i=0; i<n_points; i++) {
			printf("%u, ", adc_array_raw[i]);
		}
		printf("\n");
		// printf("\n\nadc_array_raw: \n");
		// for(auto i=0; i<n_points; i++) {
		// 	printf("%u, ", adc_array_raw[i]);
		// }
		delay_ms(1000);
	}
}
void test_adc_stream_reg(void) {
	printf("test_adc_stream_reg\n");
	// Sampling parameters for STM32F103C8T6 device
	float F_clk = 48000000;							// Main clock system [Hz]
	float div_1 = 1;								// Advanced High-performance Bus (AHB) prescale;
	float div_2 = 2;								// Advanced Peripheral Bus (APB2) prescale;
	float div_3 = 8;								// ADC prescale
	float adc_clk = F_clk/div_1/div_2/div_3;		// ADC clock after all prescalers
	float T_conv = 12.5 + 239.5;					// Number of clock cycles to make one conversion
	float Fs_adc = adc_clk/T_conv;

	float f_signal = 60.0;
	float n_points_cycle = Fs_adc/f_signal;
	
	int n_cycles = 3;
	int n_points = ceil(n_cycles*n_points_cycle);	// The number of points is calculated based on the ADC sample rate.
	uint16_t adc_array_raw[n_points];				// Array allocation to receive converted points

	printf("Sample rate: %f\n", Fs_adc);
	printf("points/cycle: %f\n", n_points_cycle);
	printf("n cycles: %d\n", n_cycles);
	printf("total points: %d\n", n_points);

	ADC_Driver adc0(adc_mode::stream);
	adc0.channel_config(3);

	// adc_dma_begin((uint32_t*)&adc_array_raw[0], n_points);
	adc_dma_begin(&adc_array_raw[0], n_points);
	memset(adc_array_raw, 0, sizeof(adc_array_raw));

	// adc0.array_addr_config((uint32_t*)&adc_array_raw[0], n_points);

	// std::cout << "teste" << std::endl;
	dma1_read_ISR_reg();
	dma1_read_CNDTR_reg();
	dma1_read_CPAR_reg();
	dma1_read_CMAR_reg();

	dma1_print_ISR_reg();
	dma1_print_CNDTR_reg();
	dma1_print_CPAR_reg();
	dma1_print_CMAR_reg();

	adc_start_conversion();

	int count = 0;
	while(1) {

		printf("next:%d\n", count++);
		// dma1_read_ISR_reg();
		// dma1_read_CNDTR_reg();
		// dma1_read_CPAR_reg();
		// dma1_read_CMAR_reg();

		// dma1_print_ISR_reg();
		// dma1_print_CNDTR_reg();
		// dma1_print_CPAR_reg();
		// dma1_print_CMAR_reg();

		if(adc_dma_tc_flag) {
			adc_dma_tc_flag = 0;

			printf("\nadc_array_raw: ");
			for(auto i=0; i<n_points; i++) {
				printf("%u, ", adc_array_raw[i]);
			}
			printf("\n");

			adc_dma_set_addr_cnt(n_points);
		}
		delay_ms(1000);
	}
}
void test_aht10(void) {
	
	I2C_Driver i2c(2);

	printf("probe list\n");
	i2c.probe_list();
	printf("probe list done\n");
	
	aht10 sensor0(&i2c);
	sensor0.init(aht10_mode::NORMAL_MODE);

	GPIO_Driver led0(1, 1);
	led0.write(1);
	int count = 0;

	while (1)
	{
		// AHT10 test sensor
		if(sensor0.probe())
		{
			sensor0.trig_meas();
			// sensor0.print_raw_data();
			printf("Count: %d, Humidity: %.2f %%, Temperature: %.2f C\n", count++, sensor0.get_humidity(), sensor0.get_temperature());
			led0.write(0);
			delay_ms(200);
			led0.write(1);
		}
		else {
			i2c.deinit();
			delay_ms(100);
			i2c.init();
		}
		// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		delay_ms(10000);
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
void test_i2c(void) {
	// instantiate and initialize using port I2C2 (PB10 and PB11);
	I2C_Driver i2c1(2);

	printf("probe_find addr: 0x%02x\n", i2c1.probe_find());
	printf("probe list start\n");
	i2c1.probe_list();

	int count = 0;
	while (1) {
		printf("%d I2C probe:%d\n", count++, i2c1.probe(0x38));
		// printf("I2C %d - probe addr 0x39:%d\n", count++, i2c1.probe(0x39));
		delay_ms(1000);
	}
}
void test_lcd(void) {
	LCD_Driver lcd0;

	char c = '0';
	
	char str[10];
	sprintf(str, "Benjamin");
	lcd0.print(str, 0, 0);

	while(1) {
		lcd0.position(1, 10);
		lcd0.write(c);
		lcd0.write(0xdf);
		printf("write:%c 0x%02x\n", c, 0xde);
		c++;
		// lcd0.home();
		delay_ms(500);
	}

}
void test_pjb20(void) {
	// This repoduces the behavior or PJB-20 electric panel
	// TODO list:
	// - implement PID controller. Slow response.
	
	// Using PB0
	TIM_Driver tim0(3, 800, timer_mode::pwm_output, 3);
	ADC_Driver adc3(adc_mode::oneshot);

	uint16_t adc_value = 0;				// instant adc read input value

	// int n_bits = 12;
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
	tim0.duty(0);

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
				tim0.duty(pwm_pid);
				// delay_ms(1);
				delay_ms(2);		
			}

			if(adc_value < min_adc_value) {
				fsm0 = states::off;
				pwm_pid = 0;
				tim0.duty(pwm_pid);
			}
		// printf("ADC3_: %u\n", adc_value);
		}
	}
}
void test_pwm(void) {

	// GPIO_Driver pin(10, 1);
	// TIM 3 on channel 1 use the pin PA6;
	pwm_tim3_ch1();
	// MX_TIM3_Init();

	uint16_t TIMX_SMCR = TIM3->SMCR;
	// tim2_pwm_init(500);
	printf("TIMX_SMCR:0x%04x\n", TIMX_SMCR);
	printf("TIMX_CR1:0x%04lx\n", TIM3->CR1);
	printf("TIMX_CR2:0x%04lx\n", TIM3->CR2);
	while(1) {
		// pin.toggle();
		// printf("TIM3_CH1\n");
		printf("TIM2_CH3\n");
		delay_ms(1000);
	}
}
void test_ssd1306(void) {
	I2C_Driver i2c(2);
	SSD1306 d0(&i2c);

	d0.clear();

	int count = 0;
	char str[4];
	// d0.print(str, 0, 0);
	// d0.print_Arial24x32(str, 0, 0);
	
	// uint8_t i = 0;
	while(1) {
		sprintf(str, "%4d", count++);
		printf(str);
		d0.print_Arial24x32_Numbers(str, 0, 0);
		// d0.print_Arial24x32_Numbers(str, 24*4, 0);
		// d0.print_Arial16x24(str, 0, 0);
		// if(i < 64) {
		// 	sprintf(str, "c:%d", count++);
		// 	d0.print(str, 0, 24);
		// 	// d0.draw_pixel(i, i*2);
		// 	i++;
		// }
		// else {
		// 	i=0;
		// }
		delay_ms(1000);
	}
}
void test_time(void) {
	struct tm *p;
	time_t t;

	// t = time(NULL);		
	// t = 10;

	TIM_Driver tim3(3, 1, timer_mode::timer_interrupt);

	const auto p1 = std::chrono::system_clock::now();

	printf("Time since epoch: %llu\n", std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());

	printf("sizeof(t): %d\n", sizeof(t));

	while(1) {
		t += tim3.get_uptime();
		p = localtime(&t);
		printf("%lu, %s\n",static_cast<uint32_t>(t), asctime(p));
		// printf("%u\n", 	tim3.get_uptime());
		delay_ms(1000);
	}

}
void test_timer_interrupt(void) {

	printf("- test_tim_class_interrupt_mode\n");

	// tim3_init();
	// TIM_Driver tim1(1, 1, timer_mode::timer_interrupt);
	// TIM_Driver tim2(2, 1, timer_mode::timer_interrupt);

	// Timer period [s];
	double Tp = 1;
	TIM_Driver timer0(2, 1/Tp, timer_mode::timer_interrupt);

	uint32_t i = 0;
	while(1) {

		if(timer0.isr_flag()) { //tim3.get_isr_flag()) {
			// tim3.set_TIMx_EGR_UG();
			// TIM3->EGR |= (1<<0);
			// tim2.get_AHB_APBx_div_();
			// tim2.get_TIM_ARR_();
			// tim2.get_TIM_PSC_();
			// tim2.get_TIM_CKD_();
			// tim2.get_TIM_CR1_();
			printf("TIM2: %lu\n", i++);
			// printf("TIM%d\n", tim3.get_tim_number());
		}

		// 1 second flag to refresh watchdog timer
		// if(tim1_flag_) {
		// 	tim1_flag_ = 0;
		// 	printf("TIM%d\n", tim1.get_tim_number());
		// }

		// if(tim2.get_isr_flag()) {
		// 	i++;
		// 	if(i<3)
		// 		tim2.set_isr_flag();
		// 	printf("TIM%d\n", tim2.get_tim_number());
		// }

		// NOP function. Just to compile avoid to discart the interruption that do nothing;
		// if(i == 1000000) {
		// 	i = 0;
		// } else {
		// 	i++;
		// }

		// if(tim4_flag_) {
		// 	tim4_flag_ = 0;
		// 	printf("TIM%d\n", tim4.get_tim_number());
		// }
	}
}
void test_timer_pwm(void) {

	// time period [s];
	double Tp = 1.0/40000;

	// Select timer module 3, f = 1/Tp, pwm output mode and channel 1 (TIM3_CH1 --> PA6)
	TIM_Driver timer0(3, 1/Tp, timer_mode::pwm_output, 1);
	timer0.duty(50);

	int i = 0;
	while(1) {
		delay_ms(1000);
		printf("Count:%d\n",i++);
	}
}
void test_timer_counter(void) {

	// time period [s];
	double Tp = 1.0/1000;
	TIM_Driver timer0(3, 1/Tp, timer_mode::timer_counter);
	GPIO_Driver pin(1,1);

	// const uint32_t p_cnt = static_cast<uint32_t>(timer0.get_ARR()/(1/T_us)-4);
	const uint32_t p_cnt = static_cast<uint32_t>(timer0.get_ARR()-5);

	uint32_t i = 0;
	while(1) {

		timer0.reset_cnt();
		while(timer0.get_cnt() < p_cnt);			

		pin.toggle();
		printf("CNT:%lu\n", i++);
		
	}
}
void test_gpio(void) {

	// GPIO_Driver pin[2]{{4,1}, {5,1}};
	GPIO_Driver pin0(14,1);
	// TPI->ACPR = HAL_RCC_GetHCLKFreq() / 2000000 - 1;

	printf("GPIO test!\n");

	// delay_us(10);

	// timer0.set_TIM_EGR_UG_bit();
	// printf("p_cnt: %lu\n", p_cnt);
	// uint32_t i=0;
	while(1) {

		// if(timer0.isr_flag()) {
			// timer0.reset_cnt();
		// while(timer0.get_cnt() < p_cnt);			
		// pin[0].toggle();
		// pin[1].toggle();
		pin0.toggle();
		// printf("%lu\n", i++);
			// printf("CNT:%lu\n", count++);
		// }
		// printf("PWM test\n");
		delay_ms(500);



			// if(count > v) {
			// 	count = 0;
			// 	printf("CNT:%lu\n", count2++);
			// }
			// else
			// 	count++;
		// }
		// count = 0;
		// timer_us.enable_cnt();
		// timer_us.reset_cnt();
		// while(count < microseconds) {
		// 	if(timer_us.isr_flag()){
		// 		count++;
		// 	}
		// }
		// timer_us.disable_cnt();

		// // delay_us(1);
		// pin.write(0);
		// count = 0;
		// timer_us.enable_cnt();
		// timer_us.reset_cnt();
		// while(count < microseconds) {
		// 	if(timer_us.isr_flag()){
		// 		count++;
		// 	}
		// }
		// timer_us.disable_cnt();


		// delay_us(1);

		// if(i > 500000) {
		// 	i = 0;
		// 	printf("A:%lu\n", j++);
		// } else {
		// 	i++;
		// }

		// printf("CNT:%lu\n", i++);

		// delay_ms(200);
	}
}

void test_lcd_aht10_pwm(void) {

	// Control the LCD backlight - channel 2, f = 500 Hz, pwm mode (using PA2, see TIM_Driver docs)
	TIM_Driver pwm(2, 60, timer_mode::pwm_output, 2);

	// set duty cycle to 90 %
	pwm.duty(30);

	// show results
	LCD_Driver lcd0(27, 28, 29, 30, 31, 32);

	// For fetch data indicator only
	GPIO_Driver led0(1, 1);

	// I2C for interface with sensors using port PB10 and PB11
	I2C_Driver i2c(2);
	// i2c.probe_list();

	// Statement of AHT10 sensor
	aht10 sensor0(&i2c);
	sensor0.init(aht10_mode::NORMAL_MODE);

	uint32_t count = 0;
	char str[80];

	while(1) {
		// AHT10 test sensor
		if(sensor0.probe())
		{
			led0.write(0);

			sensor0.trig_meas();
			
			sprintf(str, "T:%.1f°C, H:%.1f %%, %5lu", sensor0.get_temperature(), sensor0.get_humidity(), count);
			printf("%s\n", str);

			sprintf(str, "T %.1f%cC                                H %.1f %%", sensor0.get_temperature(), 0xdf, sensor0.get_humidity());
			lcd0.print(str, 0, 0);

			sprintf(str, "%6lu", count++);
			lcd0.print(str, 1, 10);

			delay_ms(500);
			led0.write(1);
		}
		else {
			i2c.deinit();
			delay_ms(100);
			i2c.reset();
			i2c.init();
		}
		delay_ms(9500);
	}
}

// Example to declare an array of GPIO_Driver class
// #include "gpio"
// GPIO_Driver pin0[] = {
// 	GPIO_Driver{3, 1}, GPIO_Driver{4, 1}, GPIO_Driver{5, 1}, GPIO_Driver{6, 1},
// 	GPIO_Driver{7, 1}, GPIO_Driver{8, 1}, GPIO_Driver{9, 1}, GPIO_Driver{10,1},
// 	GPIO_Driver{11,1}, GPIO_Driver{12,1}, GPIO_Driver{13,1}, GPIO_Driver{14,1},
// 	GPIO_Driver{15,1}, GPIO_Driver{16,1}, GPIO_Driver{17,1}};
