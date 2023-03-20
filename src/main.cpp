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


void aht10_test(void);
void i2c_slave_pcy8575(void);
void adc_test(void);

int main(void)
{
	init_system();
	printf("\nSystem reset...\n");

	// aht10_test();
	// i2c_slave_pcy8575();
	adc_test();

    return 0;
}

// Example to declare an array of GPIO_driver class
// #include "gpio"
// GPIO_driver pin0[] = {
// 	GPIO_driver{3, 1}, GPIO_driver{4, 1}, GPIO_driver{5, 1}, GPIO_driver{6, 1},
// 	GPIO_driver{7, 1}, GPIO_driver{8, 1}, GPIO_driver{9, 1}, GPIO_driver{10,1},
// 	GPIO_driver{11,1}, GPIO_driver{12,1}, GPIO_driver{13,1}, GPIO_driver{14,1},
// 	GPIO_driver{15,1}, GPIO_driver{16,1}, GPIO_driver{17,1}};

#define ADC_BUFLEN 10
volatile uint8_t adc_dma_flag = 0;

// extern "C" {
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
void adc_test(void) {
	ADC_driver adc0;
	adc0.init();

	adc_dma_init();
	
	uint32_t adc_raw_data[6];

	tim3_init();
	// tim2_init();

	// adc_set_CR2_EXTSEL_bits(0x04);
	// adc_set_CR2_DMA_bit();

	adc_read_SR_reg();
	adc_print_SR_reg();
	adc_read_CR1_reg();
	adc_print_CR1_reg();
	adc_read_CR2_reg();
	adc_print_CR2_reg();

	uint32_t adc_buffer[ADC_BUFLEN];   //For ADC samples.
	memset(adc_buffer, 0, sizeof(adc_buffer));
	for(int j=0; j<ADC_BUFLEN; j++) {
		printf("ADC[%d]= %lu\n", j, adc_buffer[j]);
	}
	HAL_ADC_Start_DMA(&hadc1, &adc_buffer[0], ADC_BUFLEN); //Link DMA to ADC1
	// adc0.read_stream(&adc_buffer[0], ADC_BUFLEN);

	// volatile int i = 0;
	while(1) {

		// 1 second flag to refresh watchdog timer
		if(tim3_flag_1sec) {
			tim3_flag_1sec = 0;
			// printf("TIM3\n");
			adc_read_SR_reg();
			adc_print_SR_reg();
			adc_read_CR1_reg();
			adc_print_CR1_reg();
			adc_read_CR2_reg();
			adc_print_CR2_reg();

			if(adc_read_SR_EOC_bit()) {
				printf("ADC1->DR: %lu\n", ADC1->DR);
			}
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
			// i++;
			for(int j=0; j<ADC_BUFLEN; j++) {
				printf("ADC[%d]= %lu\n", j, adc_buffer[j]);
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







// #define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
// /* Size of Transmission buffer */
// #define TXBUFFERSIZE                      (COUNTOF(aTxBuffer))
// /* Size of Reception buffer */
// #define RXBUFFERSIZE                      TXBUFFERSIZE
// __IO uint32_t     Transfer_Direction = 0;
// __IO uint32_t     Xfer_Complete = 0;
// static I2C_Master i2c;
// uint8_t aTxBuffer[4]; /* Buffer used for reception */
// uint8_t aRxBuffer[4]; // 
// void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
// {
// /* Toggle LED4: Transfer in transmission process is correct */
// 	Xfer_Complete = 1;
// 	aTxBuffer[0]++;
// 	aTxBuffer[1]++;
// 	aTxBuffer[2]++;
// 	aTxBuffer[3]++;
// 	printf("HAL_I2C_SlaveTxCpltCallback\n");
// }
// /**
//   * @brief  Rx Transfer completed callback.
//   * @param  I2cHandle: I2C handle
//   * @note   This example shows a simple way to report end of IT Rx transfer, and
//   *         you can add your own implementation.
//   * @retval None
//   */
// void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
// {
// 	/* Toggle LED4: Transfer in reception process is correct */
// 	Xfer_Complete = 1;
// 	aRxBuffer[0]=0x00;
// 	aRxBuffer[1]=0x00;
// 	aRxBuffer[2]=0x00;
// 	aRxBuffer[3]=0x00;
// 	printf("HAL_I2C_SlaveRxCpltCallback\n");
// }
// /**
//   * @brief  Slave Address Match callback.
//   * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
//   *                the configuration information for the specified I2C.
//   * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
//   * @param  AddrMatchCode: Address Match Code
//   * @retval None
//   */
// void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
// {
// 	printf("ADDR MATCH!\n");
// 	Transfer_Direction = TransferDirection;
// 	if (Transfer_Direction != 0)
// 	{
// 		/*##- Start the transmission process #####################################*/
// 		/* While the I2C in reception process, user can transmit data through
// 		"aTxBuffer" buffer */
// 		if (HAL_I2C_Slave_Seq_Transmit_IT(&i2c.hi2c2_, (uint8_t *)aTxBuffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
// 		{
// 			/* Transfer error in transmission process */
// 			// Error_Handler();
// 		}
// 	}
// 	else
// 	{
// 		/*##- Put I2C peripheral in reception process ###########################*/
// 		if (HAL_I2C_Slave_Seq_Receive_IT(&i2c.hi2c2_, (uint8_t *)aRxBuffer, RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
// 		{
// 			/* Transfer error in reception process */
// 			// Error_Handler();
// 		}
// 	}
// }
// /**
//   * @brief  Listen Complete callback.
//   * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
//   *                the configuration information for the specified I2C.
//   * @retval None
//   */
// void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
// {
// 	printf("HAL_I2C_ListenCpltCallback\n");
// }
// /**
//   * @brief  I2C error callbacks.
//   * @param  I2cHandle: I2C handle
//   * @note   This example shows a simple way to report transfer error, and you can
//   *         add your own implementation.
//   * @retval None
//   */
// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
// {
// 	/** Error_Handler() function is called when error occurs.
// 	* 1- When Slave doesn't acknowledge its address, Master restarts communication.
// 	* 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
// 	*/
// 	printf("HAL_I2C_ErrorCallback\n");
// 	if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
// 	{
// 		Error_Handler();
// 	}
// }
// void i2c_slave(void) {

// 	i2c.init(I2C_NORMAL_SPEED_HZ);

// 	i2c.listen_enable();

// 	int count = 0;

// 	while(1)
// 	{

// 		HAL_Delay(1000);
// 		printf("Count: %d\n", count++);
		

// 		if(__HAL_I2C_GET_FLAG(&i2c.hi2c2_, I2C_FLAG_ADDR) == FlagStatus::SET)
// 		{
// 			printf("ADD Match!\n");
// 			// printf("Direction: %d\n", __HAL_I2C_GET_FLAG(&i2c.hi2c2_, I2C_FLAG_TRA));
// 			aTxBuffer[0] = 0xAA;
// 			aTxBuffer[1] = 0xBB;
// 			aTxBuffer[2] = 0xCC;
// 			aTxBuffer[3] = 0xDD;
// 			// HAL_I2C_Slave_Receive(&hi2c2_, &data_rx[0], 2, 1000)
// 			// if(HAL_I2C_Slave_Seq_Transmit_IT(&i2c.hi2c2_, (uint8_t *)aTxBuffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
// 			if(HAL_I2C_Slave_Seq_Transmit_IT(&i2c.hi2c2_, (uint8_t *)aTxBuffer, 4, I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
// 				printf("Error sending back\n");
// 			}
// 			else
// 				printf("Sent!\n");
// 		}

// 		// if (Xfer_Complete ==1)
// 		// {
// 		// 	// HAL_Delay(1);
// 		// 	/*##- Put I2C peripheral in listen mode process ###########################*/
// 		// 	i2c.listen();
// 		// 	// if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
// 		// 	// {
// 		// 	// /* Transfer error in reception process */
// 		// 	// Error_Handler();
// 		// 	// }
// 		// 	Xfer_Complete =0;
// 		// }

// 	// 	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
// 	// 	HAL_Delay(100);
// 	}
// }