#include <stdio.h>
#include "system_main.h"

#include "i2c_master.hpp"
#include "aht10.hpp"
#include "pcy8575.hpp"

#include "gpio.hpp"
#include "i2c.h"

enum class states {
	receiver = 0,
	transmitter
};

states state_fsm = states::receiver;

// IMplement class vector for GPIO into pcy8575

void i2c_slave_FSM(void) {

	// GPIO_driver pin0[] = {
	// 	GPIO_driver{3, 1}, GPIO_driver{4, 1}, GPIO_driver{5, 1}, GPIO_driver{6, 1},
	// 	GPIO_driver{7, 1}, GPIO_driver{8, 1}, GPIO_driver{9, 1}, GPIO_driver{10,1},
	// 	GPIO_driver{11,1}, GPIO_driver{12,1}, GPIO_driver{13,1}, GPIO_driver{14,1},
	// 	GPIO_driver{15,1}, GPIO_driver{16,1}, GPIO_driver{17,1}};

	pcy8575 extender0;
	extender0.init();

	// uint16_t value = (1<<11);
	while(1) {

		extender0.handle_message();

		// extender0.put(value);
		// // extender0.test_up();
		// HAL_Delay(500);

		// extender0.put(~value);
		// // extender0.test_down();
		// HAL_Delay(500);
		// printf("Ola!\n");
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

int main(void)
{
	init_system();

	// aht10_test();
	i2c_slave_FSM();

    return 0;
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