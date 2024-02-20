#include "gpio_driver.h"

GPIO_DRIVER::GPIO_DRIVER(int pin_number, int direction) : direction_(direction){
	
	GPIO_InitTypeDef GPIO_InitStruct;// = {0};

	if(direction_) {
		// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// Push pull
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	// open drain
		// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //GPIO_SPEED_FREQ_LOW;
	}
	else {
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	}

	switch (pin_number) {
		case 1:
			__HAL_RCC_GPIOC_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 13);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOC;
			break;

		case 2:
			__HAL_RCC_GPIOC_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 14);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOC;
			break;

		case 3:
			__HAL_RCC_GPIOC_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 15);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOC;
			break;

		case 4:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 0);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 5:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 1);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 6:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 2);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 7:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 3);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 8:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 4);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 9:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 5);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 10:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 6);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 11:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 7);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 12:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 0);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 13:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 1);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 14:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 10);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 15:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 11);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 16:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 12);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 17:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 13);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 18:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 14);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 19:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 15);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 20:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 8);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 21:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 9);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 22:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 10);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 23:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 11);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 24:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 12);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 25:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 15);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOA;
			break;

		case 26:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 3);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 27:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 4);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 28:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 5);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 29:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 6);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 30:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 7);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 31:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 8);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		case 32:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			gpio_pin_mask_ = (0x0001 << 9);
			GPIO_InitStruct.Pin = gpio_pin_mask_;
			port_ = GPIOB;
			break;

		// default:
			// printf("GPIO: pin not found!\n");
			// break;


		// case 37:
		// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
		// 	break;

	}
	
	HAL_GPIO_Init(port_, &GPIO_InitStruct);
	
	if(direction_) {
		write(0);
	}
	// this->num = num;
	// gpio_set_direction(num,mode);
}
GPIO_DRIVER::~GPIO_DRIVER() {
	// unregister_interrupt();
	// void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
}
void GPIO_DRIVER::mode(int direction) {

	GPIO_InitTypeDef GPIO_InitStruct;// = {0};

	if(direction)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //GPIO_SPEED_FREQ_LOW;
	}
	else
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	}

	GPIO_InitStruct.Pin = gpio_pin_mask_;

	HAL_GPIO_Init(port_, &GPIO_InitStruct);
}
int GPIO_DRIVER::read(void) {

	// GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
	if(direction_) {
		// read the ODR output register
		if(port_->ODR & gpio_pin_mask_)
			level_ = 1;
		else
			level_ = 0;
	} else{
		// read the IDR input register
		level_ = static_cast<int>(HAL_GPIO_ReadPin(port_, gpio_pin_mask_));
	}

	// level = gpio_get_level(num);
	return level_;
}
void GPIO_DRIVER::write(int level) {
	/*Configure GPIO pin Output Level into STM32*/
	level_ = level;
	HAL_GPIO_WritePin(port_, gpio_pin_mask_, static_cast<GPIO_PinState>(level_));
	// gpio_set_level(num, level);
}
void GPIO_DRIVER::toggle(void) {
	level_ = !level_;
	// write(level);
	HAL_GPIO_TogglePin(port_, gpio_pin_mask_);
}
void GPIO_DRIVER::reset(void) noexcept
{
	// gpio_reset_pin(num);
}
// void GPIO_DRIVER::pull(gpio_pull_mode_t mode){
// 	// gpio_set_pull_mode(num, mode);
// }
// void GPIO_DRIVER::strength(gpio_drive_cap_t cap){
// 	// gpio_set_drive_capability(num, cap);
// }
// void GPIO_DRIVER::hold(bool hold){
// 	// if(hold){
// 	// 	gpio_hold_en(num);
// 	// } else {
// 	// 	gpio_hold_dis(num);
// 	// }
// }
// void GPIO_DRIVER::deep_sleep_hold(bool hold){
// 	// if(hold){
// 	// 	gpio_deep_sleep_hold_en();
// 	// } else {
// 	// 	gpio_deep_sleep_hold_dis();
// 	// }
// }
// void GPIO_DRIVER::register_interrupt(gpio_isr_t handler, void* isr_args){
// 	// if(driver_instaled == 0){
// 	// 	gpio_install_isr_service(GPIO_INTERRUPT_FLAGS);
// 	// 	driver_instaled++;
// 	// }

// 	// this->isr_args = isr_args;
// 	// gpio_isr_handler_add(num, handler,this);
// }
// void GPIO_DRIVER::unregister_interrupt(){
// 	// if(driver_instaled == 0){
// 	// 	return;
// 	// }

// 	// gpio_isr_handler_remove(num);
// 	// disable_interrupt();

// 	// if(--driver_instaled){
// 	// 	gpio_uninstall_isr_service();
// 	// }
// }
// void GPIO_DRIVER::enable_interrupt(gpio_int_type_t type){
// 	// gpio_set_intr_type(num, type);
// 	// gpio_intr_enable(num);
// }
// void GPIO_DRIVER::disable_interrupt(){
// 	// gpio_intr_disable(num);
// }
// void* GPIO_DRIVER::get_isr_args(){
// 	// return isr_args;
// }
