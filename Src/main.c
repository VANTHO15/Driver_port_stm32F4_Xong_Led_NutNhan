

#include <stdint.h>
#include <stm32f4.h>
#include "stm32_gpio.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int a =0;

int main(void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);
	GPIO_WriteToOutputPin(GPIOC,GPIO_PIN_NO_11,GPIO_PIN_RESET);

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&GPIOBtn);
	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 3);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);



	while(1)
	{

	}
	return 0;

}

void EXTI15_10_IRQHandler()
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_11);
	GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_11);
	a++;
}
