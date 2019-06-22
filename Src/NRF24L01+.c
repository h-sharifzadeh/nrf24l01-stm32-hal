#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "NRF24L01+.h"

nrf_pins nrf1;


void NRF24L01_InitPins(void) {
	HAL_GPIO_WritePin( GPIOB ,nrf1.CE , 0);    // CE low = disable TX/RX
	HAL_GPIO_WritePin( GPIOB ,nrf1.CSN , 1);  // CSN high = disable SPI 
}
uint8_t TM_NRF24L01_Init(uint8_t channel, uint8_t payload_size,uint8_t PA_control,uint8_t channel_frequency){
	NRF24L01_InitPins();
	
	
	
