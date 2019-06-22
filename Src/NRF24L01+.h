/* Define to prevent recursive inclusion -------------------------------*/
#ifndef __NRF24L01_H__ 
#define __NRF24L01_H__ 
/* Includes ------------------------------------------------------------*/ 
#include "stm32f4xx.h" 
#include "stm32f4xx_hal_gpio.h"

typedef struct
{
	uint8_t CSN ; 
	uint8_t CE ;
	uint8_t IRQ ;
} nrf_pins; 
typedef struct
{
	uint8_t MOSI;
	uint8_t MISO;
	uint8_t SS;
}spi_pins;

#endif