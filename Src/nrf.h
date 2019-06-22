//******************************************INCLUDE*************************************

#include "main.h"
#include "stm32f4xx_hal.h"

//*******************************************DEFINES*************************************

//Change SPI used. Refer to TM SPI library to check which pins are for SPI
#define NRF24L01_SPI				SPI3
#define NRF24L01_SPI_PINS			TM_SPI_PinsPack_2

//Change CSN pin. This is for SPI communication
#define NRF24L01_CSN_PORT			GPIOD
#define NRF24L01_CSN_PIN			GPIO_Pin_7

//Change CE pin. This pin is used to enable/disable transmitter/receiver functionality
#define NRF24L01_CE_PORT			GPIOD
#define NRF24L01_CE_PIN				GPIO_Pin_8

/* Default SPI used */
#ifndef NRF24L01_SPI
#define NRF24L01_SPI				SPI3
#define NRF24L01_SPI_PINS			TM_SPI_PinsPack_2
#endif

/* SPI chip enable pin */
#ifndef NRF24L01_CSN_PIN
#define NRF24L01_CSN_PORT			GPIOD
#define NRF24L01_CSN_PIN			GPIO_PIN_7
#endif

/* Chip enable for transmitting */
#ifndef NRF24L01_CE_PIN
#define NRF24L01_CE_PORT			GPIOD
#define NRF24L01_CE_PIN				GPIO_PIN_8
#endif

/* Pins configuration */
#define NRF24L01_CE_LOW				HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN , RESET)
#define NRF24L01_CE_HIGH			HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN , SET)
#define NRF24L01_CSN_LOW			HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN , RESET)
#define NRF24L01_CSN_HIGH			HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN , SET)

/* Interrupt masks */
#define NRF24L01_IRQ_DATA_READY     0x40 /*!< Data ready for receive */
#define NRF24L01_IRQ_TRAN_OK        0x20 /*!< Transmission went OK */
#define NRF24L01_IRQ_MAX_RT         0x10 /*!< Max retransmissions reached, last transmission failed */

//**********************************************************************************

typedef union _TM_NRF24L01_IRQ_t {
	struct {
		uint8_t reserved0:4;
		uint8_t MaxRT:1;     /*!< Set to 1 if MAX retransmissions flag is set */
		uint8_t DataSent:1;  /*!< Set to 1 if last transmission is OK */
		uint8_t DataReady:1; /*!< Set to 1 if data are ready to be read */
		uint8_t reserved1:1;
	} F;
	uint8_t Status;          /*!< NRF status register value */
} TM_NRF24L01_IRQ_t;

typedef enum _TM_NRF24L01_Transmit_Status_t {
	TM_NRF24L01_Transmit_Status_Lost = 0x00,   /*!< Message is lost, reached maximum number of retransmissions */
	TM_NRF24L01_Transmit_Status_Ok = 0x01,     /*!< Message sent successfully */
	TM_NRF24L01_Transmit_Status_Sending = 0xFF /*!< Message is still sending */
} TM_NRF24L01_Transmit_Status_t;

typedef enum {
	TM_GPIO_Mode_IN = 0x00,  /*!< GPIO Pin as General Purpose Input */
	TM_GPIO_Mode_OUT = 0x01, /*!< GPIO Pin as General Purpose Output */
	TM_GPIO_Mode_AF = 0x02,  /*!< GPIO Pin as Alternate Function */
	TM_GPIO_Mode_AN = 0x03,  /*!< GPIO Pin as Analog input/output */
} TM_GPIO_Mode_t;

typedef enum {
	TM_GPIO_OType_PP = 0x00, /*!< GPIO Output Type Push-Pull */
	TM_GPIO_OType_OD = 0x01  /*!< GPIO Output Type Open-Drain */
} TM_GPIO_OType_t;

typedef enum {
	TM_GPIO_PuPd_NOPULL = 0x00, /*!< No pull resistor */
	TM_GPIO_PuPd_UP = 0x01,     /*!< Pull up resistor enabled */
	TM_GPIO_PuPd_DOWN = 0x02    /*!< Pull down resistor enabled */
} TM_GPIO_PuPd_t;

typedef enum {
	TM_GPIO_Speed_Low = 0x00,    /*!< GPIO Speed Low */
	TM_GPIO_Speed_Medium = 0x01, /*!< GPIO Speed Medium */
	TM_GPIO_Speed_Fast = 0x02,   /*!< GPIO Speed Fast, not available on STM32F0xx devices */
	TM_GPIO_Speed_High = 0x03    /*!< GPIO Speed High */
} TM_GPIO_Speed_t;
//-------------------------------maybe unnecessary--------------------------------------
/**
 * @brief  Data rate enumeration
 */
typedef enum _TM_NRF24L01_DataRate_t {
	TM_NRF24L01_DataRate_2M = 0x00, /*!< Data rate set to 2Mbps */
	TM_NRF24L01_DataRate_1M,        /*!< Data rate set to 1Mbps */
	TM_NRF24L01_DataRate_250k       /*!< Data rate set to 250kbps */
} TM_NRF24L01_DataRate_t;

/**
 * @brief  Output power enumeration
 */
typedef enum _TM_NRF24L01_OutputPower_t {
	TM_NRF24L01_OutputPower_M18dBm = 0x00,/*!< Output power set to -18dBm */
	TM_NRF24L01_OutputPower_M12dBm,       /*!< Output power set to -12dBm */
	TM_NRF24L01_OutputPower_M6dBm,        /*!< Output power set to -6dBm */
	TM_NRF24L01_OutputPower_0dBm          /*!< Output power set to 0dBm */
} TM_NRF24L01_OutputPower_t;

//*****************************************FUNCTIONS******************************************

/**
 * @brief  Initializes NRF24L01+ module
 * @param  channel: channel you will use for communication, from 0 to 125 eg. working frequency from 2.4 to 2.525 GHz
 * @param  payload_size: maximum data to be sent in one packet from one NRF to another.
 * @note   Maximal payload size is 32bytes
 * @retval 1
 */
uint8_t TM_NRF24L01_Init(uint8_t channel, uint8_t payload_size);

/**
 * @brief  Sets own address. This is used for settings own id when communication with other modules
 * @note   "Own" address of one device must be the same as "TX" address of other device (and vice versa),
 *         if you want to get successful communication
 * @param  *adr: Pointer to 5-bytes length array with address
 * @retval None
 */
void TM_NRF24L01_SetMyAddress(uint8_t* adr);

/**
 * @brief  Sets address you will communicate with
 * @note   "Own" address of one device must be the same as "TX" address of other device (and vice versa),
 *         if you want to get successful communication
 * @param  *adr: Pointer to 5-bytes length array with address
 * @retval None
 */
void TM_NRF24L01_SetTxAddress(uint8_t* adr);

/**
 * @brief  Sets NRF24L01+ to TX mode
 * @note   In this mode is NRF able to send data to another NRF module
 * @param  None
 * @retval None
 */
void TM_NRF24L01_PowerUpTx(void);

/**
 * @brief  Sets NRF24L01+ to RX mode
 * @note   In this mode is NRF able to receive data from another NRF module.
 *         This is default mode and should be used all the time, except when sending data
 * @param  None
 * @retval None
 */
void TM_NRF24L01_PowerUpRx(void);

/**
 * @brief  Gets transmissions status
 * @param  None
 * @retval Transmission status. Return is based on @ref TM_NRF24L01_Transmit_Status_t enumeration
 */
TM_NRF24L01_Transmit_Status_t TM_NRF24L01_GetTransmissionStatus(void);

/**
 * @brief  Transmits data with NRF24L01+ to another NRF module
 * @param  *data: Pointer to 8-bit array with data.
 *         Maximum length of array can be the same as "payload_size" parameter on initialization
 * @retval None
 */
void TM_NRF24L01_Transmit(uint8_t *data);

/**
 * @brief  Checks if data is ready to be read from NRF24L01+
 * @param  None
 * @retval Data ready status:
 *            - 0: No data available for receive in bufferReturns
 *            - > 0: Data is ready to be collected
 */
uint8_t TM_NRF24L01_DataReady(void);

/**
 * @brief  Gets data from NRF24L01+
 * @param  *data: Pointer to 8-bits array where data from NRF will be saved
 * @retval None
 */
void TM_NRF24L01_GetData(uint8_t *data);

/**
 * @brief  Sets working channel
 * @note   Channel value is just an offset in units MHz from 2.4GHz
 *         For example, if you select channel 65, then operation frequency will be set to 2.465GHz.
 * @param  channel: RF channel where device will operate
 * @retval None 
 */
void TM_NRF24L01_SetChannel(uint8_t channel);

/**
 * @brief  Gets NRLF+ status register value
 * @param  None
 * @retval Status register from NRF
 */
uint8_t TM_NRF24L01_GetStatus(void);

//----------------------------------------------unnecessory-functions-----------------------------------

/* Private */
void TM_NRF24L01_WriteRegister(uint8_t reg, uint8_t value);
