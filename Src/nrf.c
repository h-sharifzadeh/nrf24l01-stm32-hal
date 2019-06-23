# include "nrf.h"


typedef struct {
	uint8_t PayloadSize;				//Payload size
	uint8_t Channel;					//Channel selected
	TM_NRF24L01_OutputPower_t OutPwr;	//Output power
	TM_NRF24L01_DataRate_t DataRate;	//Data rate
} TM_NRF24L01_t;

//------------------------------DEFINES--------------------------------------------------------------
//*** Variables declaration ***//
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x) (((x)>0) ? 1:0)

//*** Library variables ***//
static uint64_t pipe0_reading_address;
static bool ack_payload_available; /**< Whether there is an ack payload waiting */
static uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
static uint8_t payload_size; /**< Fixed size of payloads */
static bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
static bool p_variant; /* False for RF24L01 and true for RF24L01P */
static bool wide_band; /* 2Mbs data rate in use? */

//*** NRF24L01 pins and handles ***//
//CE and CSN pins
static GPIO_TypeDef		*nrf24_PORT;
static uint16_t				nrf24_CSN_PIN;
static uint16_t				nrf24_CE_PIN;
//SPI handle
static SPI_HandleTypeDef nrf24_hspi;
//Debugging UART handle
static UART_HandleTypeDef nrf24_huart;

//**** Functions prototypes ****//
//Microsecond delay function
void NRF24_DelayMicroSeconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
	while(uSecVar--);
}

//1. Chip Select function
void NRF24_csn(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT, nrf24_CSN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CSN_PIN, GPIO_PIN_RESET);
}
//2. Chip Enable
void NRF24_ce(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_RESET);
}
//3. Read single byte from a register
uint8_t NRF24_read_register(uint8_t reg)
{
	uint8_t spiBuf[3];
	uint8_t retData;
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	//Receive data
	HAL_SPI_Receive(&nrf24_hspi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];
	//Bring CSN high
	NRF24_csn(1);
	return retData;
}
//4. Read multiple bytes register
void NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	//spiStatus = NRF24_SPI_Write(spiBuf, 1);
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	//Receive data
	HAL_SPI_Receive(&nrf24_hspi, buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//5. Write single byte register
void NRF24_write_register(uint8_t reg, uint8_t value)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	spiBuf[1] = value;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 2, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//6. Write multipl bytes register
void NRF24_write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t*)buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//7. Write transmit payload
void NRF24_write_payload(const void* buf, uint8_t len)
{
	uint8_t wrPayloadCmd;
	//Bring CSN low
	NRF24_csn(0);
	//Send Write Tx payload command followed by pbuf data
	wrPayloadCmd = CMD_W_TX_PAYLOAD;
	HAL_SPI_Transmit(&nrf24_hspi, &wrPayloadCmd, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t *)buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//8. Read receive payload
void NRF24_read_payload(void* buf, uint8_t len)
{
	uint8_t cmdRxBuf;
	//Get data length using payload size
	uint8_t data_len = MIN(len, NRF24_getPayloadSize());
	//Read data from Rx payload buffer
	NRF24_csn(0);
	cmdRxBuf = CMD_R_RX_PAYLOAD;
	HAL_SPI_Transmit(&nrf24_hspi, &cmdRxBuf, 1, 100);
	HAL_SPI_Receive(&nrf24_hspi, buf, data_len, 100);
	NRF24_csn(1);
}

//9. Flush Tx buffer
void NRF24_flush_tx(void)
{
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
}
//10. Flush Rx buffer
void NRF24_flush_rx(void)
{
	NRF24_write_register(CMD_FLUSH_RX, 0xFF);
}
//11. Get status register value
uint8_t NRF24_get_status(void)
{
	uint8_t statReg;
	statReg = NRF24_read_register(REG_STATUS);
	return statReg;
}

//12. Begin function
void NRF24_begin(GPIO_TypeDef *nrf24PORT, uint16_t nrfCSN_Pin, uint16_t nrfCE_Pin, SPI_HandleTypeDef nrfSPI)
{
	//Copy SPI handle variable
	memcpy(&nrf24_hspi, &nrfSPI, sizeof(nrfSPI));
	//Copy Pins and Port variables
	nrf24_PORT = nrf24PORT;
	nrf24_CSN_PIN = nrfCSN_Pin;
	nrf24_CE_PIN = nrfCE_Pin;
	
	//Put pins to idle state
	NRF24_csn(1);
	NRF24_ce(0);
	//5 ms initial delay
	HAL_Delay(5);
	
	//**** Soft Reset Registers default values ****//
	NRF24_write_register(0x00, 0x08);
	NRF24_write_register(0x01, 0x3f);
	NRF24_write_register(0x02, 0x03);
	NRF24_write_register(0x03, 0x03);
	NRF24_write_register(0x04, 0x03);
	NRF24_write_register(0x05, 0x02);
	NRF24_write_register(0x06, 0x0f);
	NRF24_write_register(0x07, 0x0e);
	NRF24_write_register(0x08, 0x00);
	NRF24_write_register(0x09, 0x00);
	uint8_t pipeAddrVar[6];
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	NRF24_write_registerN(0x0A, pipeAddrVar, 5);
	pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; 
	NRF24_write_registerN(0x0B, pipeAddrVar, 5);
	NRF24_write_register(0x0C, 0xC3);
	NRF24_write_register(0x0D, 0xC4);
	NRF24_write_register(0x0E, 0xC5);
	NRF24_write_register(0x0F, 0xC6);
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	NRF24_write_registerN(0x10, pipeAddrVar, 5);
	NRF24_write_register(0x11, 0);
	NRF24_write_register(0x12, 0);
	NRF24_write_register(0x13, 0);
	NRF24_write_register(0x14, 0);
	NRF24_write_register(0x15, 0);
	NRF24_write_register(0x16, 0);
	
	NRF24_ACTIVATE_cmd();
	NRF24_write_register(0x1c, 0);
	NRF24_write_register(0x1d, 0);
	printRadioSettings();
	//Initialise retries 15 and delay 1250 usec
	NRF24_setRetries(15, 15);
	//Initialise PA level to max (0dB)
	NRF24_setPALevel(RF24_PA_0dB);
	//Initialise data rate to 1Mbps
	NRF24_setDataRate(RF24_2MBPS);
	//Initalise CRC length to 16-bit (2 bytes)
	NRF24_setCRCLength(RF24_CRC_16);
	//Disable dynamic payload
	NRF24_disableDynamicPayloads();
	//Set payload size
	NRF24_setPayloadSize(32);
	
	//Reset status register
	NRF24_resetStatus();
	//Initialise channel to 76
	NRF24_setChannel(76);
	//Flush buffers
	NRF24_flush_tx();
	NRF24_flush_rx();
	
	NRF24_powerDown();
	
}
//13. Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
void NRF24_startListening(void)
{
	//Power up and set to RX mode
	NRF24_write_register(REG_CONFIG, NRF24_read_register(REG_CONFIG) | (1UL<<1) |(1UL <<0));
	//Restore pipe 0 address if exists
	if(pipe0_reading_address)
		NRF24_write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);
	
	//Flush buffers
	NRF24_flush_tx();
	NRF24_flush_rx();
	//Set CE HIGH to start listenning
	NRF24_ce(1);
	//Wait for 130 uSec for the radio to come on
	NRF24_DelayMicroSeconds(150);
}
//14. Stop listening (essential before any write operation)
void NRF24_stopListening(void)
{
	NRF24_ce(0);
	NRF24_flush_tx();
	NRF24_flush_rx();
}
//15. Write(Transmit data), returns true if successfully sent
bool NRF24_write( const void* buf, uint8_t len )
{
	bool retStatus;
	//Start writing
	NRF24_resetStatus();
	NRF24_startWrite(buf,len);
	//Data monitor
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 10; //ms to wait for timeout
	do
  {
    NRF24_read_registerN(REG_OBSERVE_TX,&observe_tx,1);
		//Get status register
		status = NRF24_get_status();
  }
  while( ! ( status & ( _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );
	
//	printConfigReg();
//	printStatusReg();
	
	bool tx_ok, tx_fail;
  NRF24_whatHappened(&tx_ok,&tx_fail, &ack_payload_available);
	retStatus = tx_ok;
	if ( ack_payload_available )
  {
    ack_payload_length = NRF24_getDynamicPayloadSize();
	}
	
	//Power down
	NRF24_available();
	NRF24_flush_tx();
	return retStatus;
}
//16. Check for available data to read
bool NRF24_available(void)
{
	return NRF24_availablePipe(NULL);
}
//17. Read received data
bool NRF24_read( void* buf, uint8_t len )
{
	NRF24_read_payload( buf, len );
	uint8_t rxStatus = NRF24_read_register(REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY);
	NRF24_flush_rx();
	NRF24_getDynamicPayloadSize();
	return rxStatus;
}
//18. Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
void NRF24_openWritingPipe(uint64_t address)
{
	NRF24_write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
  NRF24_write_registerN(REG_TX_ADDR, (uint8_t *)(&address), 5);
	
	const uint8_t max_payload_size = 32;
  NRF24_write_register(REG_RX_PW_P0,MIN(payload_size,max_payload_size));
}
//19. Open reading pipe
void NRF24_openReadingPipe(uint8_t number, uint64_t address)
{
	if (number == 0)
    pipe0_reading_address = address;
	
	if(number <= 6)
	{
		if(number < 2)
		{
			//Address width is 5 bytes
			NRF24_write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 5);
		}
		else
		{
			NRF24_write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 1);
		}
		//Write payload size
		NRF24_write_register(RF24_RX_PW_PIPE[number],payload_size);
		//Enable pipe
		NRF24_write_register(REG_EN_RXADDR, NRF24_read_register(REG_EN_RXADDR) | _BV(number));
	}
	
}
//20 set transmit retries (rf24_Retries_e) and delay
void NRF24_setRetries(uint8_t delay, uint8_t count)
{
	NRF24_write_register(REG_SETUP_RETR,(delay&0xf)<<BIT_ARD | (count&0xf)<<BIT_ARC);
}


void TM_NRF24L01_SetMyAddress(uint8_t *adr) {
	NRF24L01_CE_LOW;
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, adr, 5);
	NRF24L01_CE_HIGH;
}

void TM_NRF24L01_SetTxAddress(uint8_t *adr) {
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, adr, 5);
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, adr, 5);
}

void TM_NRF24L01_WriteBit(uint8_t reg, uint8_t bit, uint8_t value) {
	uint8_t tmp;
	/* Read register */
	tmp = TM_NRF24L01_ReadRegister(reg);
	/* Make operation */
	if (value) {
		tmp |= 1 << bit;
	} else {
		tmp &= ~(1 << bit);
	}
	/* Write back */
	TM_NRF24L01_WriteRegister(reg, tmp);
}

uint8_t TM_NRF24L01_ReadBit(uint8_t reg, uint8_t bit) {
	uint8_t tmp;
	tmp = TM_NRF24L01_ReadRegister(reg);
	if (!NRF24L01_CHECK_BIT(tmp, bit)) {
		return 0;
	}
	return 1;
}

uint8_t TM_NRF24L01_ReadRegister(uint8_t reg) {
	uint8_t value;
	NRF24L01_CSN_LOW;
	TM_SPI_Send(NRF24L01_SPI, NRF24L01_READ_REGISTER_MASK(reg));
	value = TM_SPI_Send(NRF24L01_SPI, NRF24L01_NOP_MASK);
	NRF24L01_CSN_HIGH;
	
	return value;
}

