#include "nRF24L01.h"

/**
 * @brief Creates a delay given in microseconds
 *
 * @param microseconds delay in microseconds
 *
 * @note This function uses timer handle htim1, which is assumed to be
 * 1MHz and count until 65535.
 */
static inline void delay_us(uint16_t microseconds)
{
	__HAL_TIM_SET_COUNTER(&TIMER_HANDLE_1MHZ, 0);
	while (__HAL_TIM_GET_COUNTER(&TIMER_HANDLE_1MHZ) < microseconds);
}

/**
 * @brief Sets CE high
 */
static inline void nrf24_set_CE_high()
{
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, 1);
}

/**
 * @brief Sets CE low
 */
static inline void nrf24_set_CE_low()
{
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, 0);
}

/**
 * @brief Sets CSN high
 */
static inline void nrf24_set_CSN_high()
{
    HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, 1);
}

/**
 * @brief Sets CSN low
 */
static inline void nrf24_set_CSN_low()
{
    HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, 0);
}


HAL_StatusTypeDef nrf24_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg | NRF24_W_REGISTER, data};
    uint8_t status[2] = {0xFF, 0xFF};

    nrf24_set_CSN_low();
    HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(&SPI_HANDLE, buffer, status, 2, SPI_TIMEOUT);
    nrf24_set_CSN_high();

    return retval;
}

HAL_StatusTypeDef nrf24_write_reg_mb(uint8_t reg, uint8_t* data, uint8_t size)
{
	uint8_t cmd = reg | NRF24_W_REGISTER;
    uint8_t status = 0xFF;
    uint8_t response[size];

	nrf24_set_CSN_low();
	HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(&SPI_HANDLE, &cmd, &status, 1, SPI_TIMEOUT);
	if (retval == HAL_OK) {
		retval = HAL_SPI_TransmitReceive(&SPI_HANDLE, data, response, size, SPI_TIMEOUT);
	}
	nrf24_set_CSN_high();

	return retval;
}

uint8_t nrf24_read_reg(uint8_t reg)
{
	// first byte in received data is STATUS and second is the actual data
	uint8_t cmd[2] = {reg, 0x00};
	uint8_t data[2] = {0xFF, 0xFF};

	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive(&SPI_HANDLE, cmd, data, 2, SPI_TIMEOUT);
	nrf24_set_CSN_high();

	return data[1];
}

HAL_StatusTypeDef nrf24_read_reg_mb(uint8_t reg, uint8_t size, uint8_t* output)
{
	uint8_t status = 0xFF;

	nrf24_set_CSN_low();
	HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(&SPI_HANDLE, &reg, &status, 1, SPI_TIMEOUT);
	if (retval == HAL_OK) {
		retval = HAL_SPI_Receive(&SPI_HANDLE, output, size, SPI_TIMEOUT);
	}
	nrf24_set_CSN_high();

	return retval;
}

HAL_StatusTypeDef nrf24_init()
{
	nrf24_set_CE_low();

	// Activate cmd to enable FEATURE register
	uint8_t activate_cmd[2] = {NRF24_ACTIVATE, 0x73};
	HANDLE_SPI_ERROR(HAL_SPI_Transmit(&SPI_HANDLE, activate_cmd, 2, SPI_TIMEOUT));

	// Disable auto-acknowledgment
	HANDLE_SPI_ERROR(nrf24_write_reg(NRF24_EN_AA_REG, 0));

	// Set everything in CONFIG to 0 -> disable CRC, PTX mode, power down
	HANDLE_SPI_ERROR(nrf24_write_reg(NRF24_CONFIG_REG, 0));

	// Disable all data pipes
	HANDLE_SPI_ERROR(nrf24_write_reg(NRF24_EN_RXADDR_REG, 0));
	
	// 5-byte address width
	HANDLE_SPI_ERROR(nrf24_write_reg(NRF24_SETUP_AW_REG, 0b00000011));

	// Disable auto retransmit
	HANDLE_SPI_ERROR(nrf24_write_reg(NRF24_SETUP_RETR_REG, 0));
	
	// Set channel 2 (reset value)
	HANDLE_SPI_ERROR(nrf24_write_reg(NRF24_RF_CH_REG, 2));

	// Data rate 2Mbps, power 0dBm, LNA gain
	HANDLE_SPI_ERROR(nrf24_write_reg(NRF24_RF_SETUP_REG, 0b00001111));

	return HAL_OK;
}

void nrf24_reset()
{
	nrf24_set_CE_low();

	nrf24_write_reg(NRF24_EN_AA_REG, 0x3F);
	nrf24_write_reg(NRF24_CONFIG_REG, 0x08);
	nrf24_write_reg(NRF24_EN_RXADDR_REG, 0x03);
	nrf24_write_reg(NRF24_SETUP_AW_REG, 0x03);
	nrf24_write_reg(NRF24_SETUP_RETR_REG, 0x03);
	nrf24_write_reg(NRF24_RF_CH_REG, 0x02);
	nrf24_write_reg(NRF24_RF_SETUP_REG, 0xF);
	nrf24_write_reg(NRF24_STATUS_REG, 0x00);
	nrf24_write_reg(NRF24_OBSERVE_TX_REG, 0x00);
	nrf24_write_reg(NRF24_CD_REG, 0x00);

	uint8_t rx_addr_p0[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_write_reg_mb(NRF24_RX_ADDR_P0_REG, rx_addr_p0, 5);

	uint8_t rx_addr_p1[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_write_reg_mb(NRF24_RX_ADDR_P1_REG, rx_addr_p1, 5);

	nrf24_write_reg(NRF24_RX_ADDR_P2_REG, 0xC3);
	nrf24_write_reg(NRF24_RX_ADDR_P3_REG, 0xC4);
	nrf24_write_reg(NRF24_RX_ADDR_P4_REG, 0xC5);
	nrf24_write_reg(NRF24_RX_ADDR_P5_REG, 0xC6);

	uint8_t tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_write_reg_mb(NRF24_TX_ADDR_REG, tx_addr, 5);

	nrf24_write_reg(NRF24_RX_PW_P0_REG, 0);
	nrf24_write_reg(NRF24_RX_PW_P1_REG, 0);
	nrf24_write_reg(NRF24_RX_PW_P2_REG, 0);
	nrf24_write_reg(NRF24_RX_PW_P3_REG, 0);
	nrf24_write_reg(NRF24_RX_PW_P4_REG, 0);
	nrf24_write_reg(NRF24_RX_PW_P5_REG, 0);
	nrf24_write_reg(NRF24_FIFO_STATUS_REG, 0x11);
	nrf24_write_reg(NRF24_DYNPD_REG, 0);
	nrf24_write_reg(NRF24_FEATURE_REG, 0);
}

void nrf24_read_state(NRF24_StateTypeDef* state) {
  state->config = nrf24_read_reg(NRF24_CONFIG_REG);
  state->en_aa = nrf24_read_reg(NRF24_EN_AA_REG);
  state->en_rxaddr = nrf24_read_reg(NRF24_EN_RXADDR_REG);
  state->setup_aw = nrf24_read_reg(NRF24_SETUP_AW_REG);
  state->setup_retr = nrf24_read_reg(NRF24_SETUP_RETR_REG);
  state->rf_ch = nrf24_read_reg(NRF24_RF_CH_REG);
  state->rf_setup = nrf24_read_reg(NRF24_RF_SETUP_REG);
  state->observe_tx = nrf24_read_reg(NRF24_OBSERVE_TX_REG);
  state->cd = nrf24_read_reg(NRF24_CD_REG);

  // Read RX addresses for pipes 0 to 5
  nrf24_read_reg_mb(NRF24_RX_ADDR_P0_REG, 5, state->rx_addr_p0);
  nrf24_read_reg_mb(NRF24_RX_ADDR_P1_REG, 5, state->rx_addr_p1);
  state->rx_addr_p2 = nrf24_read_reg(NRF24_RX_ADDR_P2_REG);
  state->rx_addr_p3 = nrf24_read_reg(NRF24_RX_ADDR_P3_REG);
  state->rx_addr_p4 = nrf24_read_reg(NRF24_RX_ADDR_P4_REG);
  state->rx_addr_p5 = nrf24_read_reg(NRF24_RX_ADDR_P5_REG);

  // Read TX address (5 bytes)
  nrf24_read_reg_mb(NRF24_TX_ADDR_REG, 5, state->tx_addr);

  // Read RX payload widths for pipes 0 to 5
  nrf24_read_reg_mb(NRF24_RX_PW_P0_REG, 6, state->rx_pw_px);
  // Read remaining registers
  state->fifo_status = nrf24_read_reg(NRF24_FIFO_STATUS_REG);
  state->dynpd = nrf24_read_reg(NRF24_DYNPD_REG);
  state->feature = nrf24_read_reg(NRF24_FEATURE_REG);
  state->status = nrf24_read_reg(NRF24_STATUS_REG);
}

void nrf24_set_tx_mode(uint8_t channel, uint8_t* address)
{
	if (channel > 127) {
		return;
	}
	nrf24_set_CE_low();
	nrf24_write_reg(NRF24_RF_CH_REG, channel);
	nrf24_write_reg_mb(NRF24_TX_ADDR_REG, address, 5);
	uint8_t config = nrf24_read_reg(NRF24_CONFIG_REG);
	config &= 0b11111110;		// ptx mode
	config |= 0b10;					// power up
	config |= 0b1010000; 		// disable RX_DR and MAX_RT interrupts
	nrf24_write_reg(NRF24_CONFIG_REG, config);
	HAL_Delay(2);
}

void nrf24_set_rx_mode(uint8_t channel, uint8_t* address, uint8_t pipe)
{
	if (channel > 127 || pipe > 5) {
		return;
	}

	nrf24_set_CE_low();

	uint8_t current_pipes = nrf24_read_reg(NRF24_EN_RXADDR_REG);
	nrf24_write_reg(NRF24_EN_RXADDR_REG, current_pipes | (1 << pipe));

	uint8_t pipe_addr_reg = NRF24_RX_ADDR_P0_REG + pipe;
	if (pipe < 3) {
		// TODO: handle different address width than 5 bytes
		nrf24_write_reg_mb(pipe_addr_reg, address, 5);
	}
	else {
		// if pipe is 3-5, only the first byte of address will be written
		nrf24_write_reg(pipe_addr_reg, address[0]);
	}

	uint8_t pipe_size_reg = NRF24_RX_PW_P0_REG + pipe;
	// TODO: handle dynamic payload length
	nrf24_write_reg(pipe_size_reg, 32);

	nrf24_write_reg(NRF24_RF_CH_REG, channel); // set channel

	uint8_t config = nrf24_read_reg(NRF24_CONFIG_REG);
	config |= 0b110000; // disable TX_DS and MAX_RT interrupts
	config |= 0b10;  // power up
	nrf24_write_reg(NRF24_CONFIG_REG, config);
	HAL_Delay(2);
	config |= 0b11; // PRX mode
	nrf24_write_reg(NRF24_CONFIG_REG, config);
	while (SPI_HANDLE.State != HAL_SPI_STATE_READY);
	nrf24_set_CE_high();
	delay_us(130);
}

void nrf24_transmit(uint8_t* data, uint8_t size)
{
	uint8_t cmd = NRF24_W_TX_PAYLOAD;
	uint8_t status = 0;

	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive_IT(&SPI_HANDLE, &cmd, &status, 1);
	while (SPI_HANDLE.State != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit_IT(&SPI_HANDLE, data, size);
	while (SPI_HANDLE.State != HAL_SPI_STATE_READY);
	nrf24_set_CSN_high();

	nrf24_set_CE_high();
	delay_us(30);
	nrf24_set_CE_low();

	delay_us(400); // PLL lock + TX delay + 100 us

	/* Flush TX FIFO if it's not empty */
	uint8_t fifo_status = nrf24_read_reg(NRF24_FIFO_STATUS_REG);
	if ( !((fifo_status >> 4) & 1) ) {
		nrf24_send_command(NRF24_FLUSH_TX);
	}

	nrf24_write_reg(NRF24_STATUS_REG, status |= (1<<5)); // clear interrupt
}

uint8_t nrf24_receive(uint8_t pipe, uint8_t* data, uint8_t size)
{
	uint8_t status = 0;

	// Check if received pipe is correct
	if (((status >> 1) & 0x7) != pipe) {
		return 0;
	}

	uint8_t fifo_status = nrf24_read_reg(NRF24_FIFO_STATUS_REG);
	// Check if rx fifo empty
	if (fifo_status & 1) {
		return 0;
	}

	uint8_t cmd = NRF24_R_RX_PAYLOAD;

	while (!(fifo_status & 1))
	{
		nrf24_set_CSN_low();
		HAL_SPI_TransmitReceive_IT(&SPI_HANDLE, &cmd, &status, 1);
		while (SPI_HANDLE.State != HAL_SPI_STATE_READY);
		HAL_SPI_Receive_IT(&SPI_HANDLE, data, size);
		while (SPI_HANDLE.State != HAL_SPI_STATE_READY);
		nrf24_set_CSN_high();
		nrf24_write_reg(NRF24_STATUS_REG, status |= (1<<6)); // clear interrupt
		fifo_status = nrf24_read_reg(NRF24_FIFO_STATUS_REG);
	}

	return 1;
}

void nrf24_send_command(uint8_t cmd)
{
	uint8_t response = 0xFF;
	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive(&SPI_HANDLE, &cmd, &response, 1, SPI_TIMEOUT);
	nrf24_set_CSN_high();
}

void nrf24_open_rx_pipe(uint8_t pipe)
{
	if (pipe > 5) {
		return;
	}
	uint8_t current_config = nrf24_read_reg(NRF24_EN_RXADDR_REG);

	nrf24_write_reg(NRF24_EN_RXADDR_REG , current_config | (1 << pipe));
}

void nrf24_set_pa_level(NRF24_PA_Level level)
{
	uint8_t current_rf_setup = nrf24_read_reg(NRF24_RF_SETUP_REG);
	nrf24_write_reg(NRF24_RF_SETUP_REG, current_rf_setup | level);
}
