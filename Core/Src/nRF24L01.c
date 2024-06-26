#include "nRF24L01.h"


void nrf24_set_CE_high()
{
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, 1);
}

void nrf24_set_CE_low()
{
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, 0);
}

void nrf24_set_CSN_high()
{
    HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, 1);
}

void nrf24_set_CSN_low()
{
    HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, 0);
}

void delay_us(uint16_t microseconds)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < microseconds);
}

HAL_StatusTypeDef nrf24_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg | NRF24_W_REGISTER, data};
    uint8_t status[2] = {0xFF, 0xFF};

    nrf24_set_CSN_low();
    HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(&hspi1, buffer, status, 2, 100);
    nrf24_set_CSN_high();

    return retval;
}

HAL_StatusTypeDef nrf24_write_reg_mb(uint8_t reg, uint8_t* data, uint8_t size)
{
	uint8_t cmd = reg | NRF24_W_REGISTER;
    uint8_t status = 0xFF;
    uint8_t response[size];

	nrf24_set_CSN_low();
	HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, 100);
	if (retval == HAL_OK) {
		retval = HAL_SPI_TransmitReceive(&hspi1, data, response, size, 200);
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
	HAL_SPI_TransmitReceive(&hspi1, cmd, data, 2, 100);
	nrf24_set_CSN_high();

	return data[1];
}

HAL_StatusTypeDef nrf24_read_reg_mb(uint8_t reg, uint8_t size, uint8_t* output)
{
	uint8_t status = 0xFF;

	nrf24_set_CSN_low();
	HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(&hspi1, &reg, &status, 1, 100);
	if (retval == HAL_OK) {
		retval = HAL_SPI_Receive(&hspi1, output, size, 200);
	}
	nrf24_set_CSN_high();

	return retval;
}

HAL_StatusTypeDef nrf24_init()
{
	nrf24_set_CE_low();

	// Activate cmd to enable FEATURE register
	uint8_t activate_cmd[2] = {NRF24_ACTIVATE, 0x73};
	HANDLE_SPI_ERROR(HAL_SPI_Transmit(&hspi1, activate_cmd, 2, 100));

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

	// Data rate 2Mbps, power 0dBm
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
	config &= 0b11111110;  // ptx mode
	config |= 0b10;  // power up
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


	uint8_t config = nrf24_read_reg(NRF24_CONFIG_REG);
	config |= 0b11;  // prx mode and power up
	nrf24_write_reg(NRF24_CONFIG_REG, config);
	HAL_Delay(2);
	//nrf24_set_CE_high();
}

HAL_StatusTypeDef nrf24_transmit(uint8_t* data, uint8_t size)
{
	uint8_t cmd = NRF24_W_TX_PAYLOAD;
	uint8_t tmp[32];

	nrf24_set_CSN_low();
	HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&hspi1, &cmd, tmp, 1, 100);
	while (hspi1.State != HAL_SPI_STATE_READY);
	if (spi_status == HAL_OK) {
		spi_status = HAL_SPI_TransmitReceive(&hspi1, data, tmp, size, 200);
		while (hspi1.State != HAL_SPI_STATE_READY);
	}
	nrf24_set_CSN_high();

	/* Check if fifo full debug (it never is)
	uint8_t fifo_status = nrf24_read_reg(NRF24_FIFO_STATUS_REG);
	if ( (fifo_status >> 6) & 0b1 ) {
		HAL_Delay(1);
	}

	if (spi_status != HAL_OK) {
		nrf24_flush_tx_fifo();
		return spi_status;
	}
	*/

	nrf24_set_CE_high();
	delay_us(30);
	nrf24_set_CE_low();

	delay_us(400); // PLL lock + TX delay + 100 us

	/*
	uint8_t fifo_status = nrf24_read_reg(NRF24_FIFO_STATUS_REG);
	// flush tx fifo if it's not empty
	if ( !((fifo_status >> 4) & 0b1) ) {
		nrf24_flush_tx_fifo();
	}
	*/

	return spi_status;
}

void nrf24_send_command(uint8_t cmd)
{
	uint8_t response = 0xFF;
	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive(&hspi1, &cmd, &response, 1, 100);
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

void nrf24_enable_dyn_payload_len()
{
	uint8_t current_feature_reg = nrf24_read_reg(NRF24_FEATURE_REG);
	nrf24_write_reg(NRF24_FEATURE_REG, (current_feature_reg | 0b100));
}

void nrf24_flush_tx_fifo() {
	nrf24_send_command(NRF24_FLUSH_TX);
}
