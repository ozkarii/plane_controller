#include "nRF24L01.h"

/*
 * try different data rate
 * add delays
 * try different spi clocks
 * move caps closer
 *
 * */

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

void nrf24_write_reg(SPI_HandleTypeDef* handle, uint8_t reg, uint8_t data)
{
	uint8_t buffer[2] = {reg | NRF24_W_REGISTER, data};
	uint8_t status[2] = {0xFF, 0xFF};

    nrf24_set_CSN_low();
    HAL_SPI_TransmitReceive(handle, buffer, status, 2, 1000);
    nrf24_set_CSN_high();
}

void nrf24_write_reg_mb(SPI_HandleTypeDef* handle, uint8_t reg, uint8_t* data, uint8_t size)
{
	uint8_t cmd = reg | NRF24_W_REGISTER;
	uint8_t buffer[size + 1];
	buffer[0] = cmd;
    for (int i = 0; i < size; i++) {
        buffer[i + 1] = data[i];
    }
    uint8_t status[size + 1];

	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive(handle, buffer, status, size + 1, 2000);
	nrf24_set_CSN_high();
}

uint8_t nrf24_read_reg(SPI_HandleTypeDef* handle, uint8_t reg)
{
	// first byte in received data is STATUS and second is the actual data
	uint8_t cmd[2] = {reg, 0x00};
	uint8_t data[2] = {0xFF, 0xFF};

	nrf24_set_CSN_low();

	HAL_SPI_TransmitReceive(handle, cmd, data, 2, 1000);

	nrf24_set_CSN_high();

	return data[1];
}

void nrf24_read_reg_mb(SPI_HandleTypeDef* handle, uint8_t reg, uint8_t size, uint8_t* output)
{
	uint8_t cmd[size + 1];
	cmd[0] = reg;
	uint8_t data[size + 1];
	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive(handle, cmd, data, size + 1, 2000);
	nrf24_set_CSN_high();
	for (uint8_t i = 0; i < size; i++) {
		output[i] = data[i + 1];
	}
}

void nrf24_init(SPI_HandleTypeDef* handle)
{
	nrf24_set_CE_low();

	nrf24_write_reg(handle, NRF24_EN_AA_REG, 0); // disable auto-acknowledgment
	nrf24_write_reg(handle, NRF24_CONFIG_REG, 0); // set all config to 0 for now
	nrf24_write_reg(handle, NRF24_EN_RXADDR_REG, 0); // disable all data pipes
	nrf24_write_reg(handle, NRF24_SETUP_AW_REG, 0b00000011); // 5-byte address width
	nrf24_write_reg(handle, NRF24_SETUP_RETR_REG, 0); // disable auto retransmit
	nrf24_write_reg(handle, NRF24_RF_CH_REG, 0); // disable RF channels
	nrf24_write_reg(handle, NRF24_RF_SETUP_REG, 0b00001110); // all reset values -> data rate 2Mbps, power 0dBm

	nrf24_set_CE_high();
}

void nrf24_reset(SPI_HandleTypeDef* handle)
{
	nrf24_set_CE_low();

	nrf24_write_reg(handle, NRF24_EN_AA_REG, 0x00);
	nrf24_write_reg(handle, NRF24_CONFIG_REG, 0x08);
	nrf24_write_reg(handle, NRF24_EN_RXADDR_REG, 0x03);
	nrf24_write_reg(handle, NRF24_SETUP_AW_REG, 0x03);
	nrf24_write_reg(handle, NRF24_SETUP_RETR_REG, 0x03);
	nrf24_write_reg(handle, NRF24_RF_CH_REG, 0x02);
	nrf24_write_reg(handle, NRF24_RF_SETUP_REG, 0x0E);
	nrf24_write_reg(handle, NRF24_STATUS_REG, 0x00);
	nrf24_write_reg(handle, NRF24_OBSERVE_TX_REG, 0x00);
	nrf24_write_reg(handle, NRF24_CD_REG, 0x00);

	uint8_t rx_addr_p0[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_write_reg_mb(handle, NRF24_RX_ADDR_P0_REG, rx_addr_p0, 5);

	uint8_t rx_addr_p1[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_write_reg_mb(handle, NRF24_RX_ADDR_P1_REG, rx_addr_p1, 5);

	nrf24_write_reg(handle, NRF24_RX_ADDR_P2_REG, 0xC3);
	nrf24_write_reg(handle, NRF24_RX_ADDR_P3_REG, 0xC4);
	nrf24_write_reg(handle, NRF24_RX_ADDR_P4_REG, 0xC5);
	nrf24_write_reg(handle, NRF24_RX_ADDR_P5_REG, 0xC6);

	uint8_t tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_write_reg_mb(handle, NRF24_TX_ADDR_REG, tx_addr, 5);

	nrf24_write_reg(handle, NRF24_RX_PW_P0_REG, 0);
	nrf24_write_reg(handle, NRF24_RX_PW_P1_REG, 0);
	nrf24_write_reg(handle, NRF24_RX_PW_P2_REG, 0);
	nrf24_write_reg(handle, NRF24_RX_PW_P3_REG, 0);
	nrf24_write_reg(handle, NRF24_RX_PW_P4_REG, 0);
	nrf24_write_reg(handle, NRF24_RX_PW_P5_REG, 0);
	nrf24_write_reg(handle, NRF24_FIFO_STATUS_REG, 0x11);
	nrf24_write_reg(handle, NRF24_DYNPD_REG, 0);
	nrf24_write_reg(handle, NRF24_FEATURE_REG, 0);

	nrf24_set_CE_high();
}

void nrf24_read_state(SPI_HandleTypeDef* handle, NRF24_StateTypeDef* state) {
  state->config = nrf24_read_reg(handle, NRF24_CONFIG_REG);
  state->en_aa = nrf24_read_reg(handle, NRF24_EN_AA_REG);
  state->en_rxaddr = nrf24_read_reg(handle, NRF24_EN_RXADDR_REG);
  state->setup_aw = nrf24_read_reg(handle, NRF24_SETUP_AW_REG);
  state->setup_retr = nrf24_read_reg(handle, NRF24_SETUP_RETR_REG);
  state->rf_ch = nrf24_read_reg(handle, NRF24_RF_CH_REG);
  state->rf_setup = nrf24_read_reg(handle, NRF24_RF_SETUP_REG);
  state->observe_tx = nrf24_read_reg(handle, NRF24_OBSERVE_TX_REG);
  state->cd = nrf24_read_reg(handle, NRF24_CD_REG);

  // Read RX addresses for pipes 0 to 5
  nrf24_read_reg_mb(handle, NRF24_RX_ADDR_P0_REG, 5, state->rx_addr_p0);
  nrf24_read_reg_mb(handle, NRF24_RX_ADDR_P1_REG, 5, state->rx_addr_p1);
  state->rx_addr_p2 = nrf24_read_reg(handle, NRF24_RX_ADDR_P2_REG);
  state->rx_addr_p3 = nrf24_read_reg(handle, NRF24_RX_ADDR_P3_REG);
  state->rx_addr_p4 = nrf24_read_reg(handle, NRF24_RX_ADDR_P4_REG);
  state->rx_addr_p5 = nrf24_read_reg(handle, NRF24_RX_ADDR_P5_REG);

  // Read TX address (5 bytes)
  nrf24_read_reg_mb(handle, NRF24_TX_ADDR_REG, 5, state->tx_addr);

  // Read RX payload widths for pipes 0 to 5
  nrf24_read_reg_mb(handle, NRF24_RX_PW_P0_REG, 6, state->rx_pw_px);
  // Read remaining registers
  state->fifo_status = nrf24_read_reg(handle, NRF24_FIFO_STATUS_REG);
  state->dynpd = nrf24_read_reg(handle, NRF24_DYNPD_REG);
  state->feature = nrf24_read_reg(handle, NRF24_FEATURE_REG);
  state->status = nrf24_read_reg(handle, NRF24_STATUS_REG);
}

void nrf24_set_tx_mode(SPI_HandleTypeDef* handle, uint8_t channel, uint8_t* address)
{
	if (channel > 127) {
		return;
	}
	nrf24_set_CE_low();

	nrf24_write_reg(handle, NRF24_RF_CH_REG, channel);
	nrf24_write_reg_mb(handle, NRF24_TX_ADDR_REG, address, 5);

	uint8_t activate_cmd[2] = {NRF24_ACTIVATE, 0x73};
	uint8_t response[2] = {0xFF, 0xFF};
	HAL_SPI_TransmitReceive(handle, activate_cmd, response, 2, 1000);

	uint8_t current_feature = nrf24_read_reg(handle, NRF24_FEATURE_REG);
	nrf24_write_reg(handle, NRF24_FEATURE_REG, current_feature | 0b1);

	uint8_t config = nrf24_read_reg(handle, NRF24_CONFIG_REG);
	config &= 0b11111110;  // ptx mode
	config |= 0b10;  // power up
	nrf24_write_reg(handle, NRF24_CONFIG_REG, config);
	HAL_Delay(2);

	nrf24_set_CE_high();
}

void nrf24_set_rx_mode(SPI_HandleTypeDef* handle, uint8_t channel, uint8_t* address, uint8_t pipe)
{
	if (channel > 127 || pipe > 5) {
		return;
	}

	nrf24_set_CE_low();

	uint8_t current_pipes = nrf24_read_reg(handle, NRF24_EN_RXADDR_REG);
	nrf24_write_reg(handle, NRF24_EN_RXADDR_REG, current_pipes | (1 << pipe));

	uint8_t pipe_addr_reg = NRF24_RX_ADDR_P0_REG + pipe;
	if (pipe < 3) {
		// TODO: handle different address width than 5 bytes
		nrf24_write_reg_mb(handle, pipe_addr_reg, address, 5);
	}
	else {
		// if pipe is 3-5, only the first byte of address will be written
		nrf24_write_reg(handle, pipe_addr_reg, address);
	}

	uint8_t pipe_size_reg = NRF24_RX_PW_P0_REG + pipe;
	// TODO: handle dynamic payload length
	nrf24_write_reg(handle, pipe_size_reg, 32);


	uint8_t config = nrf24_read_reg(handle, NRF24_CONFIG_REG);
	config |= 0b11;  // prx mode and power up
	nrf24_write_reg(handle, NRF24_CONFIG_REG, config);
	HAL_Delay(2);
	nrf24_set_CE_high();
}

uint8_t nrf24_transmit(SPI_HandleTypeDef* handle, uint8_t* data, uint8_t size)
{
	uint8_t buffer[size + 1];
	uint8_t status[size + 1];
	buffer[0] = NRF24_W_TX_PAYLOAD_NO_ACK;
	for (uint8_t i = 0; i < size; i++) {
		buffer[i + 1] = data[i];
	}
	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive(handle, buffer, status, size + 1, 1000);
	nrf24_set_CSN_high();

	HAL_Delay(1);

	nrf24_set_CSN_low();
	nrf24_send_command(handle, NRF24_FLUSH_TX);
	nrf24_set_CSN_high();

	return status[0];

}

void nrf24_send_command(SPI_HandleTypeDef* handle, uint8_t cmd)
{
	uint8_t response = 0xFF;
	nrf24_set_CSN_low();
	HAL_SPI_TransmitReceive(handle, &cmd, &response, 1, 100);
	nrf24_set_CSN_high();
}

void nrf24_open_rx_pipe(SPI_HandleTypeDef* handle, uint8_t pipe)
{
	if (pipe > 5) {
		return;
	}
	uint8_t current_config = nrf24_read_reg(handle, NRF24_EN_RXADDR_REG);

	nrf24_write_reg(handle, NRF24_EN_RXADDR_REG , current_config | (1 << pipe));
}

void nrf24_enable_dyn_payload_len(SPI_HandleTypeDef* handle)
{
	uint8_t current_feature_reg = nrf24_read_reg(handle, NRF24_FEATURE_REG);
	nrf24_write_reg(handle, NRF24_FEATURE_REG, (current_feature_reg | 0b100));
}
