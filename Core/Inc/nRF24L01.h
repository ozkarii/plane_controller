/*
 * Driver for nRF24L01 transreceiver
*/
#ifndef NRF24L01_H
#define NRF24L01_H

#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi1; // SPI handle to be used
extern TIM_HandleTypeDef htim1; // Timer handle to be used (1MHz)

// Pins
#define NRF24_CE_Pin             GPIO_PIN_0
#define NRF24_CE_GPIO_Port       GPIOB
#define NRF24_CSN_Pin            GPIO_PIN_1
#define NRF24_CSN_GPIO_Port      GPIOB

// Registers, datasheet pages 54-59
#define NRF24_CONFIG_REG         0x00
#define NRF24_EN_AA_REG          0x01
#define NRF24_EN_RXADDR_REG      0x02
#define NRF24_SETUP_AW_REG       0x03
#define NRF24_SETUP_RETR_REG     0x04
#define NRF24_RF_CH_REG          0x05
#define NRF24_RF_SETUP_REG       0x06
#define NRF24_STATUS_REG         0x07
#define NRF24_OBSERVE_TX_REG     0x08
#define NRF24_CD_REG             0x09
#define NRF24_RX_ADDR_P0_REG     0x0A
#define NRF24_RX_ADDR_P1_REG     0x0B
#define NRF24_RX_ADDR_P2_REG     0x0C
#define NRF24_RX_ADDR_P3_REG     0x0D
#define NRF24_RX_ADDR_P4_REG     0x0E
#define NRF24_RX_ADDR_P5_REG     0x0F
#define NRF24_TX_ADDR_REG        0x10
#define NRF24_RX_PW_P0_REG       0x11
#define NRF24_RX_PW_P1_REG       0x12
#define NRF24_RX_PW_P2_REG       0x13
#define NRF24_RX_PW_P3_REG       0x14
#define NRF24_RX_PW_P4_REG       0x15
#define NRF24_RX_PW_P5_REG       0x16
#define NRF24_FIFO_STATUS_REG    0x17
#define NRF24_DYNPD_REG          0x1C
#define NRF24_FEATURE_REG        0x1D

// Commands, datasheet page 48
#define NRF24_R_REGISTER            0b00000000	// 0b000AAAAA
#define NRF24_W_REGISTER            0b00100000	// 0b001AAAAA
#define NRF24_R_RX_PAYLOAD          0b01100001
#define NRF24_W_TX_PAYLOAD          0b10100000
#define NRF24_FLUSH_TX              0b11100001
#define NRF24_FLUSH_RX              0b11100010
#define NRF24_REUSE_TX_PL           0b11100011
#define NRF24_ACTIVATE				0b01010000
#define NRF24_R_RX_PL_WID           0b01100000
#define NRF24_W_ACK_PAYLOAD         0b10101000	// 0b10101PPP
#define NRF24_W_TX_PAYLOAD_NO_ACK   0b10110000
#define NRF24_NOP                   0b11111111

#define HANDLE_SPI_ERROR(expr)          \
   do {                                 \
        HAL_StatusTypeDef status;       \
        status = (expr);                \
        if (status != HAL_OK) {         \
            return status;              \
        }                               \
    } while(0)

// Struct to store the whole state of the device's register contents
typedef struct {
  uint8_t config;
  uint8_t en_aa;
  uint8_t en_rxaddr;
  uint8_t setup_aw;
  uint8_t setup_retr;
  uint8_t rf_ch;
  uint8_t rf_setup;
  uint8_t status;
  uint8_t observe_tx;
  uint8_t cd;
  uint8_t rx_addr_p0[5];
  uint8_t rx_addr_p1[5];
  uint8_t rx_addr_p2;
  uint8_t rx_addr_p3;
  uint8_t rx_addr_p4;
  uint8_t rx_addr_p5;
  uint8_t tx_addr[5];	// Array for TX address
  uint8_t rx_pw_px[6];	// Array for RX payload widths for P0 to P5
  uint8_t fifo_status;
  uint8_t dynpd;
  uint8_t feature;
} NRF24_StateTypeDef;

// Power amplifier levels
typedef enum {
	PA_minus18dbm = 0b000,
	PA_minus12dbm = 0b010,
	PA_minus6dbm = 0b100,
	PA_0dbm = 0b110
} NRF24_PA_Level;


/**
 * @brief Sets CE high
 */
void nrf24_set_CE_high();

/**
 * @brief Sets CE low
 */
void nrf24_set_CE_low();

/**
 * @brief Sets CSN high
 */
void nrf24_set_CSN_high();

/**
 * @brief Sets CSN low
 */
void nrf24_set_CSN_low();

/**
 * @brief Creates a delay given in microseconds
 * 
 * @param microseconds delay in microseconds
 * 
 * @note This function uses timer handle htim1, which is assumed to be
 * 1MHz and count until 65535.
 */
void delay_us(uint16_t microseconds);

/**
 * @brief Writes the given data to the given register
 * 
 * @param reg address of the register to be written
 * @param data data to be written
 * @return HAL status from SPI
 */
HAL_StatusTypeDef nrf24_write_reg(uint8_t reg, uint8_t data);

/**
 * @brief Writes the given data to the given multi-byte register
 * 
 * @param reg address of the register to be written
 * @param data data to be written
 * @param size size of the register in bytes
 * @return HAL status from SPI
 */
HAL_StatusTypeDef nrf24_write_reg_mb(uint8_t reg, uint8_t* data, uint8_t size);

/**
 * @brief Reads the content of the given register
 * 
 * @param reg address of the register to be read
 * @return register content
 */
uint8_t nrf24_read_reg(uint8_t reg);

/**
 * @brief Reads the content of the given multi-byte register
 * 
 * @param reg address of the register to be read
 * @param size size of the register in bytes
 * @param output pointer to array where the register content will be stored
 * @return HAL status from SPI
 */
HAL_StatusTypeDef nrf24_read_reg_mb(uint8_t reg, uint8_t size, uint8_t* output);

/**
 * @brief Initializes the device
 * 
 * @note This function should be called before any other function
 */
HAL_StatusTypeDef nrf24_init();

/**
 * @brief Resets the device's registers to the reset values
 * given in the datasheet.
 */
void nrf24_reset();

/**
 * @brief Reads the states of the registers to the struct
 * 
 * @param state struct to which the register states will be stored
 */
void nrf24_read_state(NRF24_StateTypeDef* state);

/**
 * @brief Sets the device to transmit mode
 * 
 * @param channel transmit channel (0-127)
 * @param address 5-byte address to be used
 */
void nrf24_set_tx_mode(uint8_t channel, uint8_t* address);

/**
 * @brief Sets the device to receive mode
 * 
 * @param channel receive channel (0-127)
 * @param address 5-byte address to be used
 * @param pipe pipe number (0-5)
 */
void nrf24_set_rx_mode(uint8_t channel, uint8_t* address, uint8_t pipe);

/**
 * @brief Loads the data to be transmitted to the TX FIFO and sends it
 * 
 * @param data trasmitted data
 * @param size size of the data in bytes
 * @return STATUS register content after transmission
 */
HAL_StatusTypeDef nrf24_transmit(uint8_t* data, uint8_t size);

/**
 * @brief Sends a command to the device which does not require any data
 * other than the constant command word.
 * 
 * @param cmd command word to be sent
 */
void nrf24_send_command(uint8_t cmd);

/**
 * @brief Opens the given RX pipe
 * 
 * @param pipe pipe number (0-5)
 */
void nrf24_open_rx_pipe(uint8_t pipe);

/**
 * @brief Sets the power amplifier level
 * 
 * @param level desired power amplifier level
 */
void nrf24_set_pa_level(NRF24_PA_Level level);

/**
 * @brief Enables dynamic payload length
 * 
 * @note ACTIVATE command must be sent before this function.
 * It is sent in nrf24_init()
 */
void nrf24_enable_dyn_payload_len();

/**
 *
 */
void nrf24_flush_tx_fifo();

#endif
