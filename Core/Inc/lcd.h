#include "main.h" // included to get GPIO pins and ports
#include "stm32f1xx_hal.h"

#define LCD_DEFAULT_DELAY 1
#define LCD_CLEAR 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ON_NO_CURSOR 0x0C
#define LCD_LENGTH 16

/**
 * @brief Set the Register Select pin high
 */
void lcd_set_rs_high();

/**
 * @brief Set the Register Select pin low
 */
void lcd_set_rs_low();

/**
 * @brief Set the Enable pin high
 */
void lcd_set_en_high();

/**
 * @brief Set the Enable pin low
 */
void lcd_set_en_low();

/**
 * @brief Set the Read/Write pin low for write mode
 */
void lcd_set_rw_low();

/**
 * @brief Set the Read/Write pin high for read mode
 */
void lcd_set_rw_high();

/**
 * @brief Pulse the Enable pin for specified time
 * 
 * @param delay Pulse time in ms
 */
void lcd_pulse_en(uint16_t delay);

/**
 * @brief Write data to the LCD in 4-bit mode
 * 
 * @param data Data to write
 */
void lcd_write_data(uint8_t data);

/**
 * @brief Set the LCD to 4-bit mode
 */
void lcd_set_4bit_mode();

/**
 * @brief Set the DDRAM address of the LCD
 * 
 * @param address Address to set
 */
void lcd_set_ddram_address(uint8_t address);

/**
 * @brief Clear the LCD
 */
void lcd_clear();

/**
 * @brief Return the cursor to the home position
 */
void lcd_return_home();

/**
 * @brief Turn on the LCD with no cursor
 */
void lcd_on_no_cursor();

/**
 * @brief Initialize the LCD in 4-bit mode. Should be called before
 * anything else
 */
void lcd_init();

/**
 * @brief Print a null terminated string of max 16 characters to the LCD
 * 
 * @param str String to print
 * 
 * @note If the string is longer than 16 characters, it will be truncated
 */
void lcd_print(const char* str);
