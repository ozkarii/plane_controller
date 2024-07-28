#include "stm32f1xx_hal.h"
#include "lcd.h"

static inline void lcd_set_rs_high()
{
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, 1);
}

static inline void lcd_set_rs_low()
{
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, 0);
}

static inline void lcd_set_en_high()
{
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, 1);
}

static inline void lcd_set_en_low()
{
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, 0);
}

static inline void lcd_set_rw_high()
{
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, 1);
}
static inline void lcd_set_rw_low()
{
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, 0);

}

static inline void lcd_pulse_en(uint16_t delay)
{
	lcd_set_en_high();
	HAL_Delay(delay);
	lcd_set_en_low();
}

void lcd_write_data(uint8_t data)
{
	lcd_set_rs_high();

	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, data >> 7);
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, (data >> 6) & 1);
	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, (data >> 5) & 1);
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, (data >> 4) & 1);

	lcd_pulse_en(LCD_DEFAULT_DELAY);

	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, (data >> 3) & 1);
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, (data >> 2) & 1);
	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, (data >> 1) & 1);
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, data & 1);

	lcd_pulse_en(LCD_DEFAULT_DELAY);

	lcd_set_rs_low();
}

void lcd_send_command(uint8_t cmd)
{
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, cmd >> 7);
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, (cmd >> 6) & 1);
	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, (cmd >> 5) & 1);
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, (cmd >> 4) & 1);

	lcd_pulse_en(LCD_DEFAULT_DELAY);

	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, (cmd >> 3) & 1);
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, (cmd >> 2) & 1);
	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, (cmd >> 1) & 1);
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, cmd & 1);

	lcd_pulse_en(LCD_DEFAULT_DELAY);
}

void lcd_set_4bit_mode()
{
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, 0);
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, 0);
	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, 1);
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, 0);

	lcd_pulse_en(LCD_DEFAULT_DELAY);
}

void lcd_set_ddram_address(uint8_t address)
{
	lcd_set_rw_low();

	address |= (1 << 7);
	lcd_send_command(address);
	HAL_Delay(LCD_DEFAULT_DELAY);
}

void lcd_clear()
{
	lcd_set_rs_low();
	lcd_send_command(LCD_CLEAR);
}

void lcd_return_home()
{
	lcd_set_rs_low();
	lcd_send_command(LCD_RETURN_HOME);
}

void lcd_on_no_cursor()
{
	lcd_set_rs_low();
	lcd_send_command(LCD_ON_NO_CURSOR);
}

void lcd_init()
{
	HAL_Delay(50);
  lcd_set_rs_low();
  lcd_set_en_low();
  lcd_set_rw_low();
	lcd_set_4bit_mode();
	lcd_send_command(LCD_CLEAR);
	HAL_Delay(5);
	lcd_send_command(LCD_RETURN_HOME);
	lcd_send_command(LCD_ON_NO_CURSOR);
}

void lcd_print(const char* str) 
{
	int i = 0;
  while (str[i] != '\0' && i < LCD_LENGTH) {
    if (i == LCD_LENGTH_HALF) {
    	lcd_set_ddram_address(40);
    }
    lcd_write_data(str[i]);
    ++i;
  }
}
