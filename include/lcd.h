#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_WIDTH   240
#define LCD_HEIGHT  240

/* RGB565 colours */
#define LCD_RED     0xF800
#define LCD_GREEN   0x07E0
#define LCD_BLUE    0x001F
#define LCD_YELLOW  0xFFE0
#define LCD_WHITE   0xFFFF
#define LCD_BLACK   0x0000

/* Call from a FreeRTOS task (it uses vTaskDelay while the panel wakes up). */
void lcd_init(void);

/* Fill the whole screen with one RGB565 colour. */
void lcd_fill(uint16_t color);

/* Text: 8x16 font, each pixel enlarged 'scale' times (scale 1 = 8x16 px).
 * x, y = top-left corner in pixels. Characters that do not fit are skipped. */
void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t scale,
                   uint16_t fg, uint16_t bg);
void lcd_draw_string(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                     uint16_t fg, uint16_t bg);

#endif /* LCD_H */