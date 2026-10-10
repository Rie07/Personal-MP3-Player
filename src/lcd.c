/*
 * On-board 1.3" 240x240 ST7789 LCD (RT-Spark), 8080 8-bit bus on FSMC.
 *
 *   FSMC bank 1, NE3 (PG10), base address 0x68000000
 *   RS (data/command) = A18 (PD13)  -> data lives at base | (1 << 18)
 *   D0..D7 = PD14 PD15 PD0 PD1 PE7 PE8 PE9 PE10
 *   NOE (RD) = PD4, NWE (WR) = PD5
 *   Reset = PD3, backlight = PF9 (HIGH = on)
 *
 * Writes only for now. The timing is slow on purpose (about 450 ns per
 * access) so it is safe for reads later too.
 */
#include "lcd.h"
#include "lcd_font.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#define LCD_CMD_ADDR    0x68000000UL
#define LCD_DATA_ADDR   (0x68000000UL | (1UL << 18))

#define LCD_RST_PORT    GPIOD
#define LCD_RST_PIN     GPIO_PIN_3
#define LCD_BL_PORT     GPIOF
#define LCD_BL_PIN      GPIO_PIN_9

static inline void wr_cmd(uint8_t c)  { *(volatile uint8_t *)LCD_CMD_ADDR  = c; }
static inline void wr_data(uint8_t d) { *(volatile uint8_t *)LCD_DATA_ADDR = d; }

static void wr_cmd_data(uint8_t cmd, const uint8_t *data, uint8_t n)
{
    wr_cmd(cmd);
    for (uint8_t i = 0; i < n; i++) wr_data(data[i]);
}

static void lcd_pins_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_FSMC;

    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
            GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &g);

    g.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = GPIO_PIN_10;                       /* NE3 */
    HAL_GPIO_Init(GPIOG, &g);

    /* reset + backlight as plain outputs */
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;

    g.Pin = LCD_RST_PIN;
    HAL_GPIO_Init(LCD_RST_PORT, &g);
    g.Pin = LCD_BL_PIN;
    HAL_GPIO_Init(LCD_BL_PORT, &g);

    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_BL_PORT,  LCD_BL_PIN,  GPIO_PIN_RESET);
}

static void lcd_fsmc_init(void)
{
    __HAL_RCC_FSMC_CLK_ENABLE();

    /* Bank 3 timing (BTR3): ADDSET = 5, DATAST = 0x3F, mode A */
    FSMC_Bank1->BTCR[5] = (0x3FUL << 8) | 0x05UL;

    /* Bank 3 control (BCR3): SRAM type, 8-bit bus, write enabled */
    uint32_t bcr = FSMC_Bank1->BTCR[4];
    bcr &= ~((1UL << 1) |      /* MUXEN  */
             (3UL << 2) |      /* MTYP   */
             (3UL << 4) |      /* MWID   -> 8-bit */
             (1UL << 6) |      /* FACCEN */
             (1UL << 13));     /* WAITEN */
    bcr |= (1UL << 12);        /* WREN   */
    bcr |= 1UL;                /* MBKEN  */
    FSMC_Bank1->BTCR[4] = bcr;
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t col[4] = { x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF };
    uint8_t row[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };

    wr_cmd_data(0x2A, col, 4);     /* CASET */
    wr_cmd_data(0x2B, row, 4);     /* RASET */
    wr_cmd(0x2C);                  /* RAMWR: pixel data follows */
}

void lcd_fill(uint16_t color)
{
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * LCD_HEIGHT; i++) {
        wr_data(hi);
        wr_data(lo);
    }
}

void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t scale,
                   uint16_t fg, uint16_t bg)
{
    if (ch < FONT_FIRST || ch > FONT_LAST) ch = '?';
    const uint8_t *g = font8x16[ch - FONT_FIRST];

    uint16_t w = FONT_W * scale;
    uint16_t h = FONT_H * scale;
    if (scale == 0 || x + w > LCD_WIDTH || y + h > LCD_HEIGHT) return;

    lcd_set_window(x, y, x + w - 1, y + h - 1);
    for (int row = 0; row < FONT_H; row++) {
        for (int sy = 0; sy < scale; sy++) {
            for (int col = 0; col < FONT_W; col++) {
                uint16_t colour = (g[row] & (0x80 >> col)) ? fg : bg;
                for (int sx = 0; sx < scale; sx++) {
                    wr_data(colour >> 8);
                    wr_data(colour & 0xFF);
                }
            }
        }
    }
}

void lcd_draw_string(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                     uint16_t fg, uint16_t bg)
{
    while (*s) {
        lcd_draw_char(x, y, *s++, scale, fg, bg);
        x += FONT_W * scale;
    }
}

void lcd_init(void)
{
    lcd_pins_init();
    lcd_fsmc_init();

    /* hardware reset */
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(20));
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    wr_cmd(0x01);                              /* software reset */
    vTaskDelay(pdMS_TO_TICKS(150));
    wr_cmd(0x11);                              /* sleep out */
    vTaskDelay(pdMS_TO_TICKS(120));

    { uint8_t d[] = { 0x00 };                  wr_cmd_data(0x36, d, 1); }  /* MADCTL */
    { uint8_t d[] = { 0x05 };                  wr_cmd_data(0x3A, d, 1); }  /* 16-bit colour */
    { uint8_t d[] = { 0x0C, 0x0C, 0x00, 0x33, 0x33 }; wr_cmd_data(0xB2, d, 5); }
    { uint8_t d[] = { 0x35 };                  wr_cmd_data(0xB7, d, 1); }
    { uint8_t d[] = { 0x19 };                  wr_cmd_data(0xBB, d, 1); }
    { uint8_t d[] = { 0x2C };                  wr_cmd_data(0xC0, d, 1); }
    { uint8_t d[] = { 0x01 };                  wr_cmd_data(0xC2, d, 1); }
    { uint8_t d[] = { 0x12 };                  wr_cmd_data(0xC3, d, 1); }
    { uint8_t d[] = { 0x20 };                  wr_cmd_data(0xC4, d, 1); }
    { uint8_t d[] = { 0x0F };                  wr_cmd_data(0xC6, d, 1); }
    { uint8_t d[] = { 0xA4, 0xA1 };            wr_cmd_data(0xD0, d, 2); }
    { uint8_t d[] = { 0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,
                      0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23 };
                                               wr_cmd_data(0xE0, d, 14); } /* gamma + */
    { uint8_t d[] = { 0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,
                      0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23 };
                                               wr_cmd_data(0xE1, d, 14); } /* gamma - */

    wr_cmd(0x21);                              /* display inversion on */
    wr_cmd(0x29);                              /* display on */
    vTaskDelay(pdMS_TO_TICKS(20));

    lcd_fill(LCD_BLACK);
    HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_SET);   /* backlight on */
}