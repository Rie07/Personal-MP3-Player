/*
 * BCA182 Lab 2 - Personal MP3 Player
 * Step 5: codec + I2S test tone, LCD, on-board keys, UART console,
 *         potentiometer (PF6) -> DAC volume.
 *
 * LEDs (verified):  Yellow = PB14, Green = PA1, Red = PA8 (active HIGH)
 * Result after start-up:
 *   GREEN  steady = codec found, set up, tone is playing (listen on the jack)
 *   RED    steady = codec did not answer on I2C, or a codec write failed
 *   YELLOW steady = I2S/DMA failed to start
 *
 * Pins:
 *   I2S3  : MCK = PC7, WS = PA15, CK = PB3, SD = PB5   (AF6, DMA1 Stream7)
 *   I2C2  : see CODEC_I2C_PINSET below, codec address 0x10
 *
 * The codec register sequence follows the idea of RT-Thread's ES8388 driver
 * (Apache-2.0): mute, power up, I2S 16-bit / ratio 256, enable DAC + outputs,
 * then unmute.
 */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lcd.h"
#include <string.h>

#define LED_YELLOW_PIN   GPIO_PIN_14   /* GPIOB */
#define LED_GREEN_PIN    GPIO_PIN_1    /* GPIOA */
#define LED_RED_PIN      GPIO_PIN_8    /* GPIOA */

/* 1 -> try the board's 8 MHz crystal (HSE) first, fall back to the internal
 *      oscillator if it does not start. 0 -> always use the internal one.
 * The crystal gives a much cleaner audio clock than the internal oscillator. */
#define USE_HSE          1

/* 0 -> SCL = PF1, SDA = PF0      1 -> SCL = PB10, SDA = PB11 */
#define CODEC_I2C_PINSET 0

/* I2S3 data pin (SD):  0 -> PB5    1 -> PC12
 * If the jack only ever gives static, try the other option. */
#define I2S_SD_PINSET    0

#define ES8388_ADDR      0x10
#define CODEC_OUT_VOL    0x10          /* 0x00 (quiet) .. 0x21 (max). Start low! */

/* ES8388 registers */
#define ES_CONTROL1      0x00
#define ES_CONTROL2      0x01
#define ES_CHIPPOWER     0x02
#define ES_ADCPOWER      0x03
#define ES_DACPOWER      0x04
#define ES_MASTERMODE    0x08
#define ES_DACCONTROL1   0x17
#define ES_DACCONTROL2   0x18
#define ES_DACCONTROL3   0x19
#define ES_DACCONTROL4   0x1A
#define ES_DACCONTROL5   0x1B
#define ES_DACCONTROL16  0x26
#define ES_DACCONTROL17  0x27
#define ES_DACCONTROL20  0x2A
#define ES_DACCONTROL21  0x2B
#define ES_DACCONTROL23  0x2D
#define ES_LOUT1VOL      0x2E
#define ES_ROUT1VOL      0x2F
#define ES_LOUT2VOL      0x30
#define ES_ROUT2VOL      0x31

/* 441 Hz square wave: 100 frames per cycle at 44.1 kHz, 20 cycles per buffer */
#define TONE_FRAMES      2000
#define TONE_AMPLITUDE   4000

extern void xPortSysTickHandler(void);

static I2C_HandleTypeDef hi2c2;
static I2S_HandleTypeDef hi2s3;
static DMA_HandleTypeDef hdma_spi3_tx;
static uint16_t tone_buf[TONE_FRAMES * 2];     /* interleaved L,R */
static int es_err;
static volatile int codec_ready;      /* set once the codec is configured */
static volatile int volume_pct = -1; /* 0..100, -1 = not read yet        */
static SemaphoreHandle_t uart_mutex;

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void vApplicationIdleHook(void) { }

static void fatal(void) { for (;;) { } }

/* PLL setup: 168 MHz system clock. VCO input is 1 MHz either way:
 *   HSE 8 MHz / 8 = 1 MHz,   HSI 16 MHz / 16 = 1 MHz */
static HAL_StatusTypeDef pll_config(uint32_t source, uint32_t m)
{
    RCC_OscInitTypeDef osc = {0};

    if (source == RCC_PLLSOURCE_HSE) {
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
        osc.HSEState       = RCC_HSE_ON;
    } else {
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    }
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = source;
    osc.PLL.PLLM            = m;
    osc.PLL.PLLN            = 336;           /* 1 MHz x 336 = 336 MHz */
    osc.PLL.PLLP            = RCC_PLLP_DIV2; /* / 2         = 168 MHz */
    osc.PLL.PLLQ            = 7;
    return HAL_RCC_OscConfig(&osc);
}

static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clk = {0};
    int ok = 0;

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#if USE_HSE
    ok = (pll_config(RCC_PLLSOURCE_HSE, 8) == HAL_OK);
#endif
    if (!ok) {
        if (pll_config(RCC_PLLSOURCE_HSI, 16) != HAL_OK) fatal();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) fatal();
}

static void leds_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = LED_YELLOW_PIN;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = LED_GREEN_PIN | LED_RED_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(GPIOB, LED_YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_GREEN_PIN | LED_RED_PIN, GPIO_PIN_RESET);
}

/* ---------------- I2C (codec control) ---------------- */
static void codec_i2c_init(void)
{
    GPIO_InitTypeDef g = {0};

    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF4_I2C2;

#if CODEC_I2C_PINSET == 0
    __HAL_RCC_GPIOF_CLK_ENABLE();
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;          /* PF0 = SDA, PF1 = SCL */
    HAL_GPIO_Init(GPIOF, &g);
#else
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_10 | GPIO_PIN_11;        /* PB10 = SCL, PB11 = SDA */
    HAL_GPIO_Init(GPIOB, &g);
#endif

    __HAL_RCC_I2C2_CLK_ENABLE();

    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) fatal();
}

static void es_w(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    if (HAL_I2C_Master_Transmit(&hi2c2, ES8388_ADDR << 1, b, 2, 50) != HAL_OK) {
        es_err++;
    }
}

static void es8388_init(void)
{
    es_err = 0;

    es_w(ES_DACCONTROL3,  0x04);   /* mute while we configure           */
    es_w(ES_CONTROL2,     0x50);
    es_w(ES_CHIPPOWER,    0x00);   /* power up everything               */
    es_w(ES_MASTERMODE,   0x00);   /* codec is I2S slave, STM32 master  */
    es_w(ES_DACPOWER,     0xC0);   /* DAC and outputs off for now       */
    es_w(ES_CONTROL1,     0x12);
    es_w(ES_DACCONTROL1,  0x18);   /* I2S format, 16-bit                */
    es_w(ES_DACCONTROL2,  0x02);   /* single speed, MCLK = 256 x fs     */
    es_w(ES_DACCONTROL16, 0x00);
    es_w(ES_DACCONTROL17, 0x90);   /* left DAC -> left mixer            */
    es_w(ES_DACCONTROL20, 0x90);   /* right DAC -> right mixer          */
    es_w(ES_DACCONTROL21, 0x80);
    es_w(ES_DACCONTROL23, 0x00);
    es_w(ES_DACCONTROL4,  0x00);   /* digital volume 0 dB               */
    es_w(ES_DACCONTROL5,  0x00);
    es_w(ES_ADCPOWER,     0xFF);   /* ADC not used                      */
    es_w(ES_DACPOWER,     0x3C);   /* DAC on, OUT1 + OUT2 on            */

    es_w(ES_LOUT1VOL, CODEC_OUT_VOL);
    es_w(ES_ROUT1VOL, CODEC_OUT_VOL);
    es_w(ES_LOUT2VOL, CODEC_OUT_VOL);
    es_w(ES_ROUT2VOL, CODEC_OUT_VOL);

    es_w(ES_CHIPPOWER,    0xF0);   /* restart the codec state machine   */
    es_w(ES_CHIPPOWER,    0x00);   /* now that MCLK/LRCK are stable     */

    es_w(ES_DACCONTROL3,  0x00);   /* unmute                            */
}

/* ---------------- I2S3 + DMA (audio data) ---------------- */
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
    GPIO_InitTypeDef g = {0};

    if (hi2s->Instance != SPI3) return;

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF6_SPI3;

    g.Pin = GPIO_PIN_15;                      /* PA15 = WS  */
    HAL_GPIO_Init(GPIOA, &g);
#if I2S_SD_PINSET == 0
    g.Pin = GPIO_PIN_3 | GPIO_PIN_5;          /* PB3 = CK, PB5 = SD */
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_7;                       /* PC7 = MCK  */
    HAL_GPIO_Init(GPIOC, &g);
#else
    g.Pin = GPIO_PIN_3;                       /* PB3 = CK   */
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_7 | GPIO_PIN_12;         /* PC7 = MCK, PC12 = SD */
    HAL_GPIO_Init(GPIOC, &g);
#endif

    hdma_spi3_tx.Instance                 = DMA1_Stream7;
    hdma_spi3_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_spi3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_spi3_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_spi3_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_spi3_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) fatal();
    __HAL_LINKDMA(hi2s, hdmatx, hdma_spi3_tx);

    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 6, 0);   /* 6 is safe with FreeRTOS */
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

void DMA1_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

/* Triangle wave. period = frames per cycle: 100 -> 441 Hz, 50 -> 882 Hz,
 * 200 -> 220 Hz at 44.1 kHz. amp = 0 gives silence. */
static void tone_fill(int amp, int period)
{
    int half = period / 2;
    for (int i = 0; i < TONE_FRAMES; i++) {
        int p = i % period;
        int s = (p < half) ? (-amp + (2 * amp * p) / half)
                           : ( amp - (2 * amp * (p - half)) / half);
        tone_buf[2 * i]     = (uint16_t)(int16_t)s;   /* left  */
        tone_buf[2 * i + 1] = (uint16_t)(int16_t)s;   /* right */
    }
}

static void audio_i2s_init(void)
{
    RCC_PeriphCLKInitTypeDef pc = {0};

    /* PLLI2S: 1 MHz x 271 / 2 = 135.5 MHz.
     * With MCLK on: 135.5 MHz / (256 x 12) = 44.108 kHz (about 44.1 kHz). */
    pc.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    pc.PLLI2S.PLLI2SN       = 271;
    pc.PLLI2S.PLLI2SR       = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) fatal();

    hi2s3.Instance            = SPI3;
    hi2s3.Init.Mode           = I2S_MODE_MASTER_TX;
    hi2s3.Init.Standard       = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat     = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput     = I2S_MCLKOUTPUT_ENABLE;
    hi2s3.Init.AudioFreq      = I2S_AUDIOFREQ_44K;
    hi2s3.Init.CPOL           = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource    = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK) fatal();
}

/* ---------------- test task ---------------- */
static void audio_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(200));           /* let the codec power up */
    codec_i2c_init();

    if (HAL_I2C_IsDeviceReady(&hi2c2, ES8388_ADDR << 1, 3, 20) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOA, LED_RED_PIN, GPIO_PIN_SET);
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    tone_fill(TONE_AMPLITUDE, 100);
    audio_i2s_init();
    if (HAL_I2S_Transmit_DMA(&hi2s3, tone_buf, TONE_FRAMES * 2) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, LED_YELLOW_PIN, GPIO_PIN_SET);
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    vTaskDelay(pdMS_TO_TICKS(50));            /* MCLK/BCLK/LRCK are running */
    es8388_init();

    if (es_err == 0) {
        codec_ready = 1;
        HAL_GPIO_WritePin(GPIOA, LED_GREEN_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, LED_RED_PIN, GPIO_PIN_SET);
    }

    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

/* ---------------- UART console (USART1: PA9 = TX, PA10 = RX) ---------------- */
/* Goes through the ST-Link USB cable: open the COM port at 115200 baud. */
static UART_HandleTypeDef huart1;

static void uart_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) fatal();
}

static void uart_print(const char *s)
{
    xSemaphoreTake(uart_mutex, portMAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 200);
    xSemaphoreGive(uart_mutex);
}

/* ---------------- Potentiometer on PF6 (ADC3, channel 4) ---------------- */
static ADC_HandleTypeDef hadc3;

static void pot_init(void)
{
    GPIO_InitTypeDef g = {0};
    ADC_ChannelConfTypeDef ch = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    g.Pin  = GPIO_PIN_6;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &g);

    __HAL_RCC_ADC3_CLK_ENABLE();
    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 21 MHz */
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode          = DISABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) fatal();

    ch.Channel      = ADC_CHANNEL_4;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;   /* slow = good for a pot */
    if (HAL_ADC_ConfigChannel(&hadc3, &ch) != HAL_OK) fatal();
}

/* Average of 8 readings, 0..4095 */
static uint32_t pot_read(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 10);
        sum += HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);
    return sum / 8;
}

/* DAC digital volume: 0x00 = 0 dB, each step = -0.5 dB, 0xC0 = -96 dB.
 * Pot 0 % = mute, 1..100 % = -60 dB .. 0 dB. */
static uint8_t volume_to_codec(int pct)
{
    if (pct <= 0) return 0xC0;
    return (uint8_t)(((100 - pct) * 120) / 100);
}

static const char banner[] =
    "\r\n=== Personal MP3 Player (BCA182 Lab 2) ===\r\n"
    "Song number: hold B2-B4 (binary), then press B1 to select.\r\n"
    "Press B1 again within 5 seconds to confirm, otherwise nothing changes.\r\n"
    "Stop / play: press the stop-play button.\r\n"
    "Volume: turn the potentiometer.\r\n"
    "Buttons: B1 = UP, B2 = LEFT, B3 = DOWN, B4 = RIGHT (on-board keys).\r\n\r\n";

/* This will become the "adjust volume" thread. */
static void volume_task(void *arg)
{
    (void)arg;
    int last = -100;
    char buf[32];

    vTaskDelay(pdMS_TO_TICKS(500));
    uart_print(banner);

    for (;;) {
        int pct = (int)((pot_read() * 100U + 2047U) / 4095U);
        if (pct > 100) pct = 100;

        /* act only on real changes (>= 2 %) so the pot does not jitter */
        if (pct - last >= 2 || last - pct >= 2 ||
            (pct == 0 && last != 0) || (pct == 100 && last != 100)) {
            last = pct;
            volume_pct = pct;

            if (codec_ready) {
                uint8_t v = volume_to_codec(pct);
                es_w(ES_DACCONTROL4, v);
                es_w(ES_DACCONTROL5, v);
            }
            /* "Volume: NNN%\r\n" built by hand (no printf in the firmware) */
            {
                int n = 0;
                const char *p = "Volume: ";
                while (*p) buf[n++] = *p++;
                if (pct >= 100) buf[n++] = '1';
                if (pct >= 10)  buf[n++] = (char)('0' + (pct / 10) % 10);
                buf[n++] = (char)('0' + pct % 10);
                buf[n++] = '%'; buf[n++] = '\r'; buf[n++] = '\n'; buf[n] = 0;
            }
            uart_print(buf);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------- On-board keys (active LOW, internal pull-up, GPIOC) ----------------
 *   UP = PC5, DOWN = PC1, LEFT = PC0, RIGHT = PC4 */
#define KEY_UP_PIN     GPIO_PIN_5
#define KEY_DOWN_PIN   GPIO_PIN_1
#define KEY_LEFT_PIN   GPIO_PIN_0
#define KEY_RIGHT_PIN  GPIO_PIN_4

static void keys_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = KEY_UP_PIN | KEY_DOWN_PIN | KEY_LEFT_PIN | KEY_RIGHT_PIN;
    HAL_GPIO_Init(GPIOC, &g);
}

static int key_pressed(uint16_t pin)
{
    return HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_RESET;
}

/* Draw one row of the key test; only redraws when the state changes. */
static void show_key(uint16_t y, const char *name, uint16_t pin, int *last)
{
    int now = key_pressed(pin);
    if (now == *last) return;
    *last = now;

    lcd_draw_string(0, y, name, 2, LCD_WHITE, LCD_BLUE);
    lcd_draw_string(96, y, now ? "PRESSED" : "-------", 2,
                    now ? LCD_GREEN : LCD_WHITE, LCD_BLUE);
}

static void draw_tone_line(int on, const char *freq)
{
    char s[16];
    memcpy(s, on ? "TONE ON  " : "TONE OFF ", 9);
    memcpy(s + 9, freq, 3);
    s[12] = 'H'; s[13] = 'z'; s[14] = 0;
    lcd_draw_string(0, 172, s, 2, LCD_YELLOW, LCD_BLUE);
}

/* LCD task: title, the four on-board keys, tone status and the volume.
 * Only this task draws on the LCD for now, so no mutex is needed yet.
 * LEFT = tone on/off (silence test), UP = change the tone pitch. */
static void lcd_test_task(void *arg)
{
    (void)arg;
    static const int   periods[3] = { 100, 50, 200 };
    static const char *freqs[3]   = { "441", "882", "220" };
    int up = -1, down = -1, left = -1, right = -1;
    int shown_volume = -2;
    int tone_on = 1;
    int pitch = 0;
    char line[16];

    keys_init();
    lcd_init();
    lcd_fill(LCD_BLUE);
    lcd_draw_string(0, 6, "MP3 PLAYER", 3, LCD_WHITE, LCD_BLUE);   /* 10 chars x 24 px = 240 px */
    draw_tone_line(tone_on, freqs[pitch]);

    for (;;) {
        int prev_up = up;
        int prev_left = left;

        show_key( 56, "UP   ", KEY_UP_PIN,    &up);
        show_key( 84, "DOWN ", KEY_DOWN_PIN,  &down);
        show_key(112, "LEFT ", KEY_LEFT_PIN,  &left);
        show_key(140, "RIGHT", KEY_RIGHT_PIN, &right);

        if ((left == 1 && prev_left == 0) || (up == 1 && prev_up == 0)) {
            if (left == 1 && prev_left == 0) tone_on = !tone_on;
            if (up == 1 && prev_up == 0)     pitch = (pitch + 1) % 3;
            tone_fill(tone_on ? TONE_AMPLITUDE : 0, periods[pitch]);
            draw_tone_line(tone_on, freqs[pitch]);
        }

        if (volume_pct != shown_volume && volume_pct >= 0) {
            shown_volume = volume_pct;
            /* "VOLUME NNN%" with the number right-aligned in 3 characters */
            memcpy(line, "VOLUME ", 7);
            line[7]  = (shown_volume >= 100) ? '1' : ' ';
            line[8]  = (shown_volume >= 10) ? (char)('0' + (shown_volume / 10) % 10) : ' ';
            line[9]  = (char)('0' + shown_volume % 10);
            line[10] = '%';
            line[11] = 0;
            lcd_draw_string(0, 206, line, 2, LCD_YELLOW, LCD_BLUE);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    leds_init();

    uart_mutex = xSemaphoreCreateMutex();
    uart_init();
    pot_init();

    xTaskCreate(audio_task,    "audio",  512, NULL, 1, NULL);
    xTaskCreate(lcd_test_task, "lcd",    384, NULL, 1, NULL);
    xTaskCreate(volume_task,   "volume", 512, NULL, 1, NULL);
    vTaskStartScheduler();

    for (;;) { }
}