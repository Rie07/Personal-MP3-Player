/*
 * BCA182 Lab 2 - Personal MP3 Player (RT-Spark, STM32F407, FreeRTOS)
 *
 * Buttons: B1 = UP, B2 = LEFT, B3 = DOWN, B4 = RIGHT (on board) and
 *          USER_BUTTON = a push button on the breadboard (PA0 to GND).
 *   - Hold LEFT / DOWN / RIGHT = song code in binary (4 / 2 / 1) = songs 1-8,
 *     tap UP = select. The order of the keys does not matter.
 *   - Tap UP again within 5 s = confirm (the new song plays from the start).
 *     Do nothing for 5 s = the current song carries on.
 *   - USER_BUTTON: tap = stop / play, hold 1.5 s = faster tempo (wraps around).
 *   - Potentiometer (PF6) = volume.
 *   - Only the first 8 songs of song_def.h are used (3 code keys = 8 codes).
 *
 * LEDs: Yellow = PB14 (playing), Red = PA8 (paused / stopped),
 *       Green = PA1 (changing song).
 *
 * Threads (names follow the handout):
 *   update_lcd_leds_thread  - draws the LCD, drives the LEDs
 *   polling_buttons         - reads the keys, runs the selection flow
 *   adjust_volume           - reads the potentiometer, sets codec volume
 *   audio_task              - one-off start-up of the codec and the I2S stream
 * The "Ticker" that steps through the notes and the 5 s "Timeout" are
 * FreeRTOS software timers. The LCD is protected by lcd_mutex.
 * The idle task puts the CPU to sleep (WFI) to save power.
 */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "lcd.h"
#include "songs.h"
#include <string.h>

#define LED_YELLOW_PIN   GPIO_PIN_14   /* GPIOB */
#define LED_GREEN_PIN    GPIO_PIN_1    /* GPIOA */
#define LED_RED_PIN      GPIO_PIN_8    /* GPIOA */

/* 1 -> use the board's 8 MHz crystal (falls back to the internal oscillator) */
#define USE_HSE          1

/* 0 -> SCL = PF1, SDA = PF0      1 -> SCL = PB10, SDA = PB11 */
#define CODEC_I2C_PINSET 0

#define ES8388_ADDR      0x10

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
#define CODEC_OUT_VOL    0x1C          /* analog output level (0x00..0x21) */

/* ---- audio stream ---- */
#define AUDIO_FS         44108u        /* real sample rate from the I2S dividers */
#define AUDIO_FRAMES     256           /* stereo frames per half buffer          */
#define AUDIO_AMP        12000         /* tone level (max 32767)                 */

/* ---- note timing ----
 * Duration of a note in seconds = beat x tempo x BEAT_SCALE.
 * Chosen by ear (bigger = slower). The last part of each note is a short
 * silent gap. */
#define BEAT_SCALE       6.0f      /* start value; hold USER_BUTTON to make it faster */
#define BEAT_SCALE_MIN   1.0f      /* fastest */
#define BEAT_SCALE_MAX   12.0f     /* slowest */
#define GAP_FRACTION     0.15f
#define GAP_MIN_MS       8.0f

extern void xPortSysTickHandler(void);

static I2C_HandleTypeDef hi2c2;
static I2S_HandleTypeDef hi2s3;
static DMA_HandleTypeDef hdma_spi3_tx;
static UART_HandleTypeDef huart1;
static ADC_HandleTypeDef hadc3;

static uint16_t audio_buf[AUDIO_FRAMES * 2 * 2];   /* two halves, L/R interleaved */
static volatile uint32_t synth_inc;                /* phase step per frame, 0 = silence */
static uint32_t synth_phase;

static int es_err;
static volatile int codec_ready;
static volatile int audio_state;                   /* 0 start, 1 ok, 2 no codec, 3 i2s fail, 4 codec write error */
static volatile int volume_pct = -1;
static SemaphoreHandle_t uart_mutex;

static TimerHandle_t note_timer;
static volatile int player_playing;
static volatile int cur_song;                      /* 0 .. SONG_SELECTABLE-1 (8 songs) */
static int note_idx;
static int tick_state;                             /* 0 = start a note, 1 = gap after it */
static TickType_t gap_ticks;
static volatile float beat_scale = BEAT_SCALE;   /* hold USER_BUTTON alone for faster */

/* ---- song selection flow ---- */
#define CONFIRM_MS       5000          /* time to confirm a new song       */
enum { UI_IDLE = 0, UI_CONFIRM = 1 };
static volatile int ui_state = UI_IDLE;
static volatile int pending_song;                 /* song chosen, waiting for OK */
static volatile int confirm_expired;              /* set by the timeout          */
static volatile TickType_t confirm_deadline;
static TimerHandle_t confirm_timer;               /* the "Timeout": one shot, 5 s */
static SemaphoreHandle_t lcd_mutex;
static volatile int lcd_ready;
static volatile int user_stuck;                   /* 1 = USER_BUTTON pin reads pressed all the time */

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/* Sleep mode: the CPU sleeps until the next interrupt whenever no task has work. */
void vApplicationIdleHook(void) { __WFI(); }

static void fatal(void) { for (;;) { } }

/* ------------------------------------------------------------------ */
/* Clock: 168 MHz. VCO input is 1 MHz either way.                      */
/* ------------------------------------------------------------------ */
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
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
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

/* ------------------------------------------------------------------ */
/* LEDs                                                                */
/* ------------------------------------------------------------------ */
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

static void leds_set(int yellow, int green, int red)
{
    HAL_GPIO_WritePin(GPIOB, LED_YELLOW_PIN, yellow ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_GREEN_PIN,  green  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_RED_PIN,    red    ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------ */
/* I2C: codec control                                                  */
/* ------------------------------------------------------------------ */
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
    es_w(ES_CHIPPOWER,    0x00);
    es_w(ES_MASTERMODE,   0x00);   /* codec is I2S slave                */
    es_w(ES_DACPOWER,     0xC0);
    es_w(ES_CONTROL1,     0x12);
    es_w(ES_DACCONTROL1,  0x18);   /* I2S format, 16-bit                */
    es_w(ES_DACCONTROL2,  0x02);   /* MCLK = 256 x fs                   */
    es_w(ES_DACCONTROL16, 0x00);
    es_w(ES_DACCONTROL17, 0x90);
    es_w(ES_DACCONTROL20, 0x90);
    es_w(ES_DACCONTROL21, 0x80);
    es_w(ES_DACCONTROL23, 0x00);
    es_w(ES_DACCONTROL4,  0x00);   /* digital volume 0 dB (pot changes it) */
    es_w(ES_DACCONTROL5,  0x00);
    es_w(ES_ADCPOWER,     0xFF);
    es_w(ES_DACPOWER,     0x3C);   /* DAC + outputs on                  */

    es_w(ES_LOUT1VOL, CODEC_OUT_VOL);
    es_w(ES_ROUT1VOL, CODEC_OUT_VOL);
    es_w(ES_LOUT2VOL, CODEC_OUT_VOL);
    es_w(ES_ROUT2VOL, CODEC_OUT_VOL);

    es_w(ES_CHIPPOWER,    0xF0);   /* restart the codec state machine   */
    es_w(ES_CHIPPOWER,    0x00);

    es_w(ES_DACCONTROL3,  0x00);   /* unmute                            */
}

/* ------------------------------------------------------------------ */
/* I2S3 + DMA: audio stream. The DMA runs a circular buffer; each time  */
/* a half is played, the callback refills it from the tone generator.   */
/* ------------------------------------------------------------------ */
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

    g.Pin = GPIO_PIN_15;                      /* PA15 = WS            */
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_3 | GPIO_PIN_5;          /* PB3 = CK, PB5 = SD   */
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_7;                       /* PC7 = MCK            */
    HAL_GPIO_Init(GPIOC, &g);

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

/* Triangle wave at the current pitch. synth_inc = 0 gives silence. */
static void audio_fill(uint16_t *dst, int frames)
{
    uint32_t inc = synth_inc;
    uint32_t ph  = synth_phase;

    for (int i = 0; i < frames; i++) {
        int16_t s = 0;
        if (inc) {
            int32_t x = (int32_t)(ph >> 16);                    /* 0..65535 */
            int32_t v = (x < 32768) ? (x * 2 - 32768)
                                    : (32767 - (x - 32768) * 2);
            s = (int16_t)((v * AUDIO_AMP) >> 15);
            ph += inc;
        }
        dst[2 * i]     = (uint16_t)s;     /* left  */
        dst[2 * i + 1] = (uint16_t)s;     /* right */
    }
    synth_phase = ph;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;
    audio_fill(&audio_buf[0], AUDIO_FRAMES);                /* first half just played */
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;
    audio_fill(&audio_buf[AUDIO_FRAMES * 2], AUDIO_FRAMES); /* second half just played */
}

static void audio_i2s_init(void)
{
    RCC_PeriphCLKInitTypeDef pc = {0};

    /* PLLI2S: 1 MHz x 271 / 2 = 135.5 MHz -> 44.108 kHz with MCLK on */
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

static void audio_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(200));           /* let the codec power up */
    codec_i2c_init();

    if (HAL_I2C_IsDeviceReady(&hi2c2, ES8388_ADDR << 1, 3, 20) != HAL_OK) {
        audio_state = 2;
        vTaskDelete(NULL);
    }

    audio_i2s_init();
    if (HAL_I2S_Transmit_DMA(&hi2s3, audio_buf, AUDIO_FRAMES * 2 * 2) != HAL_OK) {
        audio_state = 3;
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(50));            /* MCLK/BCLK/LRCK are running */
    es8388_init();

    if (es_err == 0) {
        codec_ready = 1;
        audio_state = 1;
    } else {
        audio_state = 4;
    }
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Player: the "Ticker" that walks through the notes                   */
/* ------------------------------------------------------------------ */
static uint32_t freq_to_inc(float hz)
{
    return (uint32_t)((double)hz * 4294967296.0 / (double)AUDIO_FS);
}

static TickType_t ms_to_ticks(float ms)
{
    TickType_t t = (TickType_t)(ms * (float)configTICK_RATE_HZ / 1000.0f + 0.5f);
    return (t < 1) ? 1 : t;
}

/* Runs in the FreeRTOS timer task. State 0 starts the next note and waits
 * for its sounding time; state 1 is the short silent gap that follows. */
static void player_tick(TimerHandle_t t)
{
    (void)t;
    int s = cur_song;

    if (!player_playing) {
        synth_inc = 0;
        return;
    }

    if (tick_state == 0) {
        if (note_idx >= song_length(s)) {         /* song finished */
            player_playing = 0;
            synth_inc = 0;
            note_idx = 0;
            return;
        }
        float period_ms = song_note(s, note_idx);
        float dur_ms    = song_beat(s, note_idx) * song_tempo(s) * beat_scale * 1000.0f;
        float gap_ms    = dur_ms * GAP_FRACTION;
        if (gap_ms < GAP_MIN_MS) gap_ms = GAP_MIN_MS;
        float on_ms     = dur_ms - gap_ms;
        if (on_ms < GAP_MIN_MS) on_ms = GAP_MIN_MS;

        synth_inc = (period_ms > 0.0f) ? freq_to_inc(1000.0f / period_ms) : 0;   /* 0 = rest */
        gap_ticks = ms_to_ticks(gap_ms);
        tick_state = 1;
        xTimerChangePeriod(note_timer, ms_to_ticks(on_ms), 0);
    } else {
        synth_inc = 0;                            /* silent gap */
        note_idx++;
        tick_state = 0;
        xTimerChangePeriod(note_timer, gap_ticks, 0);
    }
}

/* Stop: silence and back to the start of the song. */
static void player_stop(void)
{
    player_playing = 0;
    xTimerStop(note_timer, 0);
    synth_inc = 0;
    note_idx = 0;
    tick_state = 0;
}

/* Play the current song from the beginning. */
static void player_start(void)
{
    note_idx = 0;
    tick_state = 0;
    player_playing = 1;
    xTimerChangePeriod(note_timer, 1, 0);         /* first note in 1 ms */
}

/* Pause: silence, but remember where we are. */
static void player_pause(void)
{
    player_playing = 0;
    xTimerStop(note_timer, 0);
    synth_inc = 0;
    tick_state = 0;
}

/* Resume from the remembered note (or from the start after the song ended). */
static void player_resume(void)
{
    if (note_idx >= song_length(cur_song)) note_idx = 0;
    tick_state = 0;
    player_playing = 1;
    xTimerChangePeriod(note_timer, 1, 0);
}

/* ------------------------------------------------------------------ */
/* UART console (USART1: PA9 = TX, PA10 = RX), through the ST-Link USB */
/* ------------------------------------------------------------------ */
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

/* tiny integer formatting: no printf in the firmware */
static void uart_print_int(int v)
{
    char b[12];
    int n = 0;
    if (v < 0) { b[n++] = '-'; v = -v; }
    char d[10];
    int k = 0;
    do { d[k++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (k) b[n++] = d[--k];
    b[n] = 0;
    uart_print(b);
}

/* ------------------------------------------------------------------ */
/* Potentiometer on PF6 (ADC3 channel 4)                               */
/* ------------------------------------------------------------------ */
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
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &ch) != HAL_OK) fatal();
}

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

/* DAC digital volume: 0x00 = 0 dB, 0.5 dB per step. Pot 0 % = mute. */
static uint8_t volume_to_codec(int pct)
{
    if (pct <= 0) return 0xC0;
    return (uint8_t)(((100 - pct) * 120) / 100);   /* 0 dB .. -60 dB */
}

/* ------------------------------------------------------------------ */
/* LCD helpers. Every LCD access holds lcd_mutex: the LCD is shared by */
/* the LCD thread and the volume thread.                               */
/* ------------------------------------------------------------------ */

/* Text in a fixed 15-character field (240 px at scale 2). Caller holds the mutex. */
static void draw_field(uint16_t y, const char *s, uint16_t fg)
{
    char buf[16];
    int i = 0;
    for (; i < 15 && s[i]; i++) buf[i] = s[i];
    for (; i < 15; i++) buf[i] = ' ';
    buf[15] = 0;
    lcd_draw_string(0, y, buf, 2, fg, LCD_BLUE);
}

/* Small text in a fixed 30-character field. Caller holds the mutex. */
static void draw_small(uint16_t y, const char *s, uint16_t fg)
{
    char buf[31];
    int i = 0;
    for (; i < 30 && s[i]; i++) buf[i] = s[i];
    for (; i < 30; i++) buf[i] = ' ';
    buf[30] = 0;
    lcd_draw_string(0, y, buf, 1, fg, LCD_BLUE);
}

static void bits_to_text(int v, char *out)       /* 3-bit binary, e.g. "101" */
{
    out[0] = (char)('0' + ((v >> 2) & 1));
    out[1] = (char)('0' + ((v >> 1) & 1));
    out[2] = (char)('0' + (v & 1));
    out[3] = 0;
}

/* Tiny text builders (no printf in the firmware). They return the new length. */
static int put_str(char *buf, int n, const char *s)
{
    while (*s) buf[n++] = *s++;
    return n;
}

static int put_uint(char *buf, int n, int v)
{
    char d[6];
    int k = 0;
    do { d[k++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (k) buf[n++] = d[--k];
    return n;
}

/* "N.N" or "NN.N" */
static void scale_to_text(float v, char *out)
{
    int t = (int)(v * 10.0f + 0.5f);
    int n = 0;
    if (t >= 100) out[n++] = (char)('0' + (t / 100) % 10);
    out[n++] = (char)('0' + (t / 10) % 10);
    out[n++] = '.';
    out[n++] = (char)('0' + t % 10);
    out[n] = 0;
}

static void draw_volume(int pct)
{
    char line[16];

    memcpy(line, "VOLUME ", 7);
    line[7]  = (pct >= 100) ? '1' : ' ';
    line[8]  = (pct >= 10) ? (char)('0' + (pct / 10) % 10) : ' ';
    line[9]  = (char)('0' + pct % 10);
    line[10] = '%';
    line[11] = 0;

    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    draw_field(180, line, LCD_YELLOW);
    xSemaphoreGive(lcd_mutex);
}

/* ------------------------------------------------------------------ */
/* UART instructions                                                    */
/* ------------------------------------------------------------------ */
static void print_banner(void)
{
    char code[5];

    uart_print("\r\n=== Personal MP3 Player (BCA182 Lab 2) ===\r\n\r\n");
    uart_print("Songs (code = LEFT DOWN RIGHT, a pressed key is 1):\r\n");
    for (int i = 0; i < SONG_SELECTABLE; i++) {
        bits_to_text(i, code);
        uart_print("  ");
        uart_print(code);
        uart_print("  ");
        uart_print(song_name1(i));
        uart_print(" ");
        uart_print(song_name2(i));
        uart_print("\r\n");
    }
    uart_print("\r\nHow to use (buttons 1-4 = UP, LEFT, DOWN, RIGHT):\r\n");
    uart_print(" 1. Hold LEFT / DOWN / RIGHT to make the song code in binary.\r\n");
    uart_print("    With no key held the code is 000 = the first song.\r\n");
    uart_print(" 2. While holding them, tap UP to select the song (the key order does not matter).\r\n");
    uart_print(" 3. Tap UP again within 5 seconds to confirm. If you do nothing, the current\r\n");
    uart_print("    song carries on.\r\n");
    uart_print(" 4. Tap the breadboard USER_BUTTON to stop / play.\r\n");
    uart_print("    Hold it for 1.5 seconds to make the tempo faster (wraps to the slowest).\r\n");
    uart_print(" 5. Turn the potentiometer to change the volume.\r\n\r\n");
    uart_print("LEDs: yellow = playing, red = paused / stopped, green = changing song.\r\n\r\n");
}

/* Handout name: adjust_volume thread */
static void adjust_volume(void *arg)
{
    (void)arg;
    int last = -100;

    vTaskDelay(pdMS_TO_TICKS(500));
    print_banner();

    for (;;) {
        int pct = (int)((pot_read() * 100U + 2047U) / 4095U);
        if (pct > 100) pct = 100;

        if (pct - last >= 2 || last - pct >= 2 ||
            (pct == 0 && last != 0) || (pct == 100 && last != 100)) {
            last = pct;
            volume_pct = pct;

            if (codec_ready) {
                uint8_t v = volume_to_codec(pct);
                es_w(ES_DACCONTROL4, v);
                es_w(ES_DACCONTROL5, v);
            }
            uart_print("Volume: ");
            uart_print_int(pct);
            uart_print("%\r\n");
            if (lcd_ready) draw_volume(pct);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ------------------------------------------------------------------ */
/* On-board keys (active LOW, internal pull-up, GPIOC)                 */
/*   B1 = UP (PC5)   B2 = LEFT (PC0)   B3 = DOWN (PC1)   B4 = RIGHT (PC4) */
/* ------------------------------------------------------------------ */
#define KEY_UP_PIN     GPIO_PIN_5
#define KEY_DOWN_PIN   GPIO_PIN_1
#define KEY_LEFT_PIN   GPIO_PIN_0
#define KEY_RIGHT_PIN  GPIO_PIN_4

/* USER_BUTTON (stop / play): a push button on the breadboard between this pin and GND.
 * Change these two lines if you wired it to another free pin. */
#define USER_BTN_PORT  GPIOA
#define USER_BTN_PIN   GPIO_PIN_0

static void keys_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = KEY_UP_PIN | KEY_DOWN_PIN | KEY_LEFT_PIN | KEY_RIGHT_PIN;
    HAL_GPIO_Init(GPIOC, &g);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = USER_BTN_PIN;                     /* same input + pull-up setup */
    HAL_GPIO_Init(USER_BTN_PORT, &g);
}

static int key_pressed(uint16_t pin)
{
    return HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_RESET;
}

static int user_pressed(void)
{
    return HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN) == GPIO_PIN_RESET;
}

/* The "Timeout": runs once, 5 s after a song was selected. Nothing was
 * confirmed, so everything carries on as before. */
static void confirm_timeout(TimerHandle_t t)
{
    (void)t;
    ui_state = UI_IDLE;
    confirm_expired = 1;
}

static void announce(const char *prefix, int song)
{
    uart_print(prefix);
    uart_print_int(song + 1);
    uart_print(": ");
    uart_print(song_name1(song));
    uart_print(" ");
    uart_print(song_name2(song));
}

/* Second tap of UP: switch to the chosen song and play it from the start. */
static void confirm_song(void)
{
    int declared, notes, beats;

    xTimerStop(confirm_timer, 0);
    ui_state = UI_IDLE;
    player_stop();
    cur_song = pending_song;
    player_start();

    announce("Confirmed. Playing song ", cur_song);
    song_counts(cur_song, &declared, &notes, &beats);
    uart_print("  (length ");
    uart_print_int(declared);
    uart_print(", notes ");
    uart_print_int(notes);
    uart_print(", beats ");
    uart_print_int(beats);
    uart_print(")\r\n");
}

/* UP tapped. bits = the song code seen on LEFT / DOWN / RIGHT while UP was down. */
static void on_up_tap(int bits)
{
    char code[5];

    if (ui_state == UI_IDLE) {
        /* first tap: choose the song and start the 5 second window */
        pending_song = bits;                             /* 0 .. 7 */
        confirm_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(CONFIRM_MS);
        ui_state = UI_CONFIRM;
        xTimerChangePeriod(confirm_timer, pdMS_TO_TICKS(CONFIRM_MS), 0);   /* the Timeout */

        bits_to_text(bits, code);
        announce("Selected song ", bits);
        uart_print(" (code ");
        uart_print(code);
        uart_print("). Tap UP again within 5 seconds to confirm.\r\n");
    } else {
        confirm_song();                                  /* second tap: confirm */
    }
}

/* USER_BUTTON tapped: stop / play. */
static void toggle_play(void)
{
    if (player_playing) {
        player_pause();
        uart_print("Paused.\r\n");
    } else {
        player_resume();
        uart_print("Playing.\r\n");
    }
}

/* USER_BUTTON held for 1 s: faster tempo, 15 % per hold; wraps to the slowest. */
static void change_speed(void)
{
    char b[8];
    float v = beat_scale * 0.85f;

    if (v < BEAT_SCALE_MIN) v = BEAT_SCALE_MAX;
    beat_scale = v;

    scale_to_text(v, b);
    uart_print("Tempo scale: ");
    uart_print(b);
    uart_print("  (lower = faster)\r\n");
}

/* Handout name: polling_buttons thread. A key counts after it reads the
 * same twice in a row, 20 ms apart (debounce).
 *
 *   Song code : every LEFT / DOWN / RIGHT key seen while UP is down counts, so
 *               the order of the keys does not matter. UP acts when released.
 *   USER_BUTTON: a tap = stop / play, a 1 second hold = faster tempo. */
static void polling_buttons(void *arg)
{
    (void)arg;
    int last_raw[5] = { 0, 0, 0, 0, 0 };
    int stable[5]   = { 0, 0, 0, 0, 0 };  /* 0 = UP, 1 = LEFT, 2 = DOWN, 3 = RIGHT, 4 = USER */
    int up_bits = 0;
    TickType_t user_down_at = 0;
    int user_long_done = 0;
    int user_armed = 0;                   /* 0 until the USER pin has been seen released */
    int stuck_reported = 0;

    keys_init();

    for (;;) {
        int user_pin = user_pressed();
        if (!user_pin) user_armed = 1;
        int raw[5] = { key_pressed(KEY_UP_PIN),   key_pressed(KEY_LEFT_PIN),
                       key_pressed(KEY_DOWN_PIN), key_pressed(KEY_RIGHT_PIN),
                       user_pin && user_armed };
        int prev[5] = { stable[0], stable[1], stable[2], stable[3], stable[4] };
        TickType_t now = xTaskGetTickCount();

        for (int i = 0; i < 5; i++) {
            if (raw[i] == last_raw[i]) stable[i] = raw[i];
            last_raw[i] = raw[i];
        }
        /* A USER_BUTTON that reads pressed from the start (never released) is ignored,
         * so the song code keys still work, and a warning is shown. */
        user_stuck = (!user_armed && now > pdMS_TO_TICKS(2000));
        if (user_stuck && !stuck_reported) {
            stuck_reported = 1;
            uart_print("USER_BUTTON reads pressed all the time, so it is ignored. Check its wiring\r\n"
                       "(use two legs on opposite sides of the button) or try another pin.\r\n");
        }

        int bits = (stable[1] << 2) | (stable[2] << 1) | stable[3];      /* LEFT = 4, DOWN = 2, RIGHT = 1 */

        /* --- button 1 (UP) with the song code on buttons 2-4  --- */
        if (stable[0] && !prev[0]) up_bits = 0;          /* UP went down */
        if (stable[0]) up_bits |= bits;                  /* collect the code */
        if (!stable[0] && prev[0]) on_up_tap(up_bits);   /* UP released */

        /* --- USER_BUTTON: tap = stop / play, hold 1.5 s = faster tempo --- */
        if (stable[4] && !prev[4]) {
            user_down_at = now;
            user_long_done = 0;
        }
        if (stable[4] && !user_long_done &&
            (now - user_down_at) >= pdMS_TO_TICKS(1500)) {
            user_long_done = 1;                          /* held 1.5 s: faster tempo */
            change_speed();
        }
        if (!stable[4] && prev[4] && !user_long_done) {  /* tapped and released */
            toggle_play();
        }

        if (confirm_expired) {
            confirm_expired = 0;
            uart_print("No confirmation: the current song carries on.\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ------------------------------------------------------------------ */
/* LCD + LEDs thread                                                    */
/* ------------------------------------------------------------------ */

/* Handout name: update_lcd_leds_thread */
static void update_lcd_leds_thread(void *arg)
{
    (void)arg;
    int shown_song = -1, shown_confirm = -1, shown_status = -1, shown_scale = -1, shown_held = -1;
    char line[40];                        /* longest text: the tempo line */

    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    lcd_init();
    lcd_fill(LCD_BLUE);                                      /* clear the display */
    lcd_draw_string(0, 2, "MP3 PLAYER", 3, LCD_WHITE, LCD_BLUE);   /* 10 chars x 24 px = 240 px */
    draw_small(212, "UP=select/OK  USER=stop/play", LCD_WHITE);
    xSemaphoreGive(lcd_mutex);
    lcd_ready = 1;
    if (volume_pct >= 0) draw_volume(volume_pct);
    leds_set(0, 0, 1);

    for (;;) {
        int confirming = (ui_state == UI_CONFIRM);
        int song       = confirming ? pending_song : cur_song;
        int playing    = player_playing;
        int audio      = audio_state;
        int secs       = 0;
        int status;

        if (confirming) {
            TickType_t now = xTaskGetTickCount();
            TickType_t left = (confirm_deadline > now) ? (confirm_deadline - now) : 0;
            secs = (int)(left / configTICK_RATE_HZ) + 1;
        }

        if (audio >= 2)       status = 100 + audio;
        else if (audio == 0)  status = 90;
        else if (confirming)  status = 50 + secs;
        else if (playing)     status = 1;
        else                  status = (note_idx > 0) ? 2 : 0;       /* 2 = paused, 0 = stopped */

        xSemaphoreTake(lcd_mutex, portMAX_DELAY);

        if (song != shown_song || confirming != shown_confirm) {
            int n;
            shown_song = song;
            shown_confirm = confirming;
            n = put_str(line, 0, confirming ? "CHANGE TO " : "SONG ");   /* "SONG n   bbb" */
            n = put_uint(line, n, song + 1);
            n = put_str(line, n, confirming ? " " : "   ");
            bits_to_text(song, &line[n]);
            draw_field(52,  line, confirming ? LCD_GREEN : LCD_YELLOW);
            draw_field(84,  song_name1(song), LCD_WHITE);
            draw_field(116, song_name2(song), LCD_WHITE);
        }

        if (status != shown_status) {
            shown_status = status;
            if (status >= 102) {
                memcpy(line, "AUDIO ERROR  ", 13);
                line[12] = (char)('0' + audio);
                line[13] = 0;
                draw_field(148, line, LCD_RED);
            } else if (status == 90) {
                draw_field(148, "STARTING...", LCD_WHITE);
            } else if (confirming) {
                memcpy(line, "CONFIRM: UP ", 12);             /* "CONFIRM: UP 5s" */
                line[12] = (char)('0' + (secs > 9 ? 9 : secs));
                line[13] = 's';
                line[14] = 0;
                draw_field(148, line, LCD_GREEN);
            } else if (status == 1) {
                draw_field(148, "PLAYING", LCD_GREEN);
            } else if (status == 2) {
                draw_field(148, "PAUSED", LCD_YELLOW);
            } else {
                draw_field(148, "STOPPED", LCD_YELLOW);
            }

            if (confirming)     leds_set(0, 1, 0);          /* changing song: green */
            else if (playing)   leds_set(1, 0, 0);          /* playing: yellow      */
            else                leds_set(0, 0, 1);          /* paused / stopped: red */
        }

        /* live keys: shows which keys are pressed right now (raw, not debounced) */
        int warn = user_stuck;
        int held = (key_pressed(KEY_UP_PIN) << 4) | (key_pressed(KEY_LEFT_PIN) << 3) |
                   (key_pressed(KEY_DOWN_PIN) << 2) | (key_pressed(KEY_RIGHT_PIN) << 1) |
                   (warn ? 0 : user_pressed());
        int shown = held | (warn << 8);
        if (shown != shown_held) {
            shown_held = shown;
            if (held == 0) {
                if (warn) draw_small(212, "USER BTN STUCK: CHECK WIRING", LCD_RED);
                else      draw_small(212, "UP=select/OK  USER=stop/play", LCD_WHITE);
            } else {
                char t[34];
                int n = put_str(t, 0, "HELD:");
                if (held & 16) n = put_str(t, n, " UP");
                if (held & 8)  n = put_str(t, n, " LEFT");
                if (held & 4)  n = put_str(t, n, " DOWN");
                if (held & 2)  n = put_str(t, n, " RIGHT");
                if (held & 1)  n = put_str(t, n, " USER");
                t[n] = 0;
                draw_small(212, t, LCD_YELLOW);
            }
        }

        int sc = (int)(beat_scale * 10.0f + 0.5f);
        if (sc != shown_scale) {
            char num[8];
            int n;
            shown_scale = sc;
            scale_to_text(beat_scale, num);
            n = put_str(line, 0, "TEMPO ");
            n = put_str(line, n, num);
            n = put_str(line, n, "  USER 1.5s=faster");
            line[n] = 0;
            draw_small(224, line, LCD_WHITE);
        }

        xSemaphoreGive(lcd_mutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void)
{
    HAL_Init();
    HAL_DBGMCU_EnableDBGSleepMode();      /* keep the debugger working while the CPU sleeps */
    SystemClock_Config();
    leds_init();

    uart_mutex = xSemaphoreCreateMutex();
    lcd_mutex  = xSemaphoreCreateMutex();
    uart_init();
    pot_init();

    note_timer    = xTimerCreate("notes",   1,                       pdFALSE, NULL, player_tick);
    confirm_timer = xTimerCreate("confirm", pdMS_TO_TICKS(CONFIRM_MS), pdFALSE, NULL, confirm_timeout);

    xTaskCreate(audio_task,              "audio",   512, NULL, 1, NULL);
    xTaskCreate(update_lcd_leds_thread,  "lcd",     512, NULL, 1, NULL);
    xTaskCreate(polling_buttons,         "buttons", 512, NULL, 1, NULL);
    xTaskCreate(adjust_volume,           "volume",  512, NULL, 1, NULL);
    vTaskStartScheduler();

    for (;;) { }
}