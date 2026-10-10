/*
 * BCA182 Lab 2 - Personal MP3 Player
 * Step 4b: ES8388 codec setup + I2S3 (DMA) + a steady test tone.
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

#define LED_YELLOW_PIN   GPIO_PIN_14   /* GPIOB */
#define LED_GREEN_PIN    GPIO_PIN_1    /* GPIOA */
#define LED_RED_PIN      GPIO_PIN_8    /* GPIOA */

/* 0 -> SCL = PF1, SDA = PF0      1 -> SCL = PB10, SDA = PB11 */
#define CODEC_I2C_PINSET 0

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

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void vApplicationIdleHook(void) { }

static void fatal(void) { for (;;) { } }

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) fatal();

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
    g.Pin = GPIO_PIN_3 | GPIO_PIN_5;          /* PB3 = CK, PB5 = SD */
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_7;                       /* PC7 = MCK  */
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

static void tone_fill(void)
{
    /* Triangle wave, 100 frames per cycle (441 Hz). Set TONE_AMPLITUDE to 0
     * for a silence test: any sound left over then is not coming from the data. */
    for (int i = 0; i < TONE_FRAMES; i++) {
        int p = i % 100;
        int s = (p < 50) ? (-TONE_AMPLITUDE + (2 * TONE_AMPLITUDE * p) / 50)
                         : ( TONE_AMPLITUDE - (2 * TONE_AMPLITUDE * (p - 50)) / 50);
        tone_buf[2 * i]     = (uint16_t)(int16_t)s;   /* left  */
        tone_buf[2 * i + 1] = (uint16_t)(int16_t)s;   /* right */
    }
}

static void audio_i2s_init(void)
{
    RCC_PeriphCLKInitTypeDef pc = {0};

    /* PLLI2S: 1 MHz x 429 / 4 = 107.25 MHz -> about 44.1 kHz with MCLK on */
    pc.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    pc.PLLI2S.PLLI2SN       = 429;
    pc.PLLI2S.PLLI2SR       = 4;
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

    tone_fill();
    audio_i2s_init();
    if (HAL_I2S_Transmit_DMA(&hi2s3, tone_buf, TONE_FRAMES * 2) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, LED_YELLOW_PIN, GPIO_PIN_SET);
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    vTaskDelay(pdMS_TO_TICKS(50));            /* MCLK/BCLK/LRCK are running */
    es8388_init();

    if (es_err == 0) {
        HAL_GPIO_WritePin(GPIOA, LED_GREEN_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, LED_RED_PIN, GPIO_PIN_SET);
    }

    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    leds_init();

    xTaskCreate(audio_task, "audio", 512, NULL, 1, NULL);
    vTaskStartScheduler();

    for (;;) { }
}