/*
 * BCA182 Lab 2 - Personal MP3 Player
 * Step 3a: LED test. Proves the upload works and FreeRTOS is running.
 * Wiring (from you): Green = PB14, Yellow = PA1, Red = PA8 (active HIGH).
 */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#define LED_GREEN_PIN   GPIO_PIN_14   /* GPIOB */
#define LED_YELLOW_PIN  GPIO_PIN_1    /* GPIOA */
#define LED_RED_PIN     GPIO_PIN_8    /* GPIOA */

extern void xPortSysTickHandler(void);

/* SysTick drives both the HAL tick and the FreeRTOS tick. */
void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void vApplicationIdleHook(void) { }

static void leds_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = LED_GREEN_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin = LED_YELLOW_PIN | LED_RED_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(GPIOB, LED_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_YELLOW_PIN | LED_RED_PIN, GPIO_PIN_RESET);
}

static void led_test_task(void *arg)
{
    (void)arg;
    for (;;) {
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(300));
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_PIN, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOA, LED_YELLOW_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(300));
        HAL_GPIO_WritePin(GPIOA, LED_YELLOW_PIN, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOA, LED_RED_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(300));
        HAL_GPIO_WritePin(GPIOA, LED_RED_PIN, GPIO_PIN_RESET);
    }
}

int main(void)
{
    HAL_Init();
    leds_init();

    xTaskCreate(led_test_task, "led_test", 128, NULL, 1, NULL);
    vTaskStartScheduler();

    for (;;) { }
}