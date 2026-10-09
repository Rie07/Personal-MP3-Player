#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void vApplicationIdleHook(void) { }

static void test_task(void *arg)
{
    (void)arg;
    for (;;) { vTaskDelay(pdMS_TO_TICKS(500)); }
}

int main(void)
{
    xTaskCreate(test_task, "test", 128, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;) { }
}